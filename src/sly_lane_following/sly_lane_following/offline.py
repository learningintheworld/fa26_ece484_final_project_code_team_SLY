"""Read-only bag analysis, using exactly the same core as the ROS adapter."""
import argparse
import math
from collections import Counter
import hashlib
import json
from pathlib import Path
import sqlite3
import subprocess
import time
import shutil

import cv2
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, Image

from .core import LaneDetector


def stamp(message):
    return message.header.stamp.sec*10**9+message.header.stamp.nanosec


def bag_images(bag):
    metadata=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    files=metadata['relative_file_paths']
    # This first implementation deliberately supports one SQLite bag file.
    if len(files)!=1 or not files[0].endswith('.db3'):
        raise ValueError('Expected the validated single-file SQLite bag')
    db=(bag/files[0]).resolve()
    with sqlite3.connect(db.as_uri()+'?mode=ro',uri=True) as conn:
        ids=dict(conn.execute('select name,id from topics'))
        start=conn.execute('select min(timestamp) from messages').fetchone()[0]
        infos={}
        for (blob,) in conn.execute('select data from messages where topic_id=?',(ids['/oak/rgb/camera_info'],)):
            m=deserialize_message(blob,CameraInfo)
            infos[stamp(m)]=(dict(width=m.width,height=m.height,K=list(m.k),D=list(m.d),distortion_model=m.distortion_model),m.header.frame_id)
        bridge=CvBridge()
        for t,blob in conn.execute('select timestamp,data from messages where topic_id=? order by timestamp',(ids['/oak/rgb/image_raw'],)):
            m=deserialize_message(blob,Image)
            camera,frame=infos.get(stamp(m),({},None))
            if frame!=m.header.frame_id: camera={}
            yield (t-start)/1e9,stamp(m),bridge.imgmsg_to_cv2(m,'bgr8'),camera


def evaluate_reference(record, reference, camera):
    if reference['status']!='clear':
        return dict(reference_status=reference['status'], accepted_nonclear=bool(record['vision_valid']))
    if not record['vision_valid']:return dict(reference_status='clear',accepted=False)
    errors=[];widths=[]
    k=np.asarray(camera['K']).reshape(3,3);d=np.asarray(camera['D'])
    for side in ('left','right'):
        points=np.asarray(reference[side],np.float64).reshape(-1,1,2)
        if not points.size:continue
        expected=cv2.undistortPoints(points,k,d,P=k).reshape(-1,2)
        actual=np.asarray(record[side]);other=np.asarray(record['right' if side=='left' else 'left'])
        for x,y in expected:
            if actual[0,1]<=y<=actual[-1,1]:
                predicted=np.interp(y,actual[:,1],actual[:,0])
                opposite=np.interp(y,other[:,1],other[:,0])
                errors.append(abs(float(predicted-x)));widths.append(abs(float(predicted-opposite)))
    if not errors:return dict(reference_status='clear',accepted=False,comparable=False,
                              vision_output_valid=True,unassessable=True)
    # Coarse reference: wrong corridor if boundary distance exceeds both manual
    # annotation uncertainty and a meaningful fraction of the visible corridor.
    wrong=any(e>max(25.,.20*w) for e,w in zip(errors,widths))
    return dict(reference_status='clear',accepted=True,comparable=True,
                boundary_error_px=float(np.mean(errors)),max_boundary_error_px=max(errors),
                wrong_corridor_proxy=wrong,annotation_tolerance_px=10)


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bag',type=Path,default=Path('/data/gem/oak_front_image'))
    p.add_argument('--output',type=Path,default=Path('results/stage2/run'))
    p.add_argument('--config',type=Path,default=Path(get_package_share_directory('sly_lane_following'))/'config/real_bag.yaml')
    p.add_argument('--reference',type=Path)
    p.add_argument('--start',type=float,default=0.,help='Explicit new perception session at this bag time; not continuous recovery')
    p.add_argument('--end',type=float,default=float('inf'))
    p.add_argument('--no-video',action='store_true')
    args=p.parse_args(argv)
    bag=args.bag.resolve();out=args.output.resolve()
    if out==bag or bag in out.parents:raise ValueError('Output must not be inside the read-only bag')
    out.mkdir(parents=True,exist_ok=True)
    if (out/'frames.jsonl').exists():raise FileExistsError('Choose a new output directory; previous results are preserved')
    config=yaml.safe_load(args.config.read_text());detector=LaneDetector(config)
    provenance=out/'provenance';provenance.mkdir(exist_ok=True)
    for module in ('core.py','offline.py'):
        shutil.copyfile(Path(__file__).with_name(module),provenance/module)
    shutil.copyfile(args.config,provenance/'config.yaml')
    references={}
    if args.reference:
        references={int(f['second']):f for f in json.loads(args.reference.read_text())['frames']}
        shutil.copyfile(args.reference,provenance/'reference.json')
    counts=Counter();valid=Counter();reasons=Counter();durations=[];ref_results=[]
    longest_invalid=0.;invalid_start=None;last_center=None;last_valid_t=None;jump_candidates=[]
    processed=0;writer=None;video_backend=None;start_wall=time.perf_counter();next_reference=math.ceil(args.start)
    try:
        with (out/'frames.jsonl').open('w') as records:
            for relative,ts,image,camera in bag_images(bag):
                if relative<args.start:continue
                if relative>args.end:break
                result=detector.process(image,camera,ts);debug=result.pop('debug_image')
                split='tune' if relative<60 else 'gap' if relative<70 else 'validation' # historical split, now regression only
                result.update(relative_s=relative,split=split)
                counts[split]+=1;valid[split]+=int(result['vision_valid']);reasons[result['reason']]+=1
                durations.append(result['processing_ms']);processed+=1
                if result['vision_valid']:
                    if invalid_start is not None:longest_invalid=max(longest_invalid,relative-invalid_start)
                    invalid_start=None
                    c=np.asarray(result['center'])
                    if last_center is not None and relative-last_valid_t<.2:
                        lo=max(c[0,1],last_center[0,1]);hi=min(c[-1,1],last_center[-1,1])
                        if hi-lo>20:
                            ys=np.linspace(lo,hi,10)
                            jump=float(np.median(np.abs(np.interp(ys,c[:,1],c[:,0])-np.interp(ys,last_center[:,1],last_center[:,0]))))
                            if jump>30:jump_candidates.append(dict(relative_s=relative,median_shift_px=jump,split=split))
                    last_center,last_valid_t=c,relative
                elif invalid_start is None:invalid_start=relative
                if relative>=next_reference:
                    reference=references.get(next_reference)
                    if reference:
                        if reference.get('header_ns',ts)!=ts:
                            raise ValueError('Reference frame timestamp mismatch; do not score another frame')
                        assessment=evaluate_reference(result,reference,camera)
                        assessment.update(second=next_reference,split=split,reason=result['reason'])
                        ref_results.append(assessment)
                    cv2.putText(debug,f'{relative:.2f}s | {split}',(10,85),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,255),1)
                    cv2.imwrite(str(out/f'frame_{next_reference:03d}.jpg'),debug)
                    next_reference+=1
                records.write(json.dumps(result,separators=(',',':'))+'\n')
                if not args.no_video:
                    if writer is None:
                        h,w=debug.shape[:2]
                        if shutil.which('ffmpeg'):
                            writer=subprocess.Popen(['ffmpeg','-loglevel','error','-y','-f','rawvideo','-pixel_format','bgr24','-video_size',f'{w}x{h}',
                                '-framerate','30','-i','pipe:0','-an','-c:v','libx264','-preset','veryfast','-crf','23','-pix_fmt','yuv420p','-movflags','+faststart',str(out/'overlay.mp4')],stdin=subprocess.PIPE)
                            video_backend='ffmpeg-h264'
                        else:
                            writer=cv2.VideoWriter(str(out/'overlay.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),30,(w,h))
                            if not writer.isOpened():raise RuntimeError('OpenCV video encoder unavailable; use --no-video')
                            video_backend='opencv-mp4v'
                    cv2.putText(debug,f'{relative:.2f}s | {split}',(10,85),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,255),1)
                    if video_backend=='ffmpeg-h264':writer.stdin.write(debug.tobytes())
                    else:writer.write(debug)
                if processed%300==0:print(f'{processed} frames, t={relative:.1f}s',flush=True)
            if invalid_start is not None:longest_invalid=max(longest_invalid,relative-invalid_start)
    finally:
        if writer:
            if video_backend=='ffmpeg-h264':
                writer.stdin.close()
                if writer.wait()!=0:raise RuntimeError('ffmpeg failed')
            else:writer.release()
    val=[r for r in ref_results if r['split']=='validation']
    clear=[r for r in val if r['reference_status']=='clear']
    accepted=[r for r in clear if r.get('accepted')]
    errors=[r['boundary_error_px'] for r in accepted if r.get('comparable')]
    wrong=[r for r in accepted if r.get('wrong_corridor_proxy')]
    unassessable=[r for r in clear if r.get('unassessable')]
    nonclear=[r for r in val if r.get('accepted_nonclear')]
    summary=dict(evaluation_kind="known-data regression; not independent validation",explicit_session_start_s=args.start,frames=processed,counts=dict(counts),valid=dict(valid),reasons=dict(reasons),
        coverage={k:valid[k]/v for k,v in counts.items()},longest_invalid_s=longest_invalid,
        processing_ms_median=float(np.median(durations)),processing_ms_p95=float(np.percentile(durations,95)),
        wall_seconds=time.perf_counter()-start_wall,jump_candidates=jump_candidates,
        validation_reference_clear=len(clear),validation_reference_accepted=len(accepted),
        validation_reference_coverage=len(accepted)/len(clear) if clear else None,
        validation_boundary_error_px_mean=float(np.mean(errors)) if errors else None,
        wrong_corridor_proxy=wrong,accepted_nonclear=nonclear,
        validation_unassessable=unassessable,
        metric_valid=False,metric_reason='NOT_USED_IMAGE_SERVO',
        config=config,config_sha256=hashlib.sha256(args.config.read_bytes()).hexdigest(),
        core_sha256=hashlib.sha256(Path(__file__).with_name('core.py').read_bytes()).hexdigest(),
        reference_sha256=hashlib.sha256(args.reference.read_bytes()).hexdigest() if args.reference else None,
        opencv=cv2.__version__,numpy=np.__version__,video_backend=video_backend,review_required=True)
    (out/'summary.json').write_text(json.dumps(summary,indent=2))
    (out/'reference_evaluation.json').write_text(json.dumps(ref_results,indent=2))
    print(json.dumps({k:summary[k] for k in ('frames','coverage','validation_reference_coverage','processing_ms_p95','metric_reason')}))


if __name__=='__main__':main()
