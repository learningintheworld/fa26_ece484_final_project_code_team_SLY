"""Local RGB association audit using saved v3 candidates, never a driving node.

Run in the existing ROS container with source package on PYTHONPATH:
python scripts/analyze_physical_identity.py --output results/physical_identity_v1
No full replay, model training, ROS publishers or parameter changes.
"""
import argparse,copy,hashlib,json,sqlite3,time
from collections import Counter
from pathlib import Path
import cv2,numpy as np,yaml
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from sly_lane_following.core import camera_maps
from sly_lane_following.geometry import GroundProjector
from sly_lane_following.physical_identity import PhysicalStripeTracker


def main():
 root=Path(__file__).resolve().parents[1]
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--output',type=Path,required=True);p.add_argument('--bag',type=Path,default=Path('/data/gem/oak_front_image'))
 p.add_argument('--cache',type=Path,default=root/'results/unified_handoff_v1/full/frames.jsonl')
 args=p.parse_args();out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
 if args.bag.resolve() in out.parents:raise ValueError('OUTPUT_INSIDE_RAW_BAG')
 cfg=yaml.safe_load((root/'src/sly_lane_following/config/e4_pure_pursuit.yaml').read_text());P=GroundProjector(cfg)
 camera=json.loads((root/'results/stage2/reference/camera.json').read_text());maps,_=camera_maps(camera,(480,768,3))
 rows=[json.loads(l) for l in args.cache.open()]
 manifest=json.loads((root/'results/unified_handoff_v1/start_hashes.json').read_text())
 hashes={f:hashlib.sha256((root/f).read_bytes()).hexdigest() for f in manifest}
 assert hashes==manifest,'Candidate cache no longer matches protected algorithm/config snapshot'
 info=yaml.safe_load((args.bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
 db=sqlite3.connect((args.bag/info['relative_file_paths'][0]).resolve().as_uri()+'?mode=ro',uri=True)
 topic=db.execute("select id from topics where name='/oak/rgb/image_raw'").fetchone()[0]
 base=db.execute('select min(timestamp) from messages').fetchone()[0];bridge=CvBridge()
 def fetch(r):
  ts=base+round(r['time_s']*1e9)
  data=db.execute('select data from messages where topic_id=? and timestamp between ? and ? order by abs(timestamp-?) limit 1',(topic,ts-1000,ts+1000,ts)).fetchone()[0]
  msg=deserialize_message(data,Image)
  assert msg.header.stamp.sec*10**9+msg.header.stamp.nanosec==r['stamp_ns']
  return cv2.remap(bridge.imgmsg_to_cv2(msg,'bgr8'),*maps,cv2.INTER_LINEAR)
 targets=[37.402,38.903,39.411,39.504,39.602,40.502,41.2,42.001,44.001,12.402,65.803,108.908,109.5,112.704]
 snapshots={min(rows,key=lambda r:abs(r['time_s']-t))['stamp_ns'] for t in targets}
 windows=[('left_turn',36.9,44.1),('normal',12.,13.),('gap65',65.2,66.4),('gap108',108.3,109.6),('handoff112',112.3,112.9)]
 records=[];summary={};files=[]
 for name,lo,hi in windows:
  tracker=PhysicalStripeTracker(P);local=[]
  for r in rows:
   if not lo-.25<=r['time_s']<=hi:continue
   rgb=fetch(r);gray=cv2.cvtColor(rgb,cv2.COLOR_BGR2GRAY)
   # Only a currently accepted, actually present boundary supplies a role.
   c=r['base_control'];ids={b['id'] for b in r['candidates']};roles={}
   if c['valid']:
    for side,identity in r['prediction']['boundary_ids'].items():
     if identity in ids and (c.get('observed_id') is None or identity==c.get('observed_id')):roles[identity]=side
   result=dict(candidates=r['candidates'],selected=[],center=r['prediction']['path'],command=r['command'])
   before=copy.deepcopy(result);start=time.perf_counter()
   after=tracker.annotate(result,gray,camera,r['stamp_ns'],confirmed_roles=roles,excluded_ids=r['prediction']['blocked_neighbor_ids'])
   elapsed=(time.perf_counter()-start)*1000
   assert result==before and all(after[k]==v for k,v in before.items())
   identity=after['physical_identity']
   rec=dict(time_s=r['time_s'],stamp_ns=r['stamp_ns'],segment=name,detector_session=r['segment'],
            confirmed_roles=roles,before_candidates=r['candidates'],before_command=r['command'],
            after_command=after['command'],center_changed=after['center']!=before['center'],
            physical=identity,processing_ms=elapsed)
   if r['time_s']>=lo:local.append(rec);records.append(rec)
   if r['stamp_ns'] in snapshots:
    panels=[rgb.copy() for _ in range(3)]
    for b in r['candidates']:
     pts=np.rint(b['points']).astype('int32');cv2.polylines(panels[1],[pts],False,(255,180,20),2)
     cv2.putText(panels[1],str(b['id']),tuple(pts[len(pts)//2]),0,.6,(0,0,255),2)
    for g in identity['groups']:
     color=(40,210,40) if g['role'] else (0,170,255)
     pts=np.rint(g['points']).astype('int32');cv2.polylines(panels[2],[pts],False,color,2)
     label='P%d [%s] %s'%(g['physical_id'],','.join(map(str,g['source_ids'])),g['role'] or 'role ?')
     anchor=pts[len(pts)//2];anchor[0]=max(5,min(int(anchor[0]),520));anchor[1]=max(100,min(int(anchor[1])-15,465))
     cv2.putText(panels[2],label,tuple(anchor),0,.45,color,1)
     for join in g['joins']:
      cv2.polylines(panels[2],[np.rint(join['points']).astype('int32')],False,(255,0,255),3)
    for im,title in zip(panels,['Recorded RGB (undistorted)','Before: v3 candidate fragments','After: physical line; purple=current paint bridge']):
     cv2.rectangle(im,(0,0),(768,68),(255,255,255),-1)
     cv2.putText(im,title,(8,24),0,.5,(0,0,0),1)
     cv2.putText(im,f'{r["time_s"]:.3f}s | no new driving authorization',(8,51),0,.48,(0,0,0),1)
    file=f'{r["time_s"]:.3f}_compare.jpg';cv2.imwrite(str(out/file),np.hstack(panels));files.append(file)
  s=dict(frames=len(local),joined_frames=sum(any(g['joins'] for g in r['physical']['groups']) for r in local),
    motion_accepted=sum(r['physical']['motion']['valid'] for r in local),
    linked_pairs=[dict(time_s=r['time_s'],ids=g['source_ids'],physical_id=g['physical_id'],role=g['role'],role_state=g['role_state']) for r in local for g in r['physical']['groups'] if g['joins']],
    center_changes=sum(r['center_changed'] for r in local),command_changes=sum(r['before_command']!=r['after_command'] for r in local),
    processing_ms_median=float(np.median([r['processing_ms'] for r in local])),processing_ms_p95=float(np.percentile([r['processing_ms'] for r in local],95)))
  summary[name]=s;print(name,'frames',s['frames'],'joined',s['joined_frames'],'motion',s['motion_accepted'],flush=True)
 (out/'frames.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in records))
 (out/'summary.json').write_text(json.dumps(summary,indent=2));(out/'protected_hashes.json').write_text(json.dumps(hashes,indent=2))
 assert all(hashlib.sha256((root/f).read_bytes()).hexdigest()==v for f,v in hashes.items())
 (out/'index.html').write_text('<!doctype html><meta charset="utf-8"><style>body{font:18px/1.6 sans-serif;margin:30px}img{max-width:100%}</style><h1>Physical stripe connections — local comparison</h1><p>P编号是物理观测身份；原数字ID保留。紫色连接来自当前原图白漆，不是外推。role ?表示尚未批准左右角色，未新增驾驶指令。</p>'+''.join(f'<h2>{file}</h2><img src="{file}">' for file in files))
 print('saved',out,flush=True)
if __name__=='__main__':main()
