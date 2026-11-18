#!/usr/bin/env python3
"""Prepare grouped full-lap RGB selection and a 20-image approval checkpoint.
No model inference, no training, and no writes to the source SQLite bag.
"""
import argparse
from collections import Counter
import hashlib
import html
import json
from pathlib import Path
import sqlite3
import cv2
import numpy as np
import yaml
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from sly_lane_following.core import camera_maps
from sly_lane_following.night_augmentation import make_training_view, VERSION


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def scene(t):
    for a,b,name in [(0,20,'straight_multi'),(20,45,'left_curve_single'),(45,60,'curve'),
                     (60,76,'broken_multi'),(76,95,'straight'),(95,105,'pair_single_transition'),
                     (105,118,'curve_identity'),(118,130,'straight_exit')]:
        if a<=t<b:return name
    return 'unknown'


def select_frames(rows, anchors, counts=(350,75,75), merge_audit=None, review_times=None):
    """Allocate whole 3s groups across the lap; never split neighbouring frames."""
    times = np.array([r[1] for r in rows])
    groups = (times//3).astype(int)
    rng=np.random.default_rng(484)
    names=('train','validation','evaluation')
    # Each successive ~30s part contributes groups to every split.
    assignment={}
    unique=sorted(set(groups))
    for part in np.array_split(unique,4):
        shuffled=rng.permutation(part)
        nv=max(1,round(len(part)*.16)); ne=max(1,round(len(part)*.16))
        for j,g in enumerate(shuffled):
            assignment[int(g)]='validation' if j<nv else 'evaluation' if j<nv+ne else 'train'
    # Merge audit-identified near-static groups before selecting any split samples.
    if merge_audit:
        prior=json.loads(Path(merge_audit).read_text());by={f['id']:f for f in prior['frames']}
        parent={g:g for g in unique}
        def find(g):
            while parent[g]!=g:g=parent[g]
            return g
        for pair in prior['cross_split_near_duplicates']:
            a,b=by[pair['a']]['group_id'],by[pair['b']]['group_id']
            parent[find(a)]=find(b)
        clusters={}
        for g in unique:clusters.setdefault(find(g),[]).append(g)
        for members in clusters.values():
            winner=Counter(assignment[g] for g in members).most_common(1)[0][0]
            for g in members:assignment[g]=winner
    changes=[3*g for g in unique[1:] if assignment[g]!=assignment[g-1]]
    eligible=np.ones(len(rows),bool)
    # >=0.5s separation at different-split boundaries (0.25s on either side).
    for boundary in changes:eligible &= np.abs(times-boundary)>=.251
    selected=[]; anchor_set=set()
    # Review images must be actual members of the new selection, not extra data.
    old_stamps=set(anchors)
    available=[i for i,r in enumerate(rows) if r[2] in old_stamps and eligible[i]]
    for t in review_times or []:
        pool=[i for i in available if i not in anchor_set]
        if pool:anchor_set.add(min(pool,key=lambda i:abs(times[i]-t)))
    for name,target in zip(names,counts):
        pool=[i for i in range(len(rows)) if eligible[i] and assignment[int(groups[i])]==name]
        chosen=sorted(i for i in anchor_set if i in pool)
        base=round(target*.70)
        # Farthest-in-time selection covers every eligible portion of the lap.
        while len(chosen)<base:
            remaining=[i for i in pool if i not in chosen]
            i=max(remaining,key=lambda k:min(abs(times[k]-times[j]) for j in chosen)) if chosen else remaining[len(remaining)//2]
            chosen.append(i)
        hard=[i for i in pool if scene(times[i]) in {'left_curve_single','broken_multi','pair_single_transition','curve_identity'}]
        while len(chosen)<target:
            remaining=[i for i in hard if i not in chosen]
            if not remaining:remaining=[i for i in pool if i not in chosen]
            chosen.append(max(remaining,key=lambda k:min(abs(times[k]-times[j]) for j in chosen)))
        selected += [dict(index=int(i),record_ns=int(rows[i][0]),relative_s=float(times[i]),stamp_ns=int(rows[i][2]),
                          group_id=int(groups[i]),split=name,scene=scene(times[i]),selection='uniform_or_review' if j<base else 'difficulty_dense') for j,i in enumerate(chosen)]
    selected.sort(key=lambda f:f['relative_s'])
    # Check precise temporal separation across all selected splits.
    for a,b in zip(selected,selected[1:]):
        if a['split']!=b['split'] and b['relative_s']-a['relative_s']<.5:
            raise AssertionError('Cross-split temporal guard failed')
    return selected,assignment


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bag',type=Path,default=Path('/data/gem/oak_front_image'))
    p.add_argument('--output',type=Path,default=Path('results/night_v1'))
    p.add_argument('--merge-audit',type=Path)
    args=p.parse_args(); root=Path.cwd();out=args.output.resolve()
    out.mkdir(parents=True,exist_ok=False)
    for d in ('images','review/labels','review/panels','review/light','review/medium','review/clahe'):(out/d).mkdir(parents=True)
    oldroot=root/'results/enet_real_v4_candidate'
    old=json.loads((oldroot/'annotations.json').read_text())['frames']
    old_by_stamp={f['stamp_ns']:f for f in old}
    review_times=[0,8,15,23.5,31.25,37,42,52.5,60,65,70,80,88,96,99,108,112,116,122,128]
    anchors=[f['stamp_ns'] for f in old]
    meta=yaml.safe_load((args.bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    if len(meta['relative_file_paths'])!=1:raise ValueError('Expected validated single db3')
    db=sqlite3.connect((args.bag/meta['relative_file_paths'][0]).resolve().as_uri()+'?mode=ro',uri=True)
    topic=db.execute("select id from topics where name='/oak/rgb/image_raw'").fetchone()[0]
    start=db.execute('select min(timestamp) from messages').fetchone()[0]
    # Serialized image headers are read without image decoding in this pass.
    rows=[]
    for t,blob in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(topic,)):
        m=deserialize_message(blob,Image);stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
        rows.append((t,(t-start)/1e9,stamp))
    selected,assignment=select_frames(rows,anchors,merge_audit=args.merge_audit,review_times=review_times)
    # If a preferred old image fell in an embargo, replace with closest eligible old label.
    chosen_stamps={f['stamp_ns'] for f in selected}
    available=[f for f in selected if f['stamp_ns'] in old_by_stamp]
    review=[]
    for t in review_times:
        pool=[f for f in available if f['stamp_ns'] not in {r['stamp_ns'] for r in review}]
        if not pool:break
        review.append(min(pool,key=lambda f:abs(f['relative_s']-t)))
    review_stamps={f['stamp_ns'] for f in review}
    camera=json.loads((root/'results/stage2/reference/camera.json').read_text())
    bridge=CvBridge();maps=None;thumbs=[];review_records=[]
    for j,f in enumerate(selected):
        blob=db.execute('select data from messages where topic_id=? and timestamp=?',(topic,f['record_ns'])).fetchone()[0]
        raw=bridge.imgmsg_to_cv2(deserialize_message(blob,Image),'bgr8')
        if maps is None:maps,_=camera_maps(camera,raw.shape)
        image=cv2.remap(raw,*maps,cv2.INTER_LINEAR)
        path=out/'images'/f'{j:04d}.png';cv2.imwrite(str(path),image)
        f.update(id=f'{j:04d}',image=str(path.relative_to(out)),image_sha256=digest(path),raw_sha256=hashlib.sha256(raw.tobytes()).hexdigest(),
                 source_image=str(path),camera=camera,preprocessing='undistorted at original resolution; no crop',label_status='pending',reviewed=False)
        thumbs.append(cv2.resize(cv2.cvtColor(image[275:],cv2.COLOR_BGR2GRAY),(64,24)).astype(np.float32))
        if f['stamp_ns'] not in review_stamps:continue
        source=old_by_stamp[f['stamp_ns']]
        labelpath=out/'review/labels'/f'{f["id"]}.png';labelpath.write_bytes((oldroot/source['label']).read_bytes())
        label=cv2.imread(str(labelpath),0)
        f.update(label=str(labelpath.relative_to(out)),label_sha256=digest(labelpath),label_status='historical_label_pending_visual_audit',
                 label_source=str(oldroot/source['label']),label_source_stamp=source['stamp_ns'])
        review_records.append(f)
    # Cross-split exact and near duplicate audit; broad threshold flags for review.
    near=[]
    for i,a in enumerate(selected):
        for j in range(i+1,len(selected)):
            b=selected[j]
            if a['split']==b['split']:continue
            mae=float(np.mean(np.abs(thumbs[i]-thumbs[j])))
            if a['image_sha256']==b['image_sha256'] or mae<2:
                near.append(dict(a=a['id'],b=b['id'],mae=mae))
    manifest=dict(schema='sly_night_dataset_v1',frozen=False,training_authorized=False,seed=484,
                  counts=dict(Counter(f['split'] for f in selected)),source_frames=len(rows),
                  group_seconds=3.,cross_split_gap_s=.5,group_assignment=assignment,
                  inference_roi_top=275/480,preprocessing='gray_letterbox_640x384_v1',
                  classes={'background':0,'lane':1,'ignore':255},frames=selected,
                  cross_split_near_duplicates=near,review_stamps=sorted(review_stamps),
                  augmentation_version=VERSION,training_start='MP1 epoch20, not v3',
                  merged_duplicate_groups_from=str(args.merge_audit) if args.merge_audit else None,
                  holdout_claim='same-recording grouped holdout; not unseen-night or unseen-course validation')
    (out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
    render_review(out)
    print(json.dumps(dict(counts=manifest['counts'],review_count=len(review_records),near_duplicates=len(near)),indent=2))


def render_review(out):
    manifest=json.loads((out/'manifest.json').read_text());review=[f for f in manifest['frames'] if f['stamp_ns'] in manifest['review_stamps']]
    manifest['augmentation_version']=VERSION
    manifest['user_review_approved']=False
    manifest['training_authorized']=False
    manifest['frozen']=False
    (out/'review/strong').mkdir(exist_ok=True)
    (out/'review/revisions').mkdir(exist_ok=True)
    parts=['<!doctype html><html lang="zh"><meta charset="utf-8"><title>夜间训练前审阅</title><style>body{font:18px sans-serif;margin:30px auto;max-width:1560px;padding:0 18px;color:#222}img{max-width:100%}article{border-top:2px solid #ddd;padding:18px 0}p{line-height:1.65}a{color:#376b86}</style><h1>第二版20组：更暗环境＋左右双车灯</h1><p>按反馈调低环境光；新版轻度接近旧中度，新中度更暗。左右车灯分别照亮近处，平滑重叠，远处与两侧更暗。模拟光束不是实测车灯标定。尚未训练。绿色：赛道白线；淡紫：忽略区域。上排原图/标签，下排轻度/中度夜间。照明模拟不使用标签位置。请检查灯光强度、白线漏标和干扰物误标。原有ROI保持y≥275。</p>']
    for n,f in enumerate(review,1):
        im=cv2.imread(str(out/f['image']));label=cv2.imread(str(out/f['label']),0)
        ov=im.copy();ov[label==1]=(.30*im[label==1]+.70*np.array([30,240,40])).astype(np.uint8)
        ov[label==255]=(.88*im[label==255]+.12*np.array([150,70,160])).astype(np.uint8)
        cv2.line(ov,(0,275),(767,275),(0,150,230),1)
        light,lmeta=make_training_view(im,label,'light',484+n)
        medium,mmeta=make_training_view(im,label,'medium',1484+n)
        strong,smeta=make_training_view(im,label,'strong',2484+n)
        f['review_augmentation']={'light':lmeta,'medium':mmeta,'strong':smeta}
        def title(image,text):
            head=np.full((32,image.shape[1],3),255,np.uint8);cv2.putText(head,text,(10,23),0,.62,(20,20,20),1,cv2.LINE_AA)
            return np.vstack([head,image])
        panel=np.vstack([np.hstack([title(im,f'{n:02d} | {f["relative_s"]:.3f}s | original'),title(ov,'annotation: green=paint / purple=ignore')]),
                         np.hstack([title(light,'v2 light: dusk-to-night / two headlights'),title(medium,'v2 medium: darker ambient / two headlights')])])
        cv2.imwrite(str(out/'review/panels'/f'{n:02d}.jpg'),panel,[cv2.IMWRITE_JPEG_QUALITY,93])
        for kind,image in [('light',light),('medium',medium),('strong',strong)]:cv2.imwrite(str(out/'review'/kind/f'{n:02d}.png'),image)
        gray=cv2.cvtColor(medium,cv2.COLOR_BGR2GRAY);cl=cv2.createCLAHE(clipLimit=2.,tileGridSize=(8,8)).apply(gray)
        cv2.imwrite(str(out/'review/clahe'/f'{n:02d}.jpg'),np.hstack([gray,cl]))
        old_path=out.parent/'review_revision1/review/medium'/f'{n:02d}.png'
        old=cv2.imread(str(old_path)) if old_path.exists() else None
        comparison_html=''
        if old is not None:
            compare=np.vstack([np.hstack([title(old,'previous medium'),title(light,'v2 light')]),
                               np.hstack([title(medium,'v2 medium'),title(strong,'v2 strong: 5 percent stress samples')])])
            cv2.imwrite(str(out/'review/revisions'/f'{n:02d}.jpg'),compare,[cv2.IMWRITE_JPEG_QUALITY,93])
            comparison_html=f'<details><summary>旧中度与新轻／中／较强对照</summary><img loading="lazy" src="revisions/{n:02d}.jpg"></details>'
        parts.append(f'<article><h2>{n:02d} · {f["relative_s"]:.3f}秒 · {f["split"]} · {f["scene"]}</h2><img loading="lazy" src="panels/{n:02d}.jpg"><p>{html.escape(f.get("audit_note","待复核历史标签"))}</p>{comparison_html}<details><summary>中度夜间灰度 / CLAHE 对照（不用于默认推理）</summary><img loading="lazy" src="clahe/{n:02d}.jpg"></details></article>')
    (out/'review/index.html').write_text(''.join(parts)+'</html>')
    (out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
    for j in range(0,len(review),4):
        tiles=[cv2.resize(cv2.imread(str(out/'review/panels'/f'{k+1:02d}.jpg')),(960,640)) for k in range(j,min(j+4,len(review)))]
        cv2.imwrite(str(out/'review'/f'contact_{j//4+1}.jpg'),np.vstack(tiles))


if __name__=='__main__':main()
