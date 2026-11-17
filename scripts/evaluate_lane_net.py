#!/usr/bin/env python3
"""Compare OpenCV, original MP1 and fine-tuned ENet on frozen pixel labels."""
import argparse, hashlib, json, time
from pathlib import Path
from collections import defaultdict
import cv2, numpy as np, torch, yaml
from sly_lane_following.core import LaneDetector, camera_maps
from sly_lane_following.neural import NeuralMask


def stats(tp,fp,fn):
    return dict(tp=tp,fp=fp,fn=fn,iou=tp/max(1,tp+fp+fn),dice=2*tp/max(1,2*tp+fp+fn),
                precision=tp/max(1,tp+fp),recall=tp/max(1,tp+fn))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--annotations',type=Path,required=True)
    p.add_argument('--pretrained',type=Path,required=True)
    p.add_argument('--checkpoint',type=Path,required=True)
    p.add_argument('--baseline-name', default='mp1_original')
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    cv2.setNumThreads(1);torch.set_num_threads(4)
    data=json.loads(args.annotations.read_text());assert data['frozen']
    cv=LaneDetector(yaml.safe_load(args.config.read_text()))
    models={'opencv':cv,args.baseline_name:NeuralMask(args.pretrained),'enet_finetuned':NeuralMask(args.checkpoint)}
    totals=defaultdict(lambda:np.zeros(3,np.int64));timings=defaultdict(list); records=[]; regions=defaultdict(lambda:np.zeros(3,np.int64))
    for f in data['frames']:
        if f['split'] not in ('validation','evaluation'):continue
        image=cv2.imread(f['source_image'].replace('/home/xinweiyu/Workspace/ECE484_Program/','/workspace/'))
        labelpath=args.annotations.parent/f['label']
        assert hashlib.sha256(labelpath.read_bytes()).hexdigest()==f['label_sha256']
        label=cv2.imread(str(labelpath),0);valid=label!=255;truth=label==1
        cv.maps,_=camera_maps(f['camera'],image.shape)
        panels=[]
        for name,model in models.items():
            t=time.perf_counter();pred=(model.mask(image) if name=='opencv' else model(image))>0
            ms=(time.perf_counter()-t)*1000
            tp=int((pred&truth&valid).sum());fp=int((pred&~truth&valid).sum());fn=int((~pred&truth&valid).sum())
            totals[(f['split'],name)]+=np.array([tp,fp,fn]);timings[name].append(ms)
            for region,y0,y1 in [('far',275,330),('near',330,480)]:
                v=valid[y0:y1];pr=pred[y0:y1];tr=truth[y0:y1]
                regions[(f['split'],name,region)]+=np.array([int((pr&tr&v).sum()),int((pr&~tr&v).sum()),int((~pr&tr&v).sum())])
            records.append(dict(second=f['second'],split=f['split'],method=name,ms=ms,**stats(tp,fp,fn)))
            over=image.copy()
            # Overlay only independently labelled pixels; ignored areas never enter the score.
            over[pred&valid]=(.4*over[pred&valid]+.6*np.array([0,255,0])).astype(np.uint8)
            over[~pred&truth]=(.4*over[~pred&truth]+.6*np.array([0,0,255])).astype(np.uint8)
            cv2.line(over,(0,330),(767,330),(0,220,255),1)
            header=np.full((30,768,3),245,np.uint8)
            cv2.putText(header,f'{name} | {f["second"]}s | IoU {stats(tp,fp,fn)["iou"]:.3f}',(8,21),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,0,0),1)
            panels.append(np.vstack([header,over[250:]]))
        cv2.imwrite(str(args.output/f'comparison_{f["second"]:03}.jpg'),np.vstack(panels))
    report=dict(evaluation_scope='Same-recording temporal holdout; supervised original pixels specified by frozen labels; uncertain/occluded pixels ignored; no unseen-site claim',
                label_sha256=hashlib.sha256(args.annotations.read_bytes()).hexdigest(),
                checkpoint_sha256=hashlib.sha256(args.checkpoint.read_bytes()).hexdigest(),
                metrics={split:{name:stats(*map(int,totals[(split,name)])) for name in models} for split in ('validation','evaluation')},
                region_metrics={split:{name:{region:stats(*map(int,regions[(split,name,region)])) for region in ('far','near')} for name in models} for split in ('validation','evaluation')},
                baseline_sha256=hashlib.sha256(args.pretrained.read_bytes()).hexdigest(),
                mask_ms={n:dict(median=float(np.median(ts[1:])),p95=float(np.percentile(ts[1:],95))) for n,ts in timings.items()},frames=records)
    (args.output/'metrics.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='frames'},indent=2))
if __name__=='__main__':main()
