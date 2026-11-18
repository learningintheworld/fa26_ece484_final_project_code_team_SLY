#!/usr/bin/env python3
"""Approval-gated night fine-tuning. Keeps the historical training entry intact."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import random
import time
import cv2
import numpy as np
import torch
from torch import nn
from torch.utils.data import Dataset, DataLoader
from sly_lane_following.neural import letterbox, load_checkpoint
from sly_lane_following.night_augmentation import make_training_view, VERSION
from sly_lane_following.simple_enet import create_model, initialize_from_mp1, freeze_pretrained_batchnorm

MP1_SHA256='f950873918898bd18d0dcc0fc20d9dae4beeb65df49793d783761f4549e2f766'


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_manifest(path):
    m=json.loads(path.read_text())
    if not (m.get('frozen') and m.get('training_authorized') and m.get('user_review_approved')):
        raise ValueError('NIGHT_REVIEW_REQUIRED: 20-example user approval and full label review are required')
    if m.get('cross_split_near_duplicates'):
        raise ValueError('NIGHT_SPLIT_DUPLICATES_UNRESOLVED')
    if Counter(f['split'] for f in m['frames']) != Counter(train=350,validation=75,evaluation=75):
        raise ValueError('NIGHT_DATASET_COUNTS_MISMATCH')
    grouped={}; images={}
    for f in m['frames']:
        if not f.get('reviewed') or not f.get('label') or not f.get('reviewer'):
            raise ValueError('NIGHT_LABEL_REVIEW_REQUIRED')
        if grouped.setdefault(f['group_id'],f['split'])!=f['split']:
            raise ValueError('NIGHT_GROUP_LEAKAGE')
        if images.setdefault(f['image_sha256'],f['split'])!=f['split']:
            raise ValueError('NIGHT_IMAGE_LEAKAGE')
        for key in ('image','label'):
            if sha(path.parent/f[key])!=f[key+'_sha256']:
                raise ValueError('NIGHT_FROZEN_FILE_CHANGED')
    ordered=sorted(m['frames'],key=lambda f:f['relative_s'])
    for a,b in zip(ordered,ordered[1:]):
        if a['split']!=b['split'] and b['relative_s']-a['relative_s']<.5:
            raise ValueError('NIGHT_TEMPORAL_GUARD_FAILED')
    return m


class NightDataset(Dataset):
    def __init__(self, path, split):
        self.root=path.parent;self.split=split;self.epoch=0
        m=json.loads(path.read_text());self.frames=[f for f in m['frames'] if f['split']==split]
        self.set_epoch(0)

    def set_epoch(self,epoch):
        self.epoch=epoch;n=len(self.frames)
        # Exact 50/40/10 mix among the augmented half, permuted each epoch.
        severity=['light']*round(.5*n)+['medium']*round(.4*n)
        severity+=['strong']*(n-len(severity))
        self.severity=list(np.random.default_rng(484+epoch).permutation(severity))

    def __len__(self):return len(self.frames)*(2 if self.split=='train' else 3)

    def __getitem__(self,index):
        nviews=2 if self.split=='train' else 3
        i,view=divmod(index,nviews);f=self.frames[i]
        image=cv2.imread(str(self.root/f['image']));label=cv2.imread(str(self.root/f['label']),0)
        if image is None or label is None:raise ValueError('NIGHT_DATASET_FILE_MISSING')
        if view:
            severity=self.severity[i] if self.split=='train' else ('light','medium')[view-1]
            seed=484 + i*1009 + (self.epoch*1000003 if self.split=='train' else view*1000033)
            image,_=make_training_view(image,label,severity,seed)
        x,y,_=letterbox(image,label)
        return torch.from_numpy(x[None]),torch.from_numpy(y.astype(np.int64)),view


def save_weights(model,path,architecture,roi,extra=None):
    state={k:v.detach().cpu() for k,v in model.state_dict().items()}
    torch.save(state,path)
    meta=dict(schema='sly_enet_tensor_checkpoint_v1',architecture=architecture,
              preprocessing='gray_letterbox_640x384_v1',inference_roi_top=roi,sha256=sha(path),
              augmentation_version=VERSION)
    if extra:meta.update(extra)
    path.with_suffix('.json').write_text(json.dumps(meta,indent=2))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--annotations',type=Path,required=True)
    p.add_argument('--pretrained',type=Path,required=True)
    p.add_argument('--architecture',choices=['simple_enet_v1','simple_enet_night_v1'],required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--epochs',type=int,default=50)
    args=p.parse_args()
    # Approval is checked before CUDA setup, output creation or optimizer steps.
    m=validate_manifest(args.annotations)
    if sha(args.pretrained)!=MP1_SHA256:raise ValueError('NIGHT_EXPECTED_ORIGINAL_MP1_EPOCH20')
    if not 1<=args.epochs<=50:raise ValueError('Maximum 50 epochs')
    random.seed(484);np.random.seed(484);torch.manual_seed(484)
    torch.set_num_threads(4);cv2.setNumThreads(1)
    torch.backends.cudnn.benchmark=False;torch.backends.cudnn.deterministic=True
    if not torch.cuda.is_available():raise RuntimeError('NIGHT_CUDA_REQUIRED')
    model=create_model(args.architecture)
    initial=load_checkpoint(torch,args.pretrained)
    initialize_from_mp1(model,initial.get('state_dict',initial))
    model=model.cuda()
    train=NightDataset(args.annotations,'train');val=NightDataset(args.annotations,'validation')
    trainloader=DataLoader(train,batch_size=4,shuffle=True,num_workers=2,pin_memory=True)
    valloader=DataLoader(val,batch_size=4,shuffle=False,num_workers=2,pin_memory=True)
    optimizer=torch.optim.Adam(model.parameters(),lr=1e-4)
    loss_fn=nn.CrossEntropyLoss(weight=torch.tensor([1.,10.],device='cuda'),ignore_index=255)
    args.output.mkdir(parents=True,exist_ok=False)
    config=dict(architecture=args.architecture,parameters=sum(p.numel() for p in model.parameters()),
                seed=484,train_originals=len(train.frames),train_samples_per_epoch=len(train),validation_originals=len(val.frames),
                pretrained_sha256=MP1_SHA256,annotations_sha256=sha(args.annotations),lr=1e-4,batch_size=4,
                max_epochs=args.epochs,patience=8,data_workers=2,old_batchnorm_running_stats_frozen=True,
                selection_score='0.5 clean IoU + 0.25 light IoU + 0.25 medium IoU',
                test_set_used=False,augmentation_version=VERSION,torch=str(torch.__version__))
    (args.output/'config.json').write_text(json.dumps(config,indent=2))
    history=[];best=-1.;stale=0;wall=time.perf_counter()
    for epoch in range(1,args.epochs+1):
        train.set_epoch(epoch);model.train();freeze_pretrained_batchnorm(model);losses=[]
        for x,y,_ in trainloader:
            x,y=x.cuda(non_blocking=True),y.cuda(non_blocking=True);optimizer.zero_grad(set_to_none=True)
            loss=loss_fn(model(x),y);loss.backward();optimizer.step();losses.append(float(loss.item()))
        model.eval();counts=np.zeros((3,3),np.int64)
        with torch.inference_mode():
            for x,y,view in valloader:
                x,y=x.cuda(non_blocking=True),y.cuda(non_blocking=True);pred=model(x).argmax(1);valid=y!=255
                for kind in range(3):
                    selected=view.to(y.device)==kind
                    if not selected.any():continue
                    pp,yy,vv=pred[selected],y[selected],valid[selected]
                    counts[kind]+=[int(((pp==1)&(yy==1)&vv).sum()),int(((pp==1)&(yy==0)&vv).sum()),int(((pp==0)&(yy==1)&vv).sum())]
        metrics={name:dict(iou=int(tp)/max(1,int(tp+fp+fn)),precision=int(tp)/max(1,int(tp+fp)),recall=int(tp)/max(1,int(tp+fn))) for name,(tp,fp,fn) in zip(('clean','light','medium'),counts)}
        score=.5*metrics['clean']['iou']+.25*metrics['light']['iou']+.25*metrics['medium']['iou']
        row=dict(epoch=epoch,train_loss=float(np.mean(losses)),validation=metrics,selection_score=score,elapsed_s=time.perf_counter()-wall)
        history.append(row);print(json.dumps(row),flush=True)
        if score>best:
            best=score;stale=0
            save_weights(model,args.output/'best.pth',args.architecture,m['inference_roi_top'],dict(epoch=epoch,selection_score=score))
        else:stale+=1
        (args.output/'history.json').write_text(json.dumps(history,indent=2))
        if stale>=8:break


if __name__=='__main__':main()
