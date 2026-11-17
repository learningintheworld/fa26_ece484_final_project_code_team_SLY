#!/usr/bin/env python3
"""Fine-tune MP1 SimpleENet using frozen, reviewed, time-split local labels.
Run through scripts/dev.sh exec. Does not open/write the original ROS bag.
"""
import argparse, hashlib, json, random, time
from pathlib import Path
import cv2
import numpy as np
import torch
from torch import nn
from torch.utils.data import Dataset, DataLoader
from sly_lane_following.simple_enet import SimpleENet
from sly_lane_following.neural import letterbox


class LaneDataset(Dataset):
    def __init__(self, annotation, split):
        self.root = annotation.parent
        manifest = json.loads(annotation.read_text())
        if not manifest.get('frozen') or not all(f.get('reviewed') for f in manifest['frames']):
            raise ValueError('Freeze and review all annotations before training')
        self.frames = [f for f in manifest['frames'] if f['split'] == split]
        self.augment = split == 'train'
        for f in self.frames:
            if hashlib.sha256((self.root / f['label']).read_bytes()).hexdigest() != f['label_sha256']:
                raise ValueError('Annotation changed after freeze')

    def __len__(self): return len(self.frames)

    def __getitem__(self, i):
        f = self.frames[i]
        path = f['source_image'].replace('/home/xinweiyu/Workspace/ECE484_Program/', '/workspace/')
        image = cv2.imread(path); label = cv2.imread(str(self.root / f['label']), 0)
        if self.augment:
            if random.random() < .5:
                image = cv2.flip(image, 1); label = cv2.flip(label, 1)
            image = np.clip(image.astype(float) * random.uniform(.7, 1.3) + random.uniform(-12, 12), 0, 255).astype(np.uint8)
            if random.random() < .2: image = cv2.GaussianBlur(image, (3, 3), .7)
        x, y, _ = letterbox(image, label)
        return torch.from_numpy(x[None]), torch.from_numpy(y.astype(np.int64))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--annotations', type=Path, required=True)
    p.add_argument('--pretrained', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--epochs', type=int, default=50)
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    random.seed(484); np.random.seed(484); torch.manual_seed(484)
    torch.set_num_threads(4); cv2.setNumThreads(1)
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True
    device = torch.device('cuda')  # The existing Jazzy container GPU, no silent CPU fallback.
    manifest = json.loads(args.annotations.read_text())
    roi_top = manifest.get('inference_roi_top')
    if roi_top is not None and not 0 <= roi_top < 1:
        raise ValueError('Invalid inference ROI in frozen annotations')
    train = DataLoader(LaneDataset(args.annotations, 'train'), batch_size=4, shuffle=True, num_workers=0)
    val = DataLoader(LaneDataset(args.annotations, 'validation'), batch_size=4, shuffle=False, num_workers=0)
    model = SimpleENet().to(device)
    initial = torch.load(args.pretrained, map_location='cpu', weights_only=True)
    if 'state_dict' in initial:
        if initial.get('preprocessing') != 'gray_letterbox_640x384_v1':
            raise ValueError('Pretrained preprocessing mismatch')
        initial = initial['state_dict']
    model.load_state_dict(initial)
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-4)
    loss_fn = nn.CrossEntropyLoss(weight=torch.tensor([1., 10.], device=device), ignore_index=255)
    config = dict(seed=484, train_frames=len(train.dataset), validation_frames=len(val.dataset),
                  batch_size=4, lr=1e-4, max_epochs=args.epochs, patience=8, lane_class_weight=10,
                  pretrained=str(args.pretrained), pretrained_sha256=hashlib.sha256(args.pretrained.read_bytes()).hexdigest(),
                  annotation_sha256=hashlib.sha256(args.annotations.read_bytes()).hexdigest(),
                  preprocessing='gray_letterbox_640x384_v1', inference_roi_top=roi_top, torch=str(torch.__version__),
                  gpu=torch.cuda.get_device_name(), evaluation_set_used_for_training=False)
    (args.output/'config.json').write_text(json.dumps(config, indent=2))
    best = -1.; stale = 0; history = []; wall = time.perf_counter()
    for epoch in range(1, args.epochs+1):
        model.train(); losses=[]
        for x,y in train:
            x,y=x.to(device),y.to(device); optimizer.zero_grad(set_to_none=True)
            loss=loss_fn(model(x),y); loss.backward(); optimizer.step(); losses.append(loss.item())
        model.eval(); tp=fp=fn=0; vl=[]
        with torch.inference_mode():
            for x,y in val:
                x,y=x.to(device),y.to(device); logits=model(x); vl.append(loss_fn(logits,y).item())
                pred=logits.argmax(1); valid=y!=255
                tp+=int(((pred==1)&(y==1)&valid).sum());fp+=int(((pred==1)&(y==0)&valid).sum());fn+=int(((pred==0)&(y==1)&valid).sum())
        iou=tp/max(1,tp+fp+fn)
        row=dict(epoch=epoch,train_loss=float(np.mean(losses)),validation_loss=float(np.mean(vl)),validation_iou=iou,
                 validation_precision=tp/max(1,tp+fp),validation_recall=tp/max(1,tp+fn),elapsed_s=time.perf_counter()-wall)
        history.append(row); print(json.dumps(row),flush=True)
        if iou > best:
            best=iou; stale=0
            torch.save(dict(state_dict=model.state_dict(),preprocessing=config['preprocessing'],epoch=epoch,
                            validation_iou=iou,annotation_sha256=config['annotation_sha256'],inference_roi_top=roi_top),args.output/'best.pth')
        else: stale+=1
        (args.output/'history.json').write_text(json.dumps(history,indent=2))
        if stale>=8:break
    print('Done. Best validation IoU:',best,flush=True)
if __name__=='__main__':main()
