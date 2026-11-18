"""Experimental direct frame alignment; not connected to actuator/controller nodes.

Only raw right-boundary observations enter the 0.5 s cache. Every transform
maps one original frame directly to the current frame; transforms never chain.
"""
from dataclasses import dataclass
from collections import deque
import cv2
import numpy as np


@dataclass(frozen=True)
class RawRightObservation:
    stamp_ns: int
    identity: int
    gray: np.ndarray
    motion_mask: np.ndarray
    ground: np.ndarray


class RawRightCache:
    def __init__(self):
        self.items = deque()

    def clear(self):
        self.items.clear()

    def expire(self, now):
        if self.items and now < self.items[-1].stamp_ns:
            self.clear()
        while self.items and now-self.items[0].stamp_ns > 500_000_000:
            self.items.popleft()

    def add(self, stamp_ns, identity, gray, motion_mask, ground, *,
            source='raw', ambiguous=False):
        if source != 'raw':
            raise ValueError('PREDICTION_IS_NOT_OBSERVATION')
        if ambiguous or identity is None or len(ground) < 4:
            self.clear()
            return
        self.expire(stamp_ns)
        if self.items and (stamp_ns-self.items[-1].stamp_ns > 500_000_000 or
                           identity != self.items[-1].identity):
            self.clear()
        if self.items and stamp_ns == self.items[-1].stamp_ns:
            raise ValueError('DUPLICATE_TIMESTAMP')
        arrays = [np.array(a, copy=True) for a in (gray, motion_mask, ground)]
        for a in arrays:
            a.setflags(write=False)
        self.items.append(RawRightObservation(stamp_ns, identity, *arrays))

    def get(self, now, identity, age_s, *, ambiguous=False):
        self.expire(now)
        if ambiguous or identity is None:
            self.clear()
            return None
        if not 0 < age_s <= .5:
            raise ValueError('HISTORY_AGE_LIMIT')
        candidates = [o for o in self.items if o.identity == identity and
                      0 < now-o.stamp_ns <= 500_000_000]
        if not candidates:
            return None
        best = min(candidates, key=lambda o: abs((now-o.stamp_ns)/1e9-age_s))
        return best if abs((now-best.stamp_ns)/1e9-age_s) <= .04 else None


def rigid_fit(a, b):
    ac, bc = a.mean(0), b.mean(0)
    u, _, vt = np.linalg.svd((a-ac).T@(b-bc))
    r = vt.T@u.T
    if np.linalg.det(r) < 0:
        vt[-1] *= -1
        r = vt.T@u.T
    return r, bc-r@ac


def robust_motion(a, b):
    """Rigid ground-plane motion, deterministic fit/held-out split."""
    if len(a) < 25:
        raise ValueError('TOO_FEW_GROUND_TRACKS')
    idx = np.arange(len(a));train = idx % 4 != 0;test = ~train
    aa, bb = a[train], b[train];rng = np.random.default_rng(17);best = None
    for _ in range(100):
        pair = rng.choice(len(aa), 2, replace=False)
        if np.linalg.norm(aa[pair[0]]-aa[pair[1]]) < .5:
            continue
        r, t = rigid_fit(aa[pair], bb[pair])
        inlier = np.linalg.norm(aa@r.T+t-bb, axis=1) <= .12
        if best is None or inlier.sum() > best.sum():
            best = inlier
    if best is None or best.sum() < 15 or best.mean() < .6:
        raise ValueError('MOTION_INLIERS_INSUFFICIENT')
    if np.ptp(aa[best, 0]) < 2. or np.ptp(aa[best, 1]) < 1.:
        raise ValueError('MOTION_SPATIAL_COVERAGE')
    r, t = rigid_fit(aa[best], bb[best])
    fit95 = float(np.percentile(np.linalg.norm(aa[best]@r.T+t-bb[best], axis=1),95))
    validation95 = float(np.percentile(np.linalg.norm(a[test]@r.T+t-b[test],axis=1),95))
    if fit95 > .10 or validation95 > .15:
        raise ValueError('MOTION_HELDOUT_ERROR')
    if np.linalg.norm(t) > 2. or abs(np.arctan2(r[1,0],r[0,0])) > np.deg2rad(15):
        raise ValueError('MOTION_IMPLAUSIBLE')
    models=[]
    for _ in range(12):
        sub=rng.choice(np.flatnonzero(best),max(8,int(best.sum()*.8)),replace=False)
        models.append(rigid_fit(aa[sub],bb[sub]))
    return r,t,models,dict(tracks=len(a),inliers=int(best.sum()),fit_p95_m=fit95,
                          heldout_p95_m=validation95)


def align_right(old, now, gray, motion_mask, projector, camera, current_identity,
                *, ambiguous=False):
    if ambiguous or current_identity is None or old.identity != current_identity:
        raise ValueError('CURRENT_RIGHT_NOT_CONFIRMED')
    age=(now-old.stamp_ns)/1e9
    if not 0 < age <= .5:
        raise ValueError('HISTORY_AGE_LIMIT')
    pts=cv2.goodFeaturesToTrack(old.gray,350,.01,7,mask=old.motion_mask)
    if pts is None or len(pts)<25:
        raise ValueError('TOO_FEW_IMAGE_FEATURES')
    nxt,ok,_=cv2.calcOpticalFlowPyrLK(old.gray,gray,pts,None,winSize=(21,21),maxLevel=3)
    if nxt is None:
        raise ValueError('OPTICAL_FLOW_FAILED')
    back,rev,_=cv2.calcOpticalFlowPyrLK(gray,old.gray,nxt,None,winSize=(21,21),maxLevel=3)
    if back is None:
        raise ValueError('OPTICAL_FLOW_FAILED')
    a,b=pts[:,0],nxt[:,0];h,w=gray.shape
    valid=ok[:,0].astype(bool)&rev[:,0].astype(bool)&(np.linalg.norm(back[:,0]-a,axis=1)<=1.)
    valid&=np.isfinite(b).all(1)&(b[:,0]>=0)&(b[:,0]<w-1)&(b[:,1]>=0)&(b[:,1]<h-1)
    indexes=np.flatnonzero(valid);pixels=np.rint(b[indexes]).astype(int)
    valid[indexes]&=motion_mask[pixels[:,1],pixels[:,0]]>0
    ga,va=projector.project(a[valid],camera);gb,vb=projector.project(b[valid],camera)
    keep=va&vb&(np.linalg.norm(ga,axis=1)<=20)&(np.linalg.norm(gb,axis=1)<=20)
    r,t,models,stats=robust_motion(ga[keep],gb[keep])
    transformed=old.ground@r.T+t
    spread=max(float(np.linalg.norm(old.ground@rr.T+tt-transformed,axis=1).max()) for rr,tt in models)
    if spread>.10:
        raise ValueError('MOTION_PARAMETER_UNSTABLE')
    stats.update(age_s=age,parameter_spread_m=spread,
                 alignment_sensitivity_m=stats['heldout_p95_m']+spread,
                 rotation_rad=float(np.arctan2(r[1,0],r[0,0])),translation_m=t.tolist())
    return transformed,stats
