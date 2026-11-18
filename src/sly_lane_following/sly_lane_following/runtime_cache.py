"""Opt-in Highbay corridor wrapper; measured local odometry, no track map.

The Pure Pursuit implementation and filtering state remain unchanged. Cached
points never refresh the observation stamp, and hard conflicts latch a stop.
"""
from collections import deque
import math
import numpy as np
from .finite_path import FinitePathCache
from .geometry import smooth_boundary


class RuntimePathCache:
    def __init__(self, corridor, config):
        self.current=corridor; self.cfg=config
        self.feedback=deque(maxlen=200)
        self.cache=FinitePathCache(config['wheelbase'])
        self.hard_stop=False; self.established=False

    def reset(self):
        self.current.reset(); self.feedback.clear()
        self.cache.clear(); self.cache.motion_stamp=None; self.hard_stop=False; self.established=False

    def feedback_sample(self, stamp, speed, yaw_rate):
        if self.feedback and stamp<=self.feedback[-1][0]:
            if stamp==self.feedback[-1][0]:return
            self.reset()
        self.feedback.append((stamp,speed,yaw_rate))

    def advance(self, stamp):
        samples=[s for s in self.feedback if s[0]<=stamp]
        if not samples or (stamp-samples[-1][0])/1e9>self.cache.max_gap:
            self.cache.clear();self.cache.invalid_reason='MOTION_STALE';return False
        last=self.cache.motion_stamp
        # A frozen path must be moved through every available feedback step,
        # including frames whose visual geometry cannot be refreshed.
        steps=[s for s in samples if last is None or s[0]>last]
        if last is None:steps=steps[-1:]
        if not steps or steps[-1][0]!=stamp:steps.append((stamp,*samples[-1][1:]))
        for t,v,w in steps:
            source_stamp=min(t,samples[-1][0])
            if not np.isfinite([v,w]).all() or v<-.001 or (abs(v)<.001 and abs(w)>.01):
                self.cache.clear();self.cache.invalid_reason='MOTION_INVALID';return False
            # Equivalent wheel angle from MEASURED yaw rate and forward speed.
            angle=math.atan(self.cfg['wheelbase']*w/v) if abs(v)>=.001 else 0.
            if not self.cache.advance(t,max(0.,v),angle,source_stamp):return False
        return True

    def near_conflict(self, result, path, camera):
        """A current confirmed side must agree with the cached near corridor.

        Unknown branches are not assigned a role by this check. Unresolved
        image ambiguity is never classified as an observation dropout.
        """
        if result.get('reason') in ('AMBIGUOUS_CORRIDOR','CORRIDOR_LOST_REARM_REQUIRED'):
            return True
        if len(result.get('pairs',[]))>1:return True
        if not self.current.widths:return True
        width=float(np.median(self.current.widths))
        ids={i:side for side,i in self.current.ids.items()}
        for candidate in result.get('candidates',[]):
            side=ids.get(candidate['id'])
            if candidate['score']<.7:continue
            try:
                observed=self.current.boundary(candidate['points'],camera)
                p,t=smooth_boundary(observed)
            except ValueError:continue
            if side is None:
                near=(p[:,0]>=path[0,0])&(p[:,0]<=min(path[-1,0],max(self.cfg['lookahead_m'],path[0,0])+1.))
                offset=abs(p[:,1]-np.interp(p[:,0],path[:,0],path[:,1]))
                inside=near&(offset<width/2-.3)
                if inside.sum()>=6 and np.ptp(p[inside,0])>=.6:return True
                continue
            sign=-1. if side=='left' else 1.
            center=p+sign*width/2*np.c_[-t[:,1],t[:,0]]
            # Compare only a monotone, common near region through the target.
            if np.any(np.diff(center[:,0])<-.05):continue
            lo=max(path[0,0],center[0,0]);hi=min(path[-1,0],center[-1,0],max(self.cfg['lookahead_m'],path[0,0])+1.)
            if hi-lo<.3:continue
            x=np.linspace(lo,hi,12)
            error=abs(np.interp(x,center[:,0],center[:,1])-np.interp(x,path[:,0],path[:,1]))
            if np.percentile(error,75)>self.cfg.get('center_agreement_m',.25):return True
        return False

    def compute(self, result, stamp, camera):
        if self.hard_stop:raise ValueError('CORRIDOR_REARM_REQUIRED')
        motion_ok=self.advance(stamp)
        was_latched=self.current.latched
        try:
            points,extra=self.current.compute(result,stamp,camera)
        except ValueError as error:
            reason=str(error)
            if not self.established:
                # Startup needs several observed frames to initialize width.
                # No driving path has been issued yet; do not latch startup.
                raise
            if reason not in self.cache.SUPPORT_FAILURES:
                self.cache.clear();self.hard_stop=True;raise
            try:
                if not motion_ok:raise ValueError(self.cache.invalid_reason or 'MOTION_STALE')
                points,extra=self.cache.predict(stamp,reason,conflict=False,
                    # Ideal simulator odometry, NOT a real-vehicle error bound.
                    motion_error_bound_m=0.)
                if self.near_conflict(result,points,camera):
                    raise ValueError('CACHE_NEAR_CONFLICT')
            except ValueError:
                self.cache.clear();self.hard_stop=True;raise
            # Only a successful, bounded fallback cancels the geometry latch.
            self.current.latched=was_latched
            return points,extra
        self.established=True
        if motion_ok:
            try:self.cache.observe(points,stamp,self.current.ids)
            except ValueError:self.cache.clear()
        extra=dict(extra,cache_ready=self.cache.path is not None)
        return points,extra
