"""Optional simulation speed planning; current visual geometry only."""
import math
import numpy as np


def preview_speed(points, curvature, legacy_speed, mode):
    p=np.asarray(points,float)
    if mode!='dual':return legacy_speed, 0., 'SINGLE_OR_MIXED'
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(p,axis=0),axis=1))]
    if arc[-1]<2.:return legacy_speed,0.,'SHORT_PREVIEW'
    s=np.arange(0.,arc[-1]+.001,.5)
    q=np.column_stack([np.interp(s,arc,p[:,i]) for i in range(2)])
    caps=[];max_k=abs(curvature)
    for i in range(2,len(q)-2):
        a,b,c=q[i-2],q[i],q[i+2];u=b-a;v=c-b;w=c-a
        k=2*abs(u[0]*v[1]-u[1]*v[0])/max(np.linalg.norm(u)*np.linalg.norm(v)*np.linalg.norm(w),1e-9)
        max_k=max(max_k,k)
        limit=1. if k<.015 else (.35 if k<.06 else .10)
        # Conservative jerk/latency allowance before the observed bend.
        distance=max(0.,np.linalg.norm(b)-3.5)
        caps.append(math.sqrt(limit**2+2*.18*distance))
    turn=1. if abs(curvature)<.01 else (.35 if abs(curvature)<.035 else legacy_speed)
    end=math.sqrt(.10**2+2*.18*max(0.,np.linalg.norm(p[-1])-3.5))
    return min([1.,turn,end]+caps),max_k,'VISIBLE_CURVATURE_PREVIEW'


class SmoothSpeed:
    def __init__(self):self.reset()
    def reset(self, speed=0.):self.speed=float(speed);self.acceleration=0.
    def step(self,target,dt):
        if not math.isfinite(target) or not 0<dt<=.2:raise ValueError('INVALID_SPEED_PROFILE_INPUT')
        target=max(0.,min(1.,target))
        desired=max(-.25,min(.20,(target-self.speed)/1.5))
        previous=self.acceleration
        self.acceleration+=max(-.3*dt,min(.3*dt,desired-previous))
        self.speed=max(0.,self.speed+(previous+self.acceleration)*dt/2)
        return self.speed,self.acceleration,(self.acceleration-previous)/dt


def single_speed_bound(feedback,previous_stamp,stamp):
    # Keep the old conservative floor, but account for a faster transition
    # using fresh vehicle velocity over this image interval. Missing feedback
    # uses the configured maximum; the actuator watchdog separately stops.
    if feedback is None:return 1.
    time,speed=feedback
    previous_stamp=stamp if previous_stamp is None else previous_stamp
    age=max(abs(stamp-time),abs(previous_stamp-time))/1e9
    if age>.5 or not math.isfinite(speed):return 1.
    return min(1.,max(.2,abs(speed)+.3*age))
