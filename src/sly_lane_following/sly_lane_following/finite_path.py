"""Bounded, motion-compensated path memory; no ROS dependencies.

Highbay may opt in through runtime_cache.py; e4 remains offline-only.

Only observe() stores fresh measured geometry. predict() never renews its age.
Motion must come from timestamped feedback; hypothetical motion is permitted
only in an explicitly labelled offline experiment, not the vehicle node.
"""
import math
import numpy as np


class FinitePathCache:
    SUPPORT_FAILURES = frozenset({
        'NO_RELIABLE_SINGLE_BOUNDARY', 'NO_SUPPORTED_EGO_PAIR',
        'BOUNDARY_SUPPORT_TOO_SHORT', 'CENTER_SUPPORT_TOO_SHORT',
        'WIDTH_SUPPORT_TOO_SHORT', 'BOUNDARY_OBSERVATION_GAP',
        'FRAME_DROPPED', 'TARGET_OUTSIDE_7M',
    })

    def __init__(self, wheelbase, max_age_s=1.5, max_travel_m=.3,
                 max_feedback_gap_s=.1, target_cap_m=7., max_motion_error_m=.1):
        values=[wheelbase,max_age_s,max_travel_m,max_feedback_gap_s,target_cap_m,max_motion_error_m]
        if not np.isfinite(values).all() or min(values)<=0:
            raise ValueError('INVALID_CACHE_LIMITS')
        self.wheelbase=wheelbase;self.max_age=max_age_s;self.max_travel=max_travel_m
        self.max_gap=max_feedback_gap_s;self.target_cap=target_cap_m;self.max_error=max_motion_error_m
        self.motion_stamp=None;self.clear()

    def clear(self):
        self.path=None;self.observation_stamp=None;self.identities=None
        self.pose=np.zeros(3);self.travel=0.;self.invalid_reason='CACHE_EMPTY'

    def advance(self, stamp, speed, front_angle, feedback_stamp):
        """Integrate measured bicycle motion relative to the original observation.

        Actual wheel angle is required, not the requested steering command.
        The offline experiment explicitly supplies hypothetical feedback.
        """
        if not np.isfinite([stamp,feedback_stamp,speed,front_angle]).all() or speed<0 or abs(front_angle)>=math.pi/2:
            self.clear();self.invalid_reason='MOTION_INVALID';self.motion_stamp=None;return False
        age=(stamp-feedback_stamp)/1e9
        if not 0<=age<=self.max_gap:
            self.clear();self.invalid_reason='MOTION_STALE';self.motion_stamp=None;return False
        dt=0. if self.motion_stamp is None else (stamp-self.motion_stamp)/1e9
        self.motion_stamp=stamp
        if dt<0 or dt>self.max_gap or (dt==0 and self.path is not None):
            self.clear();self.invalid_reason='MOTION_TIME_GAP';return False
        if self.path is not None:
            yaw=self.pose[2];omega=speed*math.tan(front_angle)/self.wheelbase;angle=omega*dt
            if abs(omega)<1e-9:dx,dy=speed*dt,0.
            else:dx,dy=speed/omega*math.sin(angle),speed/omega*(1-math.cos(angle))
            self.pose[:2]+=[math.cos(yaw)*dx-math.sin(yaw)*dy,math.sin(yaw)*dx+math.cos(yaw)*dy]
            self.pose[2]+=angle;self.travel+=speed*dt
        return True

    def observe(self, points, stamp, identities):
        p=np.asarray(points,dtype=float)
        if self.motion_stamp!=stamp:raise ValueError('MOTION_NOT_CURRENT')
        if p.ndim!=2 or p.shape[1]!=2 or len(p)<2 or not np.isfinite(p).all() or np.any(p[:,0]<=0):
            raise ValueError('INVALID_OBSERVED_PATH')
        if np.linalg.norm(p[0])>self.target_cap:raise ValueError('TARGET_OUTSIDE_7M')
        if np.linalg.norm(np.diff(p,axis=0),axis=1).sum()<.15:raise ValueError('PATH_TOO_SHORT')
        self.path=p.copy();self.observation_stamp=stamp;self.identities=dict(identities)
        self.pose[:]=0.;self.travel=0.;self.invalid_reason=None

    def predict(self, stamp, reason, *, conflict, motion_error_bound_m):
        if conflict or reason not in self.SUPPORT_FAILURES:
            self.clear();self.invalid_reason='CACHE_CONFLICT';raise ValueError(self.invalid_reason)
        if self.path is None:raise ValueError(self.invalid_reason or 'CACHE_EMPTY')
        age=(stamp-self.observation_stamp)/1e9
        if self.motion_stamp!=stamp:raise ValueError('MOTION_NOT_CURRENT')
        if not np.isfinite(motion_error_bound_m) or motion_error_bound_m<0 or motion_error_bound_m>self.max_error:
            self.clear();raise ValueError('MOTION_UNCERTAIN')
        if not 0<age<=self.max_age or self.travel>self.max_travel:
            self.clear();raise ValueError('CACHE_EXPIRED')
        x,y,yaw=self.pose;c,s=math.cos(yaw),math.sin(yaw)
        points=(self.path-[x,y])@np.array([[c,-s],[s,c]])
        # Keep only the remaining forward part. Never add or extend points.
        first=next((i for i,p in enumerate(points) if p[0]>0),len(points));points=points[first:]
        if len(points)<2 or np.any(points[:,0]<=0) or np.linalg.norm(np.diff(points,axis=0),axis=1).sum()<.15:
            raise ValueError('CACHE_SUPPORT_EXHAUSTED')
        if np.linalg.norm(points[0])>self.target_cap:raise ValueError('TARGET_OUTSIDE_7M')
        return points,dict(observation_mode='predicted_cached_path',cache_age_s=age,
            cache_travel_m=self.travel,cache_pose=self.pose.tolist(),
            cache_observation_stamp=self.observation_stamp,cache_identities=self.identities.copy(),
            motion_error_bound_m=motion_error_bound_m,source_failure=reason)


class PathCommand:
    """Run the unchanged VisualPurePursuit on a supplied path, not paint pixels.

    Opt-in analysis adapter; not installed in the vehicle node. All paths in
    this adapter, including dual-boundary paths, obey the same target cap.
    """
    class Input:
        def reset(self):pass
        def compute(self,result,stamp,camera):return np.asarray(result['path']),result['extra']

    def __init__(self,config,target_cap_m=7.,prediction_speed_cap=.1):
        from .geometry import VisualPurePursuit
        self.servo=VisualPurePursuit(config);self.servo.corridor=self.Input()
        self.cap=target_cap_m;self.speed_cap=prediction_speed_cap

    def compute(self,path,stamp,camera,extra):
        out=self.servo.compute(dict(path=path,extra=extra),(camera['height'],camera['width'],3),stamp,camera)
        if out['valid'] and out['lookahead_actual_m']>self.cap:
            self.servo.reset();return dict(valid=False,speed=0.,steering_angle=0.,reason='TARGET_OUTSIDE_7M')
        if out['valid'] and extra.get('observation_mode')=='predicted_cached_path':
            out['speed']=min(out['speed'],self.speed_cap);out['reason']='PURE_PURSUIT_CACHED_PATH'
        return out
