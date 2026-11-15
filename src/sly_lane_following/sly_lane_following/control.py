"""Image-space steering and a latched command gate; no metric path or TF."""
import math
import numpy as np


class ImageServo:
    def __init__(self, config):
        self.cfg = config
        if config.get("direction_mode", "difference") not in ("difference", "tangent"):
            raise ValueError("direction_mode must be difference or tangent")
        self.reset()

    def reset(self):
        self.last_stamp = None
        self.steering = 0.0

    def compute(self, result, shape, stamp):
        c = self.cfg
        out = dict(speed=0.0, steering_angle=0.0, valid=False, reason='VISION_INVALID')
        if not c.get('reference_verified', False):
            out['reason'] = 'IMAGE_REFERENCE_REQUIRED'
            self.reset()
            return out
        if not result.get('vision_valid'):
            out['reason'] = result.get('reason', 'VISION_INVALID')
            self.reset()
            return out
        h, w = shape[:2]
        pts = np.asarray(result['center'], dtype=float)
        if pts.ndim != 2 or len(pts) < 2 or pts.shape[1] != 2 or not np.isfinite(pts).all():
            self.reset()
            return out
        # Sample only the observed common range, never extrapolate into a bend.
        yn = min(c['near_row']*h, pts[-1, 1])
        yf = max(c['far_row']*h, pts[0, 1])
        if yn-yf < c.get('min_span_fraction', .08)*h:
            yn, yf = float(pts[-1,1]), float(pts[0,1])
        if yn-yf < c.get('min_span_fraction', .08)*h:
            out['reason'] = 'CONTROL_VIEW_TOO_SHORT'
            self.reset()
            return out
        xn, xf = np.interp([yn,yf],pts[:,1],pts[:,0])
        def reference(y):
            return np.interp(y/h, [c['far_row'],c['near_row']], [c['reference_far'],c['reference_near']])*w
        en = (xn-reference(yn))/(w/2)
        ef = (xf-reference(yf))/(w/2)
        image_angle = math.atan2((xf-xn)/w, (yn-yf)/h)
        reference_angle = math.atan2((reference(yf)-reference(yn))/w, (yn-yf)/h)
        tangent_error = math.atan2(math.sin(image_angle-reference_angle), math.cos(image_angle-reference_angle))
        direction = tangent_error if c.get('direction_mode', 'difference') == 'tangent' else ef-en
        # Image x points right; positive Ackermann steering points left.
        target = float(np.clip(-c['k_near']*en-c['k_direction']*direction,
                               -c['max_steering'],c['max_steering']))
        dt = 1/30 if self.last_stamp is None else (stamp-self.last_stamp)/1e9
        if not 0 < dt <= .5:
            self.reset(); out['reason']='CONTROL_TIME_DISCONTINUITY'; return out
        alpha = dt/(c['smoothing_s']+dt)
        filtered = self.steering+alpha*(target-self.steering)
        self.steering += float(np.clip(filtered-self.steering,-c['steering_rate']*dt,c['steering_rate']*dt))
        self.last_stamp = stamp
        speed = c['speed']/(1+c['turn_slowdown']*abs(self.steering))
        out.update(valid=True,reason='IMAGE_SERVO',speed=float(speed),steering_angle=self.steering,
                   near_error=float(en),direction_error=float(direction),
                   direction_mode=c.get('direction_mode','difference'),
                   image_tangent_rad=float(tangent_error), target_steering=float(target),
                   sample_span_px=float(yn-yf), visible_span_px=float(pts[-1,1]-pts[0,1]),
                   right_edge_margin_px=float(w-max(p[0] for p in result['right'])) if result.get('right') else None,
                   near=[float(xn),float(yn)],far=[float(xf),float(yf)],
                   reference_near=[float(reference(yn)),float(yn)],reference_far=[float(reference(yf)),float(yf)])
        return out


class CommandGate:
    """Latch on any fault. Fresh data alone never re-enables motion."""
    def __init__(self, timeout=.5, max_speed=.5, max_steering=.6):
        self.timeout=timeout; self.max_speed=max_speed; self.max_steering=max_steering
        self.enabled=False; self.reason='DISARMED'; self.last_time=None

    def check(self, now, request, feedback_time, measured_speed, override=False, competing=False):
        if override:return 'MANUAL_OVERRIDE'
        if competing:return 'COMPETING_PUBLISHER'
        if request is None:return 'NO_REQUEST'
        stamp,speed,steering=request
        if not all(math.isfinite(v) for v in (now,stamp,speed,steering,feedback_time,measured_speed)):
            return 'NONFINITE_INPUT'
        if self.last_time is not None and now < self.last_time:return 'CLOCK_REWIND'
        if not -.05<=now-stamp<=self.timeout:return 'STALE_REQUEST'
        if not 0<=now-feedback_time<=self.timeout:return 'STALE_FEEDBACK'
        if speed<=0:return 'STOP_REQUEST'
        if speed>self.max_speed+1e-6 or abs(steering)>self.max_steering+1e-6:return 'COMMAND_LIMIT'
        return None

    def arm(self, now, request, feedback_time, measured_speed, **flags):
        reason=self.check(now,request,feedback_time,measured_speed,**flags)
        if abs(measured_speed)>.1:reason='NOT_STATIONARY'
        self.enabled=reason is None; self.reason=reason or 'ARMED'; self.last_time=now
        return self.enabled,self.reason

    def step(self, now, request, feedback_time, measured_speed, **flags):
        reason=self.check(now,request,feedback_time,measured_speed,**flags)
        self.last_time=now
        if reason:
            self.enabled=False;self.reason=reason
        if not self.enabled:return 0.,0.
        self.reason='TRACKING'
        return min(request[1],self.max_speed),max(-self.max_steering,min(request[2],self.max_steering))

    def disarm(self, reason='OPERATOR_STOP'):
        self.enabled=False;self.reason=reason


def front_to_steering_wheel(front_radians):
    # Source: UIUC gem_odometry_control/pure_pursuit_odom.py front2steer.
    # Vehicle-specific mapping; physical e4 direction/range must be verified.
    deg=abs(math.degrees(front_radians))
    if deg>35:raise ValueError('front steering exceeds source mapping range')
    return math.copysign(math.radians(-.1084*deg*deg+21.775*deg),front_radians)
