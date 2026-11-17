"""Single RGB ground projection and visual Pure Pursuit; no map/odometry input.

Optical axes: right, down, forward. Rear-axle ground axes: forward, left, up.
CameraInfo K describes the undistorted image produced by core.camera_maps.
"""
import math
import numpy as np


class GroundProjector:
    def __init__(self, config):
        self.cfg = config
        self.t = np.asarray(config['camera_xyz_rear'], dtype=float)
        roll, pitch, yaw = config.get('camera_rpy', [0., 0., 0.])
        cr,sr,cp,sp,cy,sy = math.cos(roll),math.sin(roll),math.cos(pitch),math.sin(pitch),math.cos(yaw),math.sin(yaw)
        body = np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                         [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
        self.rotation = body @ np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
        if self.t.shape != (3,) or not np.isfinite(self.t).all() or self.t[2] <= 0 or not np.isfinite(self.rotation).all():
            raise ValueError('INVALID_CAMERA_EXTRINSICS')

    def project(self, pixels, camera):
        k = np.asarray(camera['K'], dtype=float).reshape(3,3)
        if not np.isfinite(k).all() or k[0,0] <= 0 or k[1,1] <= 0:
            raise ValueError('INVALID_CAMERA_INTRINSICS')
        points = np.asarray(pixels, dtype=float)
        if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
            raise ValueError('INVALID_PIXEL_POINTS')
        rays = np.c_[points,np.ones(len(points))] @ np.linalg.inv(k).T @ self.rotation.T
        valid = rays[:,2] < -1e-5
        scale = np.divide(-self.t[2],rays[:,2],out=np.zeros(len(points)),where=valid)
        ground = self.t + scale[:,None]*rays
        valid &= (ground[:,0] > 0) & (np.linalg.norm(ground[:,:2],axis=1) <= self.cfg.get('max_projection_m',30.))
        return ground[:,:2],valid

    def image(self, ground, camera):
        points=np.asarray(ground,dtype=float).reshape(-1,2)
        optical=(np.c_[points,np.zeros(len(points))]-self.t) @ self.rotation
        k=np.asarray(camera['K'],dtype=float).reshape(3,3)
        pixels=optical @ k.T
        return pixels[:,:2]/pixels[:,2,None]


def smooth_boundary(points):
    """Smooth only along the observed arc, with no extension past either end."""
    points=np.asarray(points,dtype=float)
    if points.ndim!=2 or len(points)<4 or not np.isfinite(points).all():
        raise ValueError('BOUNDARY_SUPPORT_TOO_SHORT')
    distance=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(points,axis=0),axis=1))]
    keep=np.r_[True,np.diff(distance)>.005]
    points,distance=points[keep],distance[keep]
    if len(points)<4 or distance[-1]<.6:raise ValueError('BOUNDARY_SUPPORT_TOO_SHORT')
    sample=np.linspace(0,distance[-1],max(12,int(distance[-1]/.12)))
    # Local quadratic tangents avoid a global polynomial flattening a sharp bend.
    raw=np.column_stack([np.interp(sample,distance,points[:,j]) for j in range(2)])
    tangent=np.empty_like(raw)
    for k in range(len(sample)):
        idx=np.argsort(abs(sample-sample[k]))[:min(9,len(sample))]
        for j in range(2):
            tangent[k,j]=np.polyfit(sample[idx]-sample[k],raw[idx,j],2)[1]
    norm=np.linalg.norm(tangent,axis=1)
    if np.any(norm<.1):raise ValueError('BOUNDARY_TANGENT_INVALID')
    tangent/=norm[:,None]
    return raw,tangent


def normal_widths(left,right):
    """Intersection distances along boundary normals, never image-row widths."""
    values=[]
    for boundary,other,sign in [(left,right,-1.),(right,left,1.)]:
        p,t=smooth_boundary(boundary);normal=sign*np.c_[-t[:,1],t[:,0]]
        for q,n in zip(p,normal):
            a=other[:-1];d=np.diff(other,axis=0);v=a-q
            cross=lambda x,y:x[...,0]*y[...,1]-x[...,1]*y[...,0]
            den=cross(n,d);good=abs(den)>1e-6
            length=np.divide(cross(v,d),den,out=np.zeros_like(den),where=good)
            fraction=np.divide(cross(v,n),den,out=np.zeros_like(den),where=good)
            ok=good&(length>1.)&(length<6.)&(fraction>=0)&(fraction<=1)
            if ok.any():values.append(float(length[ok].min()))
    return np.asarray(values)


class CorridorGeometry:
    """Current observed boundary + recently verified metric width, with limits."""
    def __init__(self,config,projector):
        self.cfg=config;self.projector=projector;self.reset()

    def reset(self):
        self.widths=[];self.ids={};self.last_dual=None;self.last_stamp=None
        self.single_travel_bound=0.;self.latched=False

    def boundary(self,pixels,camera):
        p,valid=self.projector.project(np.asarray(pixels)[::-1],camera)
        valid &= p[:,0]<=self.cfg.get('boundary_range_m',12.)
        first=next((i for i,v in enumerate(valid) if v),len(valid))
        end=next((i for i in range(first,len(valid)) if not valid[i]),len(valid))
        points=p[first:end]
        if len(points)<4:raise ValueError('BOUNDARY_SUPPORT_TOO_SHORT')
        return points

    def propose_width(self,width,mad,boundaries,candidates,ids,stamp):
        if self.widths and abs(width-np.median(self.widths))>self.cfg.get('width_change_max_m',.3):
            self.latched=True;raise ValueError('WIDTH_CHANGED')
        pending=(self.widths+[width])[-30:]
        return pending,float(np.median(pending))

    def single_limit(self,stamp,points,candidate,camera):
        age=(stamp-self.last_dual)/1e9
        if age>self.cfg.get('single_max_s',90.) or self.single_travel_bound>self.cfg.get('single_max_distance_m',10.):
            self.latched=True;raise ValueError('SINGLE_BOUNDARY_LIMIT')
        return {}

    def dual(self,result,stamp,camera,candidates):
        c=self.cfg
        ids=dict(zip(['left','right'],result['selected']))
        boundaries={side:self.boundary(candidates[identity]['points'],camera) for side,identity in ids.items()}
        values=normal_widths(boundaries['left'],boundaries['right'])
        if len(values)<6:raise ValueError('WIDTH_SUPPORT_TOO_SHORT')
        width=float(np.median(values));mad=float(np.median(abs(values-width)))
        if mad>c.get('width_mad_max_m',.15):raise ValueError('WIDTH_INCONSISTENT')
        pending_widths,width=self.propose_width(width,mad,boundaries,candidates,ids,stamp)
        if len(pending_widths)<c.get('width_init_frames',8):
            self.widths=pending_widths
            raise ValueError('INITIALIZING_METRIC_WIDTH')
        # Both sides generate normal-offset centres. Compare on overlapping
        # forward range before averaging; shared image rows are not normals.
        centers=[]
        for side,sign in [('left',-1.),('right',1.)]:
            p,t=smooth_boundary(boundaries[side]);centers.append(p+sign*width/2*np.c_[-t[:,1],t[:,0]])
        lo=max(v[:,0].min() for v in centers);hi=min(v[:,0].max() for v in centers)
        if hi-lo<.5:raise ValueError('CENTER_SUPPORT_TOO_SHORT')
        x=np.linspace(lo,hi,50)
        ys=[np.interp(x,v[np.argsort(v[:,0]),0],v[np.argsort(v[:,0]),1]) for v in centers]
        mismatch=abs(ys[0]-ys[1])>c.get('center_agreement_m',.25)
        first_bad=next((i for i,bad in enumerate(mismatch) if bad),len(x))
        if first_bad<3 or x[first_bad-1]-lo<.5:raise ValueError('CENTER_DISAGREEMENT')
        # A far branch must not invalidate the matching near corridor or
        # pull its target outward. Stop the path at the first disagreement.
        end=x[first_bad-1]
        near=int(centers[1][:,0].min()<centers[0][:,0].min())
        nearest=centers[near];nearest=nearest[np.argsort(nearest[:,0])]
        prefix=nearest[nearest[:,0]<lo]
        points=np.vstack([prefix,np.c_[x[:first_bad],((ys[0]+ys[1])/2)[:first_bad]]])
        self.widths=pending_widths;self.ids=ids;self.last_dual=stamp;self.single_travel_bound=0.
        mixed=lo-nearest[0,0]>.3
        return points,dict(observation_mode=('mixed_near_'+['left','right'][near]) if mixed else 'dual',
                           lane_width_m=width,width_mad_m=mad,single_distance_bound_m=0.,
                           truncated_at_disagreement=bool(mismatch.any()),reliable_forward_end_m=float(end))

    def near_corridor(self,result,stamp,camera,candidates,dt):
        """Defer a far branch only when current metric near geometry agrees.

        Historical width is a constraint, not updated from ambiguous pairs.
        No previous path, simulator pose or road map supplies these points.
        """
        c=self.cfg;pairs=list(result.get('pairs',[]))
        if not pairs or len(self.widths)<c.get('width_init_frames',8) or self.last_dual is None:
            raise ValueError('AMBIGUOUS_CORRIDOR')
        # Fragmentation can affect both sides simultaneously. Anchor only to
        # the previously verified pair, then require every plausible current
        # side to agree over the target-supporting prefix.
        known=[self.ids.get('left'),self.ids.get('right')]
        if not any(p['ids']==known for p in pairs):
            rejected=next((p.get('reason') for p in result.get('rejected_pairs',[]) if p['ids']==known),None)
            if rejected!='SPANS_ANOTHER_BOUNDARY' or not all(i in candidates for i in known):
                raise ValueError('AMBIGUOUS_CORRIDOR')
            # The image-wide crossing check may concern only a far branch.
            # Retain both hypotheses and test their CURRENT near geometry.
            pairs.append(dict(ids=known))
        roles={}
        for pair in pairs:
            for role,identity in zip(('left','right'),pair['ids']):
                if identity not in candidates or candidates[identity]['score']<.7:
                    raise ValueError('AMBIGUOUS_CORRIDOR')
                if identity in roles and roles[identity]!=role:raise ValueError('AMBIGUOUS_CORRIDOR')
                roles[identity]=role
        width=float(np.median(self.widths));horizon=c['lookahead_m']+width
        def centerline(candidate,role):
            observed=self.boundary(candidate['points'],camera)
            end=next((i for i,p in enumerate(observed) if np.linalg.norm(p)>horizon),len(observed))
            p,t=smooth_boundary(observed[:end]);sign=-1. if role=='left' else 1.
            points=p+sign*width/2*np.c_[-t[:,1],t[:,0]]
            # Offset tangents amplify pixel jitter into tiny local reversals.
            # Keep a forward envelope without sorting a genuine fold into a
            # fictitious straight path; truncate once reversal exceeds noise.
            kept=[points[0]]
            for point in points[1:]:
                if point[0]<kept[-1][0]-c.get('center_agreement_m',.25)/2:break
                if point[0]>kept[-1][0]+.005:kept.append(point)
            if len(kept)<4:raise ValueError('AMBIGUOUS_CORRIDOR')
            return np.asarray(kept)
        centers={i:centerline(candidates[i],role) for i,role in roles.items()}
        anchors=[i for i in self.ids.values() if i in centers]
        if len(anchors)!=2:raise ValueError('AMBIGUOUS_CORRIDOR')
        shared=[i for i in anchors if all(i in p['ids'] for p in pairs)]
        identity=shared[0] if len(shared)==1 else min(anchors,key=lambda i:np.linalg.norm(centers[i][0]))
        reference=centers[identity]
        target_radius=max(c['lookahead_m'],float(np.linalg.norm(reference[0])))
        # As in dual-boundary Pure Pursuit, clamp to the first CURRENT
        # visible centre point when the requested circle is below the view.
        # All competing sides must agree past that actual target below.
        end=float(reference[-1,0]);errors=[];consistent=[]
        # A branch is no longer ambiguous when only one continuation keeps
        # the measured width relative to the same tracked anchor. A pair
        # with a wrong near width is filtered, not granted a temporal bonus.
        for pair in pairs:
            a,b=(centers[i] for i in pair['ids'])
            lo=max(a[0,0],b[0,0]);hi=min(a[-1,0],b[-1,0],target_radius+width/2)
            if hi-lo<.6:continue
            x=np.linspace(lo,hi,max(12,int((hi-lo)/.05)))
            delta=abs(np.interp(x,a[:,0],a[:,1])-np.interp(x,b[:,0],b[:,1]))
            if np.max(delta)<=c.get('center_agreement_m',.25):consistent.append((pair,hi))
        resolved=consistent[0] if len(shared)==1 and len(consistent)==1 else None
        check_ids=set(resolved[0]['ids']) if resolved else set(centers)
        if resolved:end=min(end,resolved[1])
        for other_id,center in centers.items():
            if other_id==identity or other_id not in check_ids:continue
            lo=max(reference[0,0],center[0,0]);hi=min(reference[-1,0],center[-1,0])
            if hi-lo<.3:raise ValueError('AMBIGUOUS_CORRIDOR')
            x=np.linspace(lo,hi,max(8,int((hi-lo)/.05)+1))
            error=abs(np.interp(x,reference[:,0],reference[:,1])-np.interp(x,center[:,0],center[:,1]))
            bad=np.flatnonzero(error>c.get('center_agreement_m',.25))
            if len(bad):end=min(end,float(x[max(0,bad[0]-1)]))
            errors.append(dict(id=other_id,first_disagreement_m=float(x[bad[0]]) if len(bad) else None))
        points=reference[reference[:,0]<end]
        points=np.vstack([points,[end,np.interp(end,reference[:,0],reference[:,1])]])
        if len(points)<3 or np.linalg.norm(points[-1]-points[0])<.15 or np.linalg.norm(points[-1])<target_radius+.15:
            raise ValueError('AMBIGUOUS_CORRIDOR')
        self.single_travel_bound+=c.get('single_speed_bound',c['speed'])*dt
        age=(stamp-self.last_dual)/1e9
        quality=self.single_limit(stamp,points,candidates[identity],camera)
        return points,dict(observation_mode='mixed_near_confirmed',lane_width_m=width,
            width_age_s=age,single_distance_bound_m=self.single_travel_bound,
            observed_id=identity,deferred_branches=errors,resolved_pair=resolved[0]['ids'] if resolved else None,truncated_at_disagreement=True,
            reliable_forward_end_m=float(points[-1,0]),metric_evidence='current_shared_boundary_and_near_width_consistency',**quality)

    def compute(self,result,stamp,camera):
        c=self.cfg
        dt=0. if self.last_stamp is None else (stamp-self.last_stamp)/1e9
        if self.last_stamp is not None and not 0<dt<=.5:
            self.reset()
            raise ValueError('CONTROL_TIME_DISCONTINUITY')
        self.last_stamp=stamp
        if self.latched:raise ValueError('CORRIDOR_REARM_REQUIRED')
        candidates={b['id']:b for b in result.get('candidates',[])}
        known=[self.ids.get('left'),self.ids.get('right')]
        competing_known=(any(p['ids']==known for p in result.get('pairs',[])) or
                         any(p['ids']==known and p['reason']=='SPANS_ANOTHER_BOUNDARY'
                             for p in result.get('rejected_pairs',[])))
        if (result.get('reason')=='AMBIGUOUS_CORRIDOR' or
            (result.get('reason')=='REACQUIRING_CORRIDOR' and result.get('pairs') and competing_known)):
            return self.near_corridor(result,stamp,camera,candidates,dt)
        partial_dual=False
        if result.get('vision_valid') and len(result.get('selected',[]))==2:
            try:return self.dual(result,stamp,camera,candidates)
            except ValueError as error:
                # A current near identity may include an unresolved far tail.
                # Recheck all competing paths locally; never ignore a near
                # width/shape conflict or refresh width from this fallback.
                if str(error) in ('WIDTH_INCONSISTENT','CENTER_DISAGREEMENT') and result.get('pairs'):
                    return self.near_corridor(result,stamp,camera,candidates,dt)
                # Loss of common support can leave one fully observed side.
                # Width/identity conflicts remain faults, never a fallback.
                if str(error) not in ('CENTER_SUPPORT_TOO_SHORT','WIDTH_SUPPORT_TOO_SHORT','BOUNDARY_SUPPORT_TOO_SHORT'):
                    raise
                partial_dual=True
        if not partial_dual and result.get('reason') not in ('NO_SUPPORTED_EGO_PAIR','REACQUIRING_CORRIDOR'):
            raise ValueError(result.get('reason','NO_BOUNDARY'))
        visible=[(side,candidates[i]) for side,i in self.ids.items() if i in candidates]
        if not visible or len(self.widths)<c.get('width_init_frames',8):
            self.latched=bool(self.ids);raise ValueError('NO_RELIABLE_SINGLE_BOUNDARY')
        width=float(np.median(self.widths));horizon=c['lookahead_m']+width
        usable=[];clamped=[]
        for side,b in visible:
            if b['score']<.7:continue
            try:
                observed=self.boundary(b['points'],camera)
                # Cover the target's normal offset and local tangent fit, but
                # exclude distant bends that do not constrain this target.
                end=next((i for i,p in enumerate(observed) if np.linalg.norm(p)>horizon),len(observed))
                p,t=smooth_boundary(observed[:end])
            except ValueError:continue
            sign=-1. if side=='left' else 1.
            center=p+sign*width/2*np.c_[-t[:,1],t[:,0]]
            radius=np.linalg.norm(center,axis=1)
            # A short remnant can still exist in the image without observing
            # the lookahead target. Only one target-supporting known side is
            # allowed here; contradictory/ambiguous usable sides still stop.
            if radius.min()<=c['lookahead_m']<=radius.max():
                usable.append((side,b,p,t,center))
            elif min(abs(radius.min()-c['lookahead_m']),abs(radius.max()-c['lookahead_m']))<=c.get('single_target_clamp_m',.5):
                clamped.append((side,b,p,t,center))
        # Retain the bounded observed-endpoint clamp (default 0.5 m) when
        # exactly one side supports it. Never extend the curve to the target.
        if not usable and len(clamped)==1:usable=clamped
        if len(usable)!=1:
            self.latched=True;raise ValueError('NO_RELIABLE_SINGLE_BOUNDARY')
        self.single_travel_bound+=c.get('single_speed_bound',c['speed'])*dt
        age=(stamp-self.last_dual)/1e9
        side,b,p,t,points=usable[0]
        quality=self.single_limit(stamp,points,b,camera)
        # A different credible paint line inside the inferred corridor is a
        # contradiction, not permission to silently choose a neighboring lane.
        sign=-1. if side=='left' else 1.
        for identity,other in candidates.items():
            if identity==b['id']:continue
            try:q=self.boundary(other['points'],camera)
            except ValueError:continue
            order=np.argsort(p[:,0]);y=np.interp(q[:,0],p[order,0],p[order,1])
            inside=(q[:,0]>p[:,0].min())&(q[:,0]<p[:,0].max())&((q[:,1]-y)*sign>.3)&((q[:,1]-y)*sign<width-.3)
            if inside.sum()>5:self.latched=True;raise ValueError('SINGLE_CORRIDOR_CONTRADICTION')
        return points,dict(observation_mode='single_'+side,lane_width_m=width,width_age_s=age,
                           single_distance_bound_m=self.single_travel_bound,observed_id=b['id'],single_observation_horizon_m=horizon,**quality)


class VisualPurePursuit:
    def __init__(self, config):
        self.cfg=config
        self.projector=GroundProjector(config)
        corridor_type=CorridorGeometry
        if config.get('quality_width_history',False):
            from .real_corridor import QualityCorridor
            corridor_type=QualityCorridor
        self.corridor=corridor_type(config,self.projector) if config.get('allow_single_boundary',False) else None
        if config['wheelbase'] <= 0 or config['lookahead_m'] <= 0:
            raise ValueError('INVALID_VEHICLE_GEOMETRY')
        self.reset()

    def reset(self):
        self.last_stamp=None
        self.steering=0.
        if self.corridor:self.corridor.reset()

    def compute(self, result, shape, stamp, camera):
        c=self.cfg
        out=dict(valid=False,speed=0.,steering_angle=0.,reason=result.get('reason','VISION_INVALID'),
                 metric_valid=False,metric_path=[],metric_reason='VISION_INVALID',
                 geometry_source=c.get('geometry_source','configured'))
        extra={}
        if self.corridor:
            try:
                points,extra=self.corridor.compute(result,stamp,camera)
                pixels=self.projector.image(points,camera)
            except ValueError as error:
                self.steering=0.;self.last_stamp=None
                out.update(reason=str(error),metric_reason=str(error));return out
        else:
            if not result.get('vision_valid'):
                self.reset();return out
            pixels=np.asarray(result['center'],dtype=float)
            if len(pixels)<2 or np.ptp(pixels[:,1]) < c.get('min_span_fraction',.08)*shape[0]:
                self.reset();out.update(reason='CONTROL_VIEW_TOO_SHORT',metric_reason='CONTROL_VIEW_TOO_SHORT');return out
            pixels=pixels[::-1]
            points,valid=self.projector.project(pixels,camera)
            start=next((i for i,v in enumerate(valid) if v),len(valid))
            end=next((i for i in range(start,len(valid)) if not valid[i]),len(valid))
            points,pixels=points[start:end],pixels[start:end]
        if len(points)<2 or np.linalg.norm(np.diff(points,axis=0),axis=1).sum()<.15:
            out.update(reason='GROUND_VIEW_TOO_SHORT',metric_reason='GROUND_VIEW_TOO_SHORT');return out
        distance=np.linalg.norm(points,axis=1)
        # First forward intersection of the lookahead circle with the observed polyline.
        target=points[0].copy(); target_pixel=pixels[0].copy()
        requested=c['lookahead_m']
        if requested>distance[0]:
            target=points[-1].copy();target_pixel=pixels[-1].copy()
            for i,(a,b) in enumerate(zip(points[:-1],points[1:])):
                d=b-a;aa=float(d@d);bb=2*float(a@d);cc=float(a@a-requested**2)
                disc=bb*bb-4*aa*cc
                if aa<1e-12 or disc<0:continue
                roots=sorted([(-bb-math.sqrt(disc))/(2*aa),(-bb+math.sqrt(disc))/(2*aa)])
                root=next((v for v in roots if 0<=v<=1),None)
                if root is not None:
                    target=a+root*d;target_pixel=self.projector.image([target],camera)[0];break
        actual=float(np.linalg.norm(target));curvature=2*float(target[1])/(actual*actual)
        desired=math.atan(c['wheelbase']*curvature)
        bounded=float(np.clip(desired,-c['max_steering'],c['max_steering']))
        dt=1/30 if self.last_stamp is None else (stamp-self.last_stamp)/1e9
        if not 0<dt<=.5:
            self.reset();out.update(reason='CONTROL_TIME_DISCONTINUITY',metric_reason='CONTROL_TIME_DISCONTINUITY');return out
        filtered=self.steering+dt/(c['smoothing_s']+dt)*(bounded-self.steering)
        self.steering+=float(np.clip(filtered-self.steering,-c['steering_rate']*dt,c['steering_rate']*dt))
        self.last_stamp=stamp
        out.update(extra)
        out.update(valid=True,reason='PURE_PURSUIT',speed=float(c['speed']*(c.get('single_speed_scale',.75) if extra.get('observation_mode','').startswith(('single','mixed')) else 1.)/(1+c['turn_slowdown']*abs(self.steering))),
                   steering_angle=self.steering,target_steering=desired,target_rear_m=target.tolist(),target_pixel=target_pixel.tolist(),
                   lookahead_requested_m=requested,lookahead_actual_m=actual,curvature_request=curvature,
                   metric_valid=True,metric_reason='MODEL_GROUND_PROJECTION',metric_path=points.tolist(),
                   visible_span_px=float(np.ptp(pixels[:,1])),sample_span_px=float(np.ptp(pixels[:,1])))
        if c.get('adaptive_speed',False):
            from .speed_profile import preview_speed
            speed,k,reason=preview_speed(points,curvature,out['speed'],extra.get('observation_mode'))
            out.update(speed=speed,preview_curvature=k,speed_reason=reason,single_speed_bound=c.get('single_speed_bound',c['speed']))
        return out
