"""Unified observation handoff for an opt-in path-level experiment.

Lane identity belongs to a verified corridor, not an image fragment number.
Current observations may update it; far alternatives cannot erase its near
prefix. This module publishes nothing and is not enabled in the vehicle node.
"""
import copy
import numpy as np
from .geometry import smooth_boundary
from .finite_path import FinitePathCache,PathCommand


def relative_to_path(points,path):
    """Signed normal offsets on SUPPORTED segments, without endpoint extension."""
    q=np.asarray(points,float);p=np.asarray(path,float)
    a=p[:-1];d=np.diff(p,axis=0);length=np.linalg.norm(d,axis=1)
    keep=length>1e-5;a,d,length=a[keep],d[keep],length[keep]
    if not len(a):return np.zeros(len(q),bool),np.full(len(q),np.nan)
    frac=np.sum((q[:,None,:]-a)*d,axis=2)/(length*length)
    allowed=(frac>=0)&(frac<=1)
    proj=a+np.clip(frac,0,1)[...,None]*d
    norm=np.linalg.norm(q[:,None,:]-proj,axis=2);norm[~allowed]=np.inf
    k=np.argmin(norm,axis=1);good=np.isfinite(norm[np.arange(len(q)),k])
    delta=q-proj[np.arange(len(q)),k];t=d[k]/length[k,None]
    signed=t[:,0]*delta[:,1]-t[:,1]*delta[:,0]
    return good,signed


def near_evidence(candidates,path,width,projector,camera,cap=7.):
    """Read only the cached arc through the allowed pursuit horizon.

    At least four samples spanning .3 m inside the lane are an explicit
    contradiction. Matching an edge does not by itself approve a new path.
    """
    p=np.asarray(path,float);p=p[np.linalg.norm(p,axis=1)<=cap]
    if len(p)<2:return dict(conflict=False,matches=[],details=[],scope='no_supported_near_arc')
    matches=[];details=[];conflict=False
    for b in candidates:
        if b.get('score',0)<.7:continue
        ground,valid=projector.project(b['points'],camera);g=ground[valid]
        if len(g)<4:continue
        good,d=relative_to_path(g,p)
        near=g[good];signed=d[good]
        rec=dict(id=b['id'],near_samples=len(near),relation='outside_supported_arc')
        if len(near)>=4:
            inside=np.abs(signed)<width/2-.3
            span=float(np.linalg.norm(np.diff(near[inside],axis=0),axis=1).sum()) if inside.sum()>=2 else 0.
            if inside.sum()>=4 and span>=.3:
                rec.update(relation='near_interior_conflict',span_m=span);conflict=True
            else:
                for side,sign in [('left',1.),('right',-1.)]:
                    fit=np.abs(signed-sign*width/2)<=.3
                    arc=float(np.linalg.norm(np.diff(near[fit],axis=0),axis=1).sum()) if fit.sum()>1 else 0.
                    if fit.sum()>=4 and arc>=.3:
                        err=float(np.percentile(abs(signed[fit]-sign*width/2),90))
                        matches.append(dict(id=b['id'],side=side,error_m=err));rec.update(relation=side+'_edge',error_m=err)
        details.append(rec)
    return dict(conflict=conflict,matches=matches,details=details,scope='supported_near_arc_only')


def segment_quality(candidates, geometry, camera, roles, cap=7.):
    """Record support per distance band; these are evidence, not probabilities.

    Points are existing detector polylines, not additional observations. Far
    support never votes on lane identity. Network confidence and paint width
    are deliberately not invented from a candidate's density score.
    """
    records=[]
    for b in candidates:
        pixels=np.asarray(b['points'],float)
        ground,valid=geometry.projector.project(pixels,camera)
        radius=np.linalg.norm(ground,axis=1)
        track=getattr(geometry,'current_tracks',{}).get(b['id'],{})
        role=next((side for side,identity in roles.items() if identity==b['id']),None)
        for name,lo,hi in [('near',0.,cap),('preview',cap,geometry.cfg.get('boundary_range_m',12.)),
                           ('far',geometry.cfg.get('boundary_range_m',12.),geometry.cfg.get('max_projection_m',30.))]:
            selected=valid&(radius>lo)&(radius<=hi)
            if not selected.any():continue
            q=ground[selected];px=pixels[selected]
            indices=np.flatnonzero(selected)
            # Do not count a chord over a missing piece as observed support.
            adjacent=np.diff(indices)==1
            arc=float(np.linalg.norm(np.diff(q,axis=0),axis=1)[adjacent].sum())
            edge=np.minimum.reduce([px[:,0],camera['width']-1-px[:,0],px[:,1],camera['height']-1-px[:,1]])
            records.append(dict(id=b['id'],role=role,band=name,samples=len(q),
                radius_m=[float(radius[selected].min()),float(radius[selected].max())],
                observed_arc_m=arc,edge_fraction=float(np.mean(edge<8)),
                fit_residual_px=b.get('residual_px'),support_density=b.get('score'),
                consecutive_observations=track.get('count',0),
                use=('identity_and_target_evidence' if name=='near' else 'confirmed_role_preview_only'),
                role_confirmed=role is not None,confidence_scope='geometric_evidence_not_calibrated_probability'))
    return records


def path_agreement(new,reference,tolerance=.25):
    good,offset=relative_to_path(new,reference)
    if good.sum()<4:return False,None
    error=float(np.percentile(abs(offset[good]),90))
    return error<=tolerance,error


class UnifiedLaneTracker:
    TRANSITIONS=frozenset({'AMBIGUOUS_CORRIDOR','REACQUISITION_IDENTITY_CONFLICT',
        'REACQUISITION_WIDTH_CONFLICT','REACQUIRING_CORRIDOR','WIDTH_CHANGED',
        'WIDTH_INCONSISTENT','CENTER_DISAGREEMENT','SINGLE_CORRIDOR_CONTRADICTION'})

    def __init__(self,config):
        self.cfg=config;self.cache=FinitePathCache(config['wheelbase'])
        self.follower=PathCommand(config);self.width=None;self.ids={};self.tracks={};self.known_tracks={}

    def stop(self,reason,source=None):
        self.cache.clear();self.follower.servo.reset();self.tracks={};self.known_tracks={}
        return dict(valid=False,speed=0.,steering_angle=0.,reason=reason,
                    source_failure=source,mode='stopped',metric_path=[],metric_valid=False,
                    path_update_state='unavailable',stop_required=True)

    def track_known_observations(self,result,stamp):
        """Short CURRENT-image history for roles previously accepted by geometry.

        Never create a role from proximity to the cache or renew it from a
        predicted path. Missing/jumping observations restart confirmation.
        """
        previous=self.known_tracks;current={}
        for b in result['candidates']:
            if b['id'] not in self.ids.values():continue
            old=previous.get(b['id']);count=1
            if old and 0<(stamp-old['stamp'])/1e9<=.15:
                a=np.asarray(old['points']);z=np.asarray(b['points'])
                lo=max(a[0,1],z[0,1]);hi=min(a[-1,1],z[-1,1])
                if hi-lo>=24:
                    ys=np.linspace(max(lo,hi-60),hi,12)
                    error=np.percentile(abs(np.interp(ys,a[:,1],a[:,0])-np.interp(ys,z[:,1],z[:,0])),90)
                    if error<=12:count=min(old['count']+1,8)
            current[b['id']]=dict(stamp=stamp,points=copy.deepcopy(b['points']),count=count)
        self.known_tracks=current
        return previous

    def known_near_alternative(self,base,result,geometry,stamp,camera,previous,blocked):
        """Resolve two clamped targets with DISJOINT near/far support.

        The nearer side must already own its role and pass current quality,
        temporal continuity and all-current-candidate contradiction checks.
        Overlapping contradictory centres and genuine ambiguity are excluded.
        """
        if base['valid'] or base['reason']!='NO_RELIABLE_SINGLE_BOUNDARY':return None
        if self.width is None or set(self.ids)!= {'left','right'}:return None
        by={b['id']:b for b in result['candidates']};offers=[]
        for side,identity in self.ids.items():
            b=by.get(identity)
            if b is None or identity in blocked or b.get('score',0)<.7:return None
            try:
                observed=geometry.boundary(b['points'],camera)
                p,t=smooth_boundary(geometry.single_boundary_window(observed,self.width))
                center=p+(-1 if side=='left' else 1)*self.width/2*np.c_[-t[:,1],t[:,0]]
                radii=np.linalg.norm(center,axis=1)
                if radii.min()<=self.cfg['lookahead_m'] or not geometry.single_target_allowed(center) or radii[0]>7:return None
                offers.append((center,side,b,radii))
            except ValueError:return None
        offers.sort(key=lambda v:v[3][0]);near,far=offers
        tolerance=self.cfg.get('center_agreement_m',.25)
        if near[3].max()+tolerance>=far[3].min():return None
        # No shared support is different from two disagreeing supported paths.
        if path_agreement(near[0],far[0])[1] is not None or path_agreement(far[0],near[0])[1] is not None:return None
        center,side,b,radii=near;old=previous.get(b['id']);track=self.known_tracks.get(b['id'])
        if old is None or track is None or track['count']<3:return None
        trial=copy.deepcopy(geometry);trial.latched=False
        trial.observed[b['id']]=(old['stamp'],np.asarray(old['points']))
        try:quality=trial.single_limit(stamp,center,b,camera)
        except ValueError:return None
        evidence=near_evidence(result['candidates'],center,self.width,trial.projector,camera)
        if evidence['conflict']:return None
        # Require the purported supporting boundary to match its current role.
        if not any(m['id']==b['id'] and m['side']==side for m in evidence['matches']):return None
        return center,dict(observation_mode='single_'+side,lane_width_m=self.width,
            observed_id=b['id'],handoff_side=side,handoff_current_count=track['count'],
            transition_policy='confirmed_near_side_disjoint_far_support',
            deferred_far_id=far[2]['id'],width_updated=False,current_handoff_evidence=evidence,**quality)

    def observed_alternative(self,result,geometry,reference,evidence,stamp,camera,blocked):
        by={b['id']:b for b in result['candidates']};offers=[];next_tracks={}
        for side in ['left','right']:
            choices=[m for m in evidence['matches'] if m['side']==side and m['id'] not in blocked]
            # Never select between competing CURRENT near edges by raw ID.
            if len(choices)!=1:continue
            b=by[choices[0]['id']];previous=self.tracks.get(side)
            current=dict(id=b['id'],stamp=stamp,points=copy.deepcopy(b['points']),count=1)
            if previous and 0<(stamp-previous['stamp'])/1e9<=.15:
                a=np.asarray(previous['points']);z=np.asarray(b['points'])
                lo=max(a[0,1],z[0,1]);hi=min(a[-1,1],z[-1,1])
                if hi-lo>=24:
                    y=np.linspace(max(lo,hi-60),hi,12)
                    shift=np.percentile(abs(np.interp(y,a[:,1],a[:,0])-np.interp(y,z[:,1],z[:,0])),90)
                    if shift<=12:current['count']=min(previous['count']+1,8)
            next_tracks[side]=current
            # Both IDs and numeric roles may change only with fresh geometric
            # evidence. Actual previous pixels, never predicted pixels, are used.
            if current['count']<3 or previous is None:continue
            trial=copy.deepcopy(geometry);trial.latched=False
            trial.observed[b['id']]=(previous['stamp'],np.asarray(previous['points']))
            try:
                observed=trial.boundary(b['points'],camera)
                points,tangent=smooth_boundary(trial.single_boundary_window(observed,self.width))
                sign=1 if side=='right' else -1
                center=points+sign*self.width/2*np.c_[-tangent[:,1],tangent[:,0]]
                agrees,error=path_agreement(center,reference,self.cfg.get('center_agreement_m',.25))
                if not agrees or np.linalg.norm(center[0])>7:continue
                quality=trial.single_limit(stamp,center,b,camera)
                check=near_evidence(result['candidates'],center,self.width,trial.projector,camera)
                if check['conflict']:continue
                offers.append((center,dict(observation_mode='single_'+side,lane_width_m=self.width,
                    observed_id=b['id'],handoff_side=side,handoff_current_count=current['count'],
                    handoff_error_m=error,width_updated=False,current_handoff_evidence=check,**quality)))
            except ValueError:continue
        self.tracks=next_tracks
        if len(offers)==2:
            good,_=path_agreement(offers[0][0],offers[1][0],self.cfg.get('center_agreement_m',.25))
            if not good:return None
        return min(offers,key=lambda offer:offer[1]['handoff_error_m']) if offers else None

    def _step(self,base,result,geometry,stamp,camera,*,speed,front_angle,
             feedback_stamp,motion_error_bound_m,blocked=()):
        if not self.cache.advance(stamp,speed,front_angle,feedback_stamp):
            return self.stop('MOTION_INVALID_OR_STALE')
        reference=None;evidence=dict(conflict=False,matches=[],details=[])
        if self.cache.path is not None:
            try:reference,_=self.cache.predict(stamp,'FRAME_DROPPED',conflict=False,motion_error_bound_m=motion_error_bound_m)
            except ValueError:reference=None
        previous=self.track_known_observations(result,stamp)
        current_alternative=(self.known_near_alternative(base,result,geometry,stamp,camera,previous,blocked)
                             if reference is not None else None)
        if reference is not None and self.width is not None:
            evidence=near_evidence(result['candidates'],reference,self.width,geometry.projector,camera)
        fresh=base['valid'] and base.get('lookahead_actual_m',100)<=7
        reason=base['reason'] if not base['valid'] else 'TARGET_OUTSIDE_7M'
        current_evidence=None
        if fresh and self.width is not None:
            current_evidence=near_evidence(result['candidates'],base['metric_path'],
                base.get('lane_width_m',self.width),geometry.projector,camera)
            # A valid upstream proposal is not permission to cross a visible
            # interior stripe. Judge it against ALL current candidates too.
            if current_evidence['conflict']:
                fresh=False;reason='CURRENT_NEAR_CORRIDOR_CONFLICT'
        if fresh and reference is not None:
            compatible,error=path_agreement(np.asarray(base['metric_path']),reference,self.cfg.get('center_agreement_m',.25))
            # Lack of shared support means no positive conflict was proved.
            # Keep normal trusted observations; only a measured disagreement
            # defers an update. The near-interior test already ran above.
            if error is not None and not compatible:fresh=False;reason='CENTER_DISAGREEMENT'
        alternative=None
        if reference is not None and self.width is not None:
            alternative=self.observed_alternative(result,geometry,reference,evidence,stamp,camera,blocked)
        # Joint decision: a verified current alternative may invalidate the
        # stale cached hypothesis. A conflict without such evidence still stops.
        accepted_alternative=(alternative is not None and
            (reason in self.TRANSITIONS or reason in self.cache.SUPPORT_FAILURES))
        if (evidence['conflict'] and current_alternative is None and not accepted_alternative) or reason=='CURRENT_NEAR_CORRIDOR_CONFLICT':
            out=self.stop('NEAR_CORRIDOR_CONFLICT',base['reason'])
            out.update(handoff_evidence=evidence,current_handoff_evidence=current_evidence,
                       joint_decision='unresolved_current_near_conflict')
            return out
        if current_alternative is not None:
            points,extra=current_alternative;mode='observed_near_handoff'
        elif fresh:
            points=np.asarray(base['metric_path']);extra=dict(base);self.width=base['lane_width_m'];self.ids=geometry.ids.copy();mode='observed'
        elif accepted_alternative:
            points,extra=alternative;self.ids[extra['handoff_side']]=extra['observed_id'];mode='observed_handoff'
        else:
            if reason in self.TRANSITIONS and reference is not None:
                # A vague upstream "ambiguity" is NOT permission to ignore
                # current near conflict: the independent geometry test is above.
                why='FRAME_DROPPED';mode='predicted_pending_handoff'
            else:why=reason;mode='predicted'
            try:points,extra=self.cache.predict(stamp,why,conflict=why not in self.cache.SUPPORT_FAILURES,motion_error_bound_m=motion_error_bound_m)
            except ValueError as exc:return self.stop(str(exc),reason)
            extra.update(source_failure=reason,handoff_pending=True)
        if mode.startswith('observed'):
            self.cache.observe(points,stamp,self.ids)
        out=self.follower.compute(points,stamp,camera,extra)
        out.update(mode=mode,handoff_evidence=evidence,corridor_ids=self.ids.copy(),
                   path_update_state='updated' if mode.startswith('observed') else 'pending',
                   stop_required=not out['valid'])
        if not out['valid']:
            self.cache.clear()
        elif mode in ('observed_handoff','observed_near_handoff'):
            # Commit the verified role to BOTH consumers. Previously the cache
            # accepted the new fragment but geometry kept the retired ID, so a
            # returning opposite boundary could never confirm its role.
            side=extra['handoff_side'];identity=extra['observed_id']
            geometry.ids[side]=identity
            current=next(b for b in result['candidates'] if b['id']==identity)
            geometry.observed[identity]=(stamp,np.asarray(current['points']).copy())
            # Width, last_dual and actuator enable state are not refreshed here.
            out['identity_commit']=dict(side=side,observed_id=identity,
                source='current_temporally_and_geometrically_verified_observation')
        out['joint_decision']=('accepted_current_observation' if mode.startswith('observed')
                               else 'bounded_pending_update')
        return out

    def step(self,base,result,geometry,stamp,camera,**feedback):
        out=self._step(base,result,geometry,stamp,camera,**feedback)
        out['segment_quality']=segment_quality(result['candidates'],geometry,camera,self.ids)
        return out
