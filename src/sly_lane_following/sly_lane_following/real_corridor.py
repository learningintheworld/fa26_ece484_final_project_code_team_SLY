"""Opt-in observed-width policy for real RGB; nominal-model error budgets only.

No map, recorded route, guessed hidden pixels, or vehicle command is used.
Defaults remain in CorridorGeometry for the unchanged Highbay experiments.
"""
import copy
import numpy as np
from .geometry import CorridorGeometry,normal_widths,smooth_boundary


def near_width_observations(left,right,span):
    """Normal widths selected by their centre, not a shared boundary x-cut.

    On a bend the two ends of a normal have different forward coordinates.
    A shared x-cut can discard a perfectly observed inner boundary. Both
    endpoints below must be real observations; no extension is permitted.
    """
    samples=[]
    for side,(boundary,other,sign) in enumerate([(left,right,-1.),(right,left,1.)]):
        p,t=smooth_boundary(boundary);normal=sign*np.c_[-t[:,1],t[:,0]]
        for q,n in zip(p,normal):
            a=other[:-1];d=np.diff(other,axis=0);v=a-q
            cross=lambda x,y:x[...,0]*y[...,1]-x[...,1]*y[...,0]
            den=cross(n,d);good=abs(den)>1e-6
            length=np.divide(cross(v,d),den,out=np.zeros_like(den),where=good)
            fraction=np.divide(cross(v,n),den,out=np.zeros_like(den),where=good)
            ok=np.flatnonzero(good&(length>1.)&(length<6.)&(fraction>=0)&(fraction<=1))
            if not len(ok):continue
            i=ok[np.argmin(length[ok])];hit=a[i]+fraction[i]*d[i]
            endpoints=[q,hit] if side==0 else [hit,q]
            samples.append((float(length[i]),(q[0]+hit[0])/2,endpoints))
    if not samples:return np.array([]),[0.,0.],None
    lo=min(s[1] for s in samples);end=lo+span
    selected=[s for s in samples if s[1]<=end]
    arcs=[]
    for side,boundary in enumerate([left,right]):
        distance=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(boundary,axis=0),axis=1))]
        indices=[np.argmin(np.linalg.norm(boundary-s[2][side],axis=1)) for s in selected]
        arcs.append(float(np.ptp(distance[indices])))
    return np.array([s[0] for s in selected]),arcs,[lo,end]


class QualityCorridor(CorridorGeometry):
    def __init__(self,config,projector):
        for name,default in [('width_drift_mps',.02),('single_error_budget_m',.25),('width_radius_floor_m',.10)]:
            value=config.get(name,default)
            if not np.isfinite(value) or value<=0:raise ValueError('INVALID_WIDTH_UNCERTAINTY_CONFIG')
        if config.get('width_history_frames',180)<config.get('width_init_frames',8):
            raise ValueError('INVALID_WIDTH_HISTORY_LENGTH')
        super().__init__(config,projector)

    def reset(self):
        super().reset()
        self.width_radius=None
        self.width_fresh=False
        self.pending_pair=None
        self.pending_count=0
        self.pending_measurements=[]
        self.observed={}
        self.reassociated=None

    def propose_width(self,width,mad,boundaries,candidates,ids,stamp):
        c=self.cfg
        # Width memory is local to the nearest observed corridor. The window
        # is fixed from observation support, never chosen to minimize MAD.
        try:
            values,arcs,window=near_width_observations(boundaries['left'],boundaries['right'],2*c.get('width_min_arc_m',1.5))
        except ValueError:values,arcs,window=np.array([]),[0.,0.],None
        spans=[float(np.ptp(np.asarray(candidates[i]['points'])[:,1])) for i in ids.values()]
        edge_fractions=[float(np.mean((np.asarray(candidates[i]['points'])[:,0]<8)|
                               (np.asarray(candidates[i]['points'])[:,0]>self.image_width-8))) for i in ids.values()]
        whole_width,whole_mad=width,mad
        if len(values)>=6:
            width=float(np.median(values));mad=float(np.median(abs(values-width)))
        self.width_fresh=(len(values)>=6 and mad<=c.get('width_mad_max_m',.15)
                          and min(arcs)>=c.get('width_min_arc_m',1.5) and min(spans)>=c.get('width_min_span_px',70.)
                          and max(edge_fractions)<=.2 and all(candidates[i]['score']>=.7 for i in ids.values()))
        self.width_quality=dict(min_arc_m=min(arcs),min_span_px=min(spans),edge_fraction=max(edge_fractions),
                                measurement_scope='common_near_observation',forward_window_m=window,
                                local_samples=len(values),local_width_m=width if len(values)>=6 else None,
                                local_mad_m=mad if len(values)>=6 else None,
                                whole_width_m=whole_width,whole_mad_m=whole_mad)
        if len(values)>=6 and self.widths and abs(width-np.median(self.widths))>c.get('width_change_max_m',.3):
            self.latched=True;raise ValueError('WIDTH_CHANGED')
        if not self.width_fresh:
            if len(self.widths)<c.get('width_init_frames',8):raise ValueError('WIDTH_OBSERVATION_TOO_SHORT')
            return self.widths.copy(),float(np.median(self.widths))
        pending=(self.widths+[width])[-c.get('width_history_frames',180):]
        estimate=float(np.median(pending))
        # Do not divide by sqrt(N): adjacent frames share systematic errors.
        self.pending_radius=max(c.get('width_radius_floor_m',.10),
                                float(np.percentile(abs(np.asarray(pending)-estimate),95)),
                                2.5*mad,abs(width-estimate))
        return pending,estimate

    def dual(self,result,stamp,camera,candidates):
        before=(self.last_dual,self.single_travel_bound)
        points,extra=super().dual(result,stamp,camera,candidates)
        if self.width_fresh:
            self.width_radius=self.pending_radius
        else:
            self.last_dual,self.single_travel_bound=before
            # Current dual observations can support a path without licensing
            # unlimited use of a stale, low-quality width measurement.
            if self.last_dual is not None:
                extra.update(self.single_limit(stamp,points,candidates[self.ids['right']],camera))
                extra['width_age_s']=(stamp-self.last_dual)/1e9
        extra.setdefault('width_radius_m',self.width_radius)
        extra.update(width_updated=self.width_fresh,width_quality=self.width_quality,
                     reassociated_pair=self.reassociated)
        return points,extra

    def continuity(self,candidate,stamp):
        previous=self.observed.get(candidate['id'])
        if previous is None:return
        seen,old=previous;dt=(stamp-seen)/1e9
        if not 0<dt<=.5:raise ValueError('BOUNDARY_OBSERVATION_GAP')
        now=np.asarray(candidate['points']);lo=max(old[0,1],now[0,1]);hi=min(old[-1,1],now[-1,1])
        if hi-lo<24:raise ValueError('BOUNDARY_ASSOCIATION_UNVERIFIED')
        ys=np.linspace(max(lo,hi-60),hi,12)
        shift=float(np.percentile(abs(np.interp(ys,old[:,1],old[:,0])-np.interp(ys,now[:,1],now[:,0])),90))
        if shift>12+120*dt:raise ValueError('BOUNDARY_TEMPORAL_JUMP')

    def single_limit(self,stamp,points,candidate,camera):
        if self.width_radius is None or self.last_dual is None:raise ValueError('WIDTH_HISTORY_UNVERIFIED')
        self.continuity(candidate,stamp)
        age=(stamp-self.last_dual)/1e9
        target=points[np.argmin(abs(np.linalg.norm(points,axis=1)-self.cfg['lookahead_m']))]
        boundary=self.boundary(candidate['points'],camera)
        ground=boundary[np.argmin(np.linalg.norm(boundary-target,axis=1))]
        pixel=self.projector.image([ground],camera)[0]
        # Local image-fit sensitivity is separate from historical width drift.
        residual=float(candidate.get('residual_px',1.))
        if not np.isfinite(residual):raise ValueError('SINGLE_PROJECTION_UNCERTAIN')
        noise=max(1.,residual)
        perturbed,valid=self.projector.project(pixel+noise*np.array([[-1,-1],[-1,1],[1,-1],[1,1]]),camera)
        if not valid.all():raise ValueError('SINGLE_PROJECTION_UNCERTAIN')
        observation=max(.02,2*float(np.max(np.linalg.norm(perturbed-ground,axis=1))))
        drift=self.cfg.get('width_drift_mps',.02)
        if drift<=0:raise ValueError('INVALID_WIDTH_DRIFT_BOUND')
        radius=self.width_radius+drift*age
        budget=radius/2+observation
        if budget>self.cfg.get('single_error_budget_m',.25):
            raise ValueError('SINGLE_UNCERTAINTY_LIMIT')
        return dict(single_policy='current_observation_and_width_budget',
                    width_radius_m=radius,observation_error_budget_m=observation,
                    target_error_budget_m=budget,error_budget_scope='nominal_model_sensitivity_not_physical_confidence',
                    beyond_legacy_limit=bool(age>self.cfg.get('single_max_s',3.) or
                                            self.single_travel_bound>self.cfg.get('single_max_distance_m',.6)),
                    travel_distance_measured=False)

    def reassociate(self,result,stamp,camera):
        self.reassociated=None
        if not self.ids or self.latched:return result
        by_id={b['id']:b for b in result.get('candidates',[])}
        known=[self.ids['left'],self.ids['right']]
        pairs=result.get('pairs',[])
        selected=result.get('selected',[]) if result.get('vision_valid') else []
        if not selected and result.get('reason')=='REACQUIRING_CORRIDOR' and len(pairs)==1:selected=pairs[0]['ids']
        if not selected or selected==known:
            self.pending_pair=None;self.pending_count=0;self.pending_measurements=[]
            return result
        # A new ID needs a continuously seen anchor in the same boundary role.
        same=[i for i in range(2) if selected[i]==known[i] and selected[i] in by_id]
        if len(same)!=1 or any(i in selected and selected.index(i)!=known.index(i) for i in known):
            raise ValueError('REACQUISITION_IDENTITY_CONFLICT')
        if len(pairs)!=1:raise ValueError('AMBIGUOUS_CORRIDOR')
        self.continuity(by_id[selected[same[0]]],stamp)
        if any(by_id[i]['score']<.7 for i in selected):raise ValueError('REACQUISITION_WEAK_BOUNDARY')
        boundaries=[self.boundary(by_id[i]['points'],camera) for i in selected]
        returning=boundaries[1-same[0]]
        if np.min(np.linalg.norm(returning,axis=1))>self.cfg['lookahead_m']+float(np.median(self.widths)):
            # A purely distant new fragment cannot determine the current
            # corridor. Preserve it for contradiction checks, but defer its
            # identity instead of demanding nonexistent near width evidence.
            self.pending_pair=None;self.pending_count=0;self.pending_measurements=[]
            out=copy.copy(result);out.update(vision_valid=False,selected=[],reason='REACQUIRING_CORRIDOR')
            return out
        values=normal_widths(*boundaries)
        if len(values)<6:raise ValueError('REACQUISITION_WIDTH_UNVERIFIED')
        width=float(np.median(values));mad=float(np.median(abs(values-width)))
        if mad>self.cfg.get('width_mad_max_m',.15) or abs(width-np.median(self.widths))>self.cfg.get('width_change_max_m',.3):
            raise ValueError('REACQUISITION_WIDTH_CONFLICT')
        if selected==self.pending_pair and (not self.pending_measurements or abs(width-np.median(self.pending_measurements))<.12):
            self.pending_count+=1;self.pending_measurements.append(width)
        else:
            self.pending_pair=list(selected);self.pending_count=1;self.pending_measurements=[width]
        if self.pending_count>=self.cfg.get('reacquire_frames',8):
            self.reassociated=list(selected)
            out=copy.copy(result);out.update(vision_valid=True,selected=list(selected),reason='VISION_VALID_PIXELS_ONLY')
            return out
        # Keep the observed anchor, not the missing side. The newly seen side
        # remains a contradiction candidate; it is never erased from the input.
        out=copy.copy(result);out.update(vision_valid=False,selected=[],reason='REACQUIRING_CORRIDOR')
        return out

    def compute(self,result,stamp,camera):
        self.image_width=camera['width']
        if self.last_stamp is not None and not 0<(stamp-self.last_stamp)/1e9<=.5:
            self.reset();raise ValueError('CONTROL_TIME_DISCONTINUITY')
        try:
            result=self.reassociate(result,stamp,camera)
            points,extra=super().compute(result,stamp,camera)
            for b in result.get('candidates',[]):
                if b['id'] in self.ids.values():self.observed[b['id']]=(stamp,np.asarray(b['points']))
            return points,extra
        except ValueError as exc:
            # Once a real-data preview session is valid, all faults latch.
            # Observations returning cannot silently rearm the driving gate.
            if self.ids and str(exc)!='CONTROL_TIME_DISCONTINUITY':self.latched=True
            raise
