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
        cap=config.get('single_observed_target_max_m')
        if cap is not None and (not np.isfinite(cap) or cap<config['lookahead_m'] or
                                cap>config.get('max_projection_m',30.)):
            raise ValueError('INVALID_OBSERVED_TARGET_LIMIT')
        span=config.get('single_fit_arc_m')
        if span is not None and (not np.isfinite(span) or span<.6):
            raise ValueError('INVALID_SINGLE_FIT_ARC')
        super().__init__(config,projector)

    def reset(self):
        super().reset()
        self.input_stamp=None
        self.current_tracks={}
        self.width_radius=None
        self.width_fresh=False
        self.pending_pair=None
        self.pending_count=0
        self.pending_measurements=[]
        self.observed={}
        self.reassociated=None
        self.side_vote=None
        self.side_vote_count=0
        self.side_history={}
        self.side_evidence={}

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

    def single_boundary_window(self, observed, width):
        """Prefix of ONE current supported fragment, with no point insertion.

        The detector splits unsupported mask gaps before creating candidates.
        This policy never joins candidates or extends past their endpoints.
        """
        span=self.cfg.get('single_fit_arc_m')
        if span is None:return super().single_boundary_window(observed,width)
        arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(observed,axis=0),axis=1))]
        return observed[:np.searchsorted(arc,span,side='right')]

    def single_target_allowed(self, center):
        """Use the first CURRENT supported point when the desired circle is nearer.

        PP already uses this endpoint's actual distance. Width, continuity,
        uncertainty and contradiction checks still run before acceptance.
        """
        cap=self.cfg.get('single_observed_target_max_m')
        radius=np.linalg.norm(center,axis=1)
        if cap is None or radius.min()<=self.cfg['lookahead_m']:
            return super().single_target_allowed(center)
        return bool(np.isfinite(center).all() and center[0,0]>0 and radius[0]<=cap)

    def single_budget(self,stamp,points,candidate,camera):
        """Measure nominal budget without deciding whether driving is allowed."""
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
        return dict(single_target_policy=('bounded_current_observation' if self.cfg.get('single_observed_target_max_m') is not None else 'legacy_endpoint_tolerance'),
                    single_observed_target_max_m=self.cfg.get('single_observed_target_max_m'),
                    single_policy='current_observation_and_width_budget',
                    width_radius_m=radius,observation_error_budget_m=observation,
                    target_error_budget_m=budget,error_budget_scope='nominal_model_sensitivity_not_physical_confidence',
                    beyond_legacy_limit=bool(age>self.cfg.get('single_max_s',3.) or
                                            self.single_travel_bound>self.cfg.get('single_max_distance_m',.6)),
                    travel_distance_measured=False)

    def single_limit(self,stamp,points,candidate,camera):
        budget=self.single_budget(stamp,points,candidate,camera)
        if budget['target_error_budget_m']>self.cfg.get('single_error_budget_m',.25):
            raise ValueError('SINGLE_UNCERTAINTY_LIMIT')
        return budget

    def reassociate(self,result,stamp,camera):
        self.reassociated=None
        if not self.ids or self.latched:return result
        by_id={b['id']:b for b in result.get('candidates',[])}
        known=[self.ids['left'],self.ids['right']]
        pairs=result.get('pairs',[])
        selected=result.get('selected',[]) if result.get('vision_valid') else []
        if not selected and not pairs and sum(i in by_id for i in known)==1:
            # Image-row overlap is not a ground-normal width measurement.
            # Recover a returning side only from a unique CURRENT metric pair.
            proposals=[]
            side=next(k for k,i in enumerate(known) if i in by_id)
            anchor=by_id[known[side]]
            for identity,b in by_id.items():
                if identity in known or min(anchor['score'],b['score'])<.7:continue
                ids=[identity,known[side]] if side==1 else [known[side],identity]
                try:
                    obs=[self.boundary(by_id[i]['points'],camera) for i in ids]
                    values,arcs,_=near_width_observations(*obs,2*self.cfg.get('width_min_arc_m',1.5))
                    if len(values)<6 or min(arcs)<self.cfg.get('width_min_arc_m',1.5):continue
                    width=float(np.median(values));mad=float(np.median(abs(values-width)))
                    if mad<=self.cfg.get('width_mad_max_m',.15) and abs(width-np.median(self.widths))<=self.cfg.get('width_change_max_m',.3):
                        proposals.append(dict(ids=ids,metric_return=True))
                except ValueError:continue
            if len(proposals)==1:
                pairs=proposals;selected=proposals[0]['ids']
                result=copy.copy(result);result.update(pairs=pairs)
            elif len(proposals)>1:raise ValueError('AMBIGUOUS_CORRIDOR')
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
        values,arcs,window=near_width_observations(*boundaries,2*self.cfg.get('width_min_arc_m',1.5))
        if len(values)<6 or min(arcs)<self.cfg.get('width_min_arc_m',1.5):
            # Insufficient evidence for a NEW side does not invalidate the
            # observed anchor. Keep every candidate for contradiction checks.
            self.pending_pair=None;self.pending_count=0;self.pending_measurements=[]
            out=copy.copy(result);out.update(vision_valid=False,selected=[],reason='NO_SUPPORTED_EGO_PAIR')
            return out
        width=float(np.median(values));mad=float(np.median(abs(values-width)))
        if mad>self.cfg.get('width_mad_max_m',.15) or abs(width-np.median(self.widths))>self.cfg.get('width_change_max_m',.3):
            raise ValueError('REACQUISITION_WIDTH_CONFLICT')
        if selected==self.pending_pair and (not self.pending_measurements or abs(width-np.median(self.pending_measurements))<.12):
            self.pending_count+=1;self.pending_measurements.append(width)
        else:
            self.pending_pair=list(selected);self.pending_count=1;self.pending_measurements=[width]
        new_id=selected[1-same[0]]
        stable=self.current_tracks.get(new_id,{}).get('count',0)
        if max(self.pending_count,stable)>=self.cfg.get('reacquire_frames',8):
            self.reassociated=list(selected)
            out=copy.copy(result);out.update(vision_valid=True,selected=list(selected),reason='VISION_VALID_PIXELS_ONLY')
            return out
        # Keep the observed anchor, not the missing side. The newly seen side
        # remains a contradiction candidate; it is never erased from the input.
        out=copy.copy(result);out.update(vision_valid=False,selected=[],reason='REACQUIRING_CORRIDOR')
        return out

    def early_side(self,result,stamp,camera):
        """Opt-in selection from current support; never changes target limits.

        Existing dual geometry must succeed before this preference is applied.
        Ambiguities, new identities and missing observations retain the normal
        policy. History is six recent measurements, not accumulated drift.
        """
        self.side_evidence={}
        if not self.cfg.get('early_reliable_side',False):return None
        known=[self.ids.get('left'),self.ids.get('right')]
        pairs=result.get('pairs',[])
        eligible=(not self.latched and result.get('vision_valid') and
                  result.get('selected')==known and
                  result.get('reason')=='VISION_VALID_PIXELS_ONLY' and
                  len(pairs)==1 and pairs[0]['ids']==known and
                  len(self.widths)>=self.cfg.get('width_init_frames',8))
        if not eligible:
            self.side_vote=None;self.side_vote_count=0
            return None
        by_id={b['id']:b for b in result['candidates']}
        width=float(np.median(self.widths));target=self.cfg['lookahead_m']
        limit=self.cfg.get('single_target_clamp_m',.5)
        for side,i in self.ids.items():
            b=by_id.get(i)
            if b is None or b['score']<.7:continue
            try:
                self.continuity(b,stamp)
                p=self.boundary(b['points'],camera)
                end=next((j for j,q in enumerate(p) if np.linalg.norm(q)>target+width),len(p))
                p,t=smooth_boundary(p[:end])
                center=p+(-1 if side=='left' else 1)*width/2*np.c_[-t[:,1],t[:,0]]
                radius=np.linalg.norm(center,axis=1)
            except ValueError:continue
            pixels=np.asarray(b['points']);span=float(np.ptp(pixels[:,1]))
            edge=float(min(pixels[-1,0],camera['width']-1-pixels[-1,0]))
            old=[a for a in self.side_history.get(i,[]) if 0<(stamp-a[0])/1e9<=.5]
            shrinking=bool(len(old)>=3 and span<.8*max(a[1] for a in old))
            self.side_history[i]=(old+[(stamp,span)])[-6:]
            supports=bool(radius.min()-limit<=target<=radius.max()+limit)
            self.side_evidence[side]=dict(nearest_m=float(radius.min()),span_px=span,
                near_edge_px=edge,shrinking=shrinking,supports_target=supports)
        self.side_history={i:a for i,a in self.side_history.items() if i in known}
        vote=None
        if len(self.side_evidence)==2:
            for good,bad in [('left','right'),('right','left')]:
                a,z=self.side_evidence[good],self.side_evidence[bad]
                worse=(not z['supports_target'] or z['near_edge_px']<8 or z['shrinking'] or
                       z['span_px']<.8*a['span_px'])
                if a['supports_target'] and a['near_edge_px']>=8 and worse and z['nearest_m']-a['nearest_m']>.5:
                    vote=good
        self.side_vote_count=self.side_vote_count+1 if vote and vote==self.side_vote else int(vote is not None)
        self.side_vote=vote
        return vote if self.side_vote_count>=3 else None

    def preferred_observation(self,result,stamp,camera,side):
        # Both checks run on copies. Never hide a contradictory dual width or
        # mutate trusted width/identity while evaluating an alternative.
        dual=copy.deepcopy(self)
        super(QualityCorridor,dual).compute(result,stamp,camera)
        trial=copy.deepcopy(self);trial.ids={side:self.ids[side]}
        single=copy.copy(result)
        single.update(vision_valid=False,selected=[],reason='NO_SUPPORTED_EGO_PAIR')
        try:
            points,extra=super(QualityCorridor,trial).compute(single,stamp,camera)
        except ValueError:
            # A preferred side must pass the unchanged single-side checks.
            # Otherwise retain the already valid original dual computation.
            return None
        self.last_stamp=trial.last_stamp
        self.single_travel_bound=trial.single_travel_bound
        extra.update(early_side_selected=side,early_side_evidence=self.side_evidence,
                     width_updated=False,width_age_s=(stamp-self.last_dual)/1e9)
        return points,extra

    def deferred_anchor(self,result,stamp,camera):
        """A distant returning pair cannot veto a current, known near side.

        Do not accept its identity or update width. Every current candidate
        remains in the single-side interior-line contradiction check.
        """
        if self.latched or not self.ids or not self.widths or not result.get('pairs'):
            return None
        by={b['id']:b for b in result.get('candidates',[])}
        width=float(np.median(self.widths));choices=[]
        for side,identity in self.ids.items():
            if identity not in by or by[identity]['score']<.7:continue
            # A proposed role reversal is not a weak opposite-side return.
            if any(identity in p['ids'] and p['ids'].index(identity)!=['left','right'].index(side)
                   for p in result['pairs']):continue
            trial=copy.deepcopy(self);trial.ids={side:identity}
            single=copy.copy(result);single.update(vision_valid=False,selected=[],reason='NO_SUPPORTED_EGO_PAIR')
            try:
                points,extra=super(QualityCorridor,trial).compute(single,stamp,camera)
                anchor=self.boundary(by[identity]['points'],camera)
                # This fallback only uses the observed endpoint when it lies
                # beyond the requested circle; target selection is unchanged.
                if np.linalg.norm(points[0])<self.cfg['lookahead_m']:continue
                target_x=float(points[0,0]);deferred=[];ok=True
                for other_id,b in by.items():
                    if other_id==identity:continue
                    other=self.boundary(b['points'],camera)
                    left,right=(other,anchor) if side=='right' else (anchor,other)
                    values,_,window=near_width_observations(left,right,2*self.cfg.get('width_min_arc_m',1.5))
                    if window is not None:
                        if window[0]<=target_x+.15:ok=False;break
                    elif float(other[:,0].min())<=target_x+.15:
                        ok=False;break
                    deferred.append(other_id)
                if ok:choices.append((trial,points,extra,side,deferred))
            except ValueError:continue
        if len(choices)!=1:return None
        trial,points,extra,side,deferred=choices[0]
        self.single_travel_bound=trial.single_travel_bound
        extra.update(transition_policy='known_near_anchor_far_return_pending',
                     deferred_boundary_ids=deferred,width_updated=False)
        return points,extra

    def compute(self,result,stamp,camera):
        self.image_width=camera['width']
        if self.input_stamp is not None and not 0<(stamp-self.input_stamp)/1e9<=.5:
            self.reset();raise ValueError('CONTROL_TIME_DISCONTINUITY')
        self.input_stamp=stamp
        tracks={}
        for b in result.get('candidates',[]):
            points=np.asarray(b['points']);old=self.current_tracks.get(b['id']);count=1
            if old and 0<(stamp-old['stamp'])/1e9<=.15:
                a=old['points'];lo=max(a[0,1],points[0,1]);hi=min(a[-1,1],points[-1,1])
                if hi-lo>=24:
                    ys=np.linspace(max(lo,hi-60),hi,12)
                    error=np.percentile(abs(np.interp(ys,a[:,1],a[:,0])-np.interp(ys,points[:,1],points[:,0])),90)
                    if error<=12:count=min(old['count']+1,self.cfg.get('reacquire_frames',8))
            tracks[b['id']]=dict(points=points,stamp=stamp,count=count)
        self.current_tracks=tracks
        entry=copy.deepcopy(self)
        try:
            original=result;association_done=False
            try:
                result=self.reassociate(result,stamp,camera)
                association_done=True
                side=self.early_side(result,stamp,camera)
                preferred=self.preferred_observation(result,stamp,camera,side) if side else None
                points,extra=preferred if preferred is not None else super().compute(result,stamp,camera)
            except ValueError as error:
                if str(error) not in ('REACQUISITION_WIDTH_CONFLICT','REACQUISITION_IDENTITY_CONFLICT',
                                      'WIDTH_CHANGED','WIDTH_INCONSISTENT','CENTER_DISAGREEMENT','AMBIGUOUS_CORRIDOR'):
                    raise
                # Base geometry may latch a failed width hypothesis. A fallback
                # must be evaluated from the entry state, never from a fault.
                if association_done and self.reassociated:
                    # A confirmed returning side may now be the only NEAR
                    # observation. Let either verified role supply the target.
                    entry.ids=dict(zip(('left','right'),self.reassociated))
                fallback=entry.deferred_anchor(result if association_done else original,stamp,camera)
                if fallback is None:raise
                if association_done:
                    # Geometry can remain single-sided after a returning side
                    # has passed temporal identity checks. Keep that evidence;
                    # do not restart its confirmation counter on every frame.
                    for key in ('pending_pair','pending_count','pending_measurements','reassociated'):
                        setattr(entry,key,copy.deepcopy(getattr(self,key)))
                    if self.reassociated:
                        entry.ids=dict(zip(('left','right'),self.reassociated))
                        fallback[1]['reassociated_pair']=self.reassociated
                self.__dict__.update(entry.__dict__)
                points,extra=fallback
            if extra.get('observation_mode','').startswith('single_') and self.cfg.get('single_fit_arc_m') is not None:
                extra.update(single_observation_horizon_m=None,single_fit_arc_m=self.cfg['single_fit_arc_m'])
            if self.cfg.get('early_reliable_side',False):
                extra.setdefault('early_side_selected',None)
                extra.setdefault('early_side_evidence',self.side_evidence)
            for b in result.get('candidates',[]):
                if b['id'] in self.ids.values():self.observed[b['id']]=(stamp,np.asarray(b['points']))
            return points,extra
        except ValueError as exc:
            # Once a real-data preview session is valid, all faults latch.
            # Observations returning cannot silently rearm the driving gate.
            if self.ids and str(exc)!='CONTROL_TIME_DISCONTINUITY':self.latched=True
            raise
        finally:
            # Observation time advances even when geometry is rejected. This
            # does not clear the driving latch or renew a path/width estimate.
            self.last_stamp=stamp
