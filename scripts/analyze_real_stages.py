"""Offline RGB -> centre -> candidate commands. No ROS publisher or actuation.
Temporal budget is reported separately; physical driving gates are unchanged.
"""
import argparse,copy,json,hashlib,math,sys
from pathlib import Path
from collections import Counter
import cv2,numpy as np,yaml
from sly_lane_following.core import LaneDetector
from sly_lane_following.geometry import VisualPurePursuit
from sly_lane_following.real_corridor import QualityCorridor
from sly_lane_following.offline import bag_images,evaluate_reference

class PreviewCorridor(QualityCorridor):
    """Only instantiated by this offline script, never by the vehicle node."""
    def single_limit(self,stamp,points,candidate,camera):
        info=self.single_budget(stamp,points,candidate,camera)
        static=self.width_radius/2+info['observation_error_budget_m']
        if static>self.cfg.get('single_error_budget_m',.25):
            raise ValueError('CURRENT_WIDTH_OR_OBSERVATION_UNCERTAIN')
        info.update(offline_only=True,static_error_budget_m=static,
                    temporal_budget_exceeded=info['target_error_budget_m']>self.cfg.get('single_error_budget_m',.25))
        return info

def blocked_neighbor(result,old):
    # Same observed pixels as user-confirmed neighboring stripe; IDs are local
    # to each diagnostic session, so numeric ID34 alone cannot be reused.
    unwanted=next((b for b in old.get('candidates',[]) if b['id']==34),None)
    if unwanted is None:return result,[]
    u=np.array(unwanted['points']);blocked=[]
    for b in result['candidates']:
        p=np.array(b['points']);lo=max(u[0,1],p[0,1]);hi=min(u[-1,1],p[-1,1])
        if hi-lo<24:continue
        ys=np.linspace(lo,hi,20)
        if np.percentile(abs(np.interp(ys,u[:,1],u[:,0])-np.interp(ys,p[:,1],p[:,0])),90)<8:blocked.append(b['id'])
    r=copy.deepcopy(result);r['pairs']=[p for p in r['pairs'] if not any(i in blocked for i in p['ids'])]
    if any(i in blocked for i in r['selected']):r.update(vision_valid=False,selected=[],reason='NO_SUPPORTED_EGO_PAIR')
    return r,blocked

def main():
    root=Path(__file__).resolve().parents[1]
    a=argparse.ArgumentParser(description=__doc__);a.add_argument('--bag',type=Path,default=Path('/data/gem/oak_front_image'));a.add_argument('--output',type=Path,default=root/'results/three_stage_v1');a.add_argument('--start',type=float,default=0.);a.add_argument('--end',type=float,default=float('inf'));a.add_argument('--handoff',action='store_true',help='Offline unified handoff with explicitly hypothetical motion');a.add_argument('--physical-identity',action='store_true',help='Opt-in raw-paint connections and RGB physical IDs; does not authorize new lane roles');args=a.parse_args()
    if args.bag.resolve() in args.output.resolve().parents:raise ValueError('Output inside input bag')
    args.output.mkdir(parents=True,exist_ok=False);out=args.output
    vc=yaml.safe_load((root/'src/sly_lane_following/config/real_enet.yaml').read_text());vc['enet_checkpoint']=str(root/vc['enet_checkpoint'])
    cc=yaml.safe_load((root/'src/sly_lane_following/config/e4_pure_pursuit.yaml').read_text())
    det=LaneDetector(vc)
    def controller():
        s=VisualPurePursuit(cc);s.corridor=PreviewCorridor(cc,s.projector);return s
    s=controller();previous_valid=False;reset_next=False;segment=0;rows=[];events=[]
    from sly_lane_following.handoff import UnifiedLaneTracker
    policy=UnifiedLaneTracker(cc) if args.handoff else None
    assumed_speed=0.;assumed_angle=0.
    from sly_lane_following.physical_identity import PhysicalStripeTracker
    physical=PhysicalStripeTracker(s.projector) if args.physical_identity else None

    old={x['stamp_ns']:x for x in map(json.loads,(root/'results/enet_real_v4_candidate/replay_v3/frames.jsonl').open()) if 33<=x['relative_s']<=40}
    refs={x['header_ns']:x for x in json.loads((root/'results/stage2/reference/reference.json').read_text())['frames']}
    protected=['models/enet_v3.pth']+[f'src/sly_lane_following/config/{name}.yaml' for name in ['real_enet','e4_pure_pursuit','highbay','highbay_enet','sim_pure_pursuit']]+['src/sly_lane_following/sly_lane_following/geometry.py']
    hashes={p:hashlib.sha256((root/p).read_bytes()).hexdigest() for p in protected}
    video=None
    for idx,(t,stamp,image,cam) in enumerate(bag_images(args.bag)):
        if t<args.start:continue
        if t>args.end:break
        if reset_next:
            # Offline retry retains measured width and boundary roles. A view
            # failure must not become permission to initialize a different lane.
            s.corridor.latched=False;reset_next=False
            events.append(dict(time_s=t,event='OFFLINE_RETRY_PRESERVING_IDENTITY',segment=segment))
        if det.identity_lost:
            det.reset();s=controller();segment+=1
            if policy:policy=UnifiedLaneTracker(cc);assumed_speed=0.;assumed_angle=0.
            events.append(dict(time_s=t,event='OFFLINE_DETECTOR_REINITIALIZATION',segment=segment))
        result=det.process(image,cam,stamp);debug=result.pop('debug_image')
        r,blocked=blocked_neighbor(result,old[stamp]) if stamp in old else (result,[])
        # Do not let a forbidden neighbor become controller history after reset.
        if blocked and any(i in s.corridor.ids.values() for i in blocked):s=controller()
        for b in r['candidates']:
            if b['id'] in s.corridor.ids.values() and 'fragment_reference_points' in b:
                s.corridor.observed[b['id']]=(b['fragment_reference_stamp_ns'],np.asarray(b['fragment_reference_points']))
        control=s.compute(r,image.shape,stamp,cam)
        # Current observations remain tracking evidence even when the target
        # is unavailable; never store a predicted centre as an observation.
        for b in r['candidates']:
            if b['id'] in s.corridor.ids.values():s.corridor.observed[b['id']]=(stamp,np.asarray(b['points']))
        if control.get('observed_id') in blocked or (control['valid'] and any(i in blocked for i in s.corridor.ids.values())):raise RuntimeError('Excluded neighbor used')
        base_control=control
        if policy:
            # Replay has no measured speed/steering. Counterfactual 20 Hz
            # feedback follows previous requests; images remain recorded.
            while policy.cache.motion_stamp is not None and stamp-policy.cache.motion_stamp>50_000_000:
                mt=policy.cache.motion_stamp+50_000_000
                policy.cache.advance(mt,assumed_speed,assumed_angle,mt)
            control=policy.step(base_control,r,s.corridor,stamp,cam,speed=assumed_speed,
                front_angle=assumed_angle,feedback_stamp=stamp,motion_error_bound_m=0.,blocked=blocked)
            control.setdefault('metric_reason',control['reason'])
            control.update(offline_only=True,motion_source='hypothetical_previous_requests_20Hz',not_closed_loop=True)
            if control.get('observed_id') in blocked:raise RuntimeError('Excluded neighbor used in handoff')
            assumed_speed=control['speed'];assumed_angle=control['steering_angle']
        valid=control['valid'];reason=control['reason']
        if previous_valid and not valid:events.append(dict(time_s=t,event='PREVIEW_INTERRUPTION',reason=reason,segment=segment))
        if s.corridor.latched:reset_next=True
        previous_valid=valid
        reference=evaluate_reference(result,refs[stamp],cam) if stamp in refs else None
        row=dict(time_s=t,stamp_ns=stamp,segment=segment,
            recognition=dict(candidate_count=len(result['candidates']),pair_valid=result['vision_valid'],reason=result['reason'],reference=reference),
            prediction=dict(boundary_ids=s.corridor.ids.copy(),valid=control['metric_valid'],reason=control['metric_reason'],path=control['metric_path'],mode=control.get('observation_mode'),blocked_neighbor_ids=blocked),
            command=dict(valid=valid,speed=control['speed'],steering_angle=control['steering_angle'],reason=reason,offline_only=True),control=control,candidates=result['candidates'],base_control=base_control)
        if physical is not None and det.maps is not None:
            roles={side_id:side for side,side_id in s.corridor.ids.items()
                   if control.get('metric_valid') and side_id in {b['id'] for b in r['candidates']}
                   and control.get('observed_id',side_id)==side_id}
            dual=list(roles) if control.get('observation_mode')=='dual' and control.get('width_updated') else []
            raw_gray=cv2.cvtColor(cv2.remap(image,*det.maps,cv2.INTER_LINEAR),cv2.COLOR_BGR2GRAY)
            row['physical_identity']=physical.update(raw_gray,r['candidates'],cam,stamp,
                confirmed_roles=roles,excluded_ids=blocked,verified_dual_ids=dual)
        rows.append(row)
        if valid:
            p=s.projector.image(control['metric_path'],cam);cv2.polylines(debug,[np.rint(p).astype('int32')],False,(255,0,255),2);cv2.circle(debug,tuple(np.rint(control['target_pixel']).astype(int)),5,(0,0,255),-1)
        cv2.rectangle(debug,(0,0),(debug.shape[1],88),(20,20,20),-1)
        for j,txt in enumerate([f'OFFLINE PREVIEW ONLY | {t:.3f}s | segment {segment}',f'Lines: {len(result["candidates"])} | centre/command: {reason}',f'Age budget exceeded: {control.get("temporal_budget_exceeded",False)} | v={control["speed"]:.3f} m/s']):cv2.putText(debug,txt,(8,22+26*j),0,.48,(255,255,255),1)
        if video is None:video=cv2.VideoWriter(str(out/'preview.avi'),cv2.VideoWriter_fourcc(*'MJPG'),10,(debug.shape[1],debug.shape[0]))
        if idx%3==0:video.write(debug)
        if idx%300==0:cv2.imwrite(str(out/f'frame_{t:06.2f}.jpg'),debug);print(json.dumps(dict(frame=idx,time_s=t,reason=reason)),flush=True)
    video.release()
    (out/'frames.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in rows));(out/'events.json').write_text(json.dumps(events,indent=2))
    counts=Counter(x['command']['reason'] for x in rows);references=[x['recognition']['reference'] for x in rows if x['recognition']['reference']]
    summary=dict(frames=len(rows),frames_with_candidates=sum(x['recognition']['candidate_count']>0 for x in rows),accepted_dual_pixel_frames=sum(x['recognition']['pair_valid'] for x in rows),center_preview_frames=sum(x['prediction']['valid'] for x in rows),candidate_command_frames=sum(x['command']['valid'] for x in rows),age_budget_exceeded_preview_frames=sum(x['control'].get('temporal_budget_exceeded',False) for x in rows),reasons=dict(counts),offline_reinitializations=sum('REINITIALIZATION' in x['event'] for x in events),interruptions=[x for x in events if x['event']=='PREVIEW_INTERRUPTION'],coarse_reference_comparable=sum(x.get('comparable',False) for x in references),coarse_reference_wrong_corridor_proxy=sum(x.get('wrong_corridor_proxy',False) for x in references),handoff_hypothetical_motion=bool(policy),scope='Actual v3 on all recorded RGB; offline reinitializations and static-width preview only. NOT continuous autonomous driving or ground-truth accuracy. No topics published.')
    (out/'summary.json').write_text(json.dumps(summary,indent=2));assert all(hashlib.sha256((root/p).read_bytes()).hexdigest()==v for p,v in hashes.items());(out/'protected_hashes.json').write_text(json.dumps(hashes,indent=2));print(json.dumps(summary,indent=2))
if __name__=='__main__':main()
