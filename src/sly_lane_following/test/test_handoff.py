import copy
import numpy as np
from sly_lane_following.handoff import near_evidence,relative_to_path,UnifiedLaneTracker
from sly_lane_following.geometry import GroundProjector
from test_single_boundary import CONFIG,CAM,detection,straight,warm

P=np.c_[np.linspace(4.5,6.5,40),np.zeros(40)]

def test_far_branch_cannot_veto_near_lane_but_near_interior_does():
 p=GroundProjector(CONFIG)
 a=detection(np.c_[np.linspace(8,11,30),np.ones(30)*1.6],np.c_[np.linspace(8,11,30),np.ones(30)*-1.6])
 e=near_evidence(a['candidates'],P,3.2,p,CAM);assert not e['conflict'] and not e['matches']
 b=detection(np.c_[np.linspace(4.5,6.5,30),np.ones(30)*.5],np.c_[np.linspace(4.5,6.5,30),np.ones(30)*-1.6])
 assert near_evidence(b['candidates'],P,3.2,p,CAM)['conflict']

def test_fragment_id_does_not_determine_side():
 p=GroundProjector(CONFIG);a=straight();a['candidates'][0]['id']=41;a['candidates'][1]['id']=99
 e=near_evidence(a['candidates'],P,3.2,p,CAM)
 assert not e['conflict'];assert {(m['side'],m['id']) for m in e['matches']}=={('left',41),('right',99)}

def test_no_endpoint_extrapolation_for_conflict_classification():
 good,d=relative_to_path(np.array([[3.,0.],[5.,.4],[8.,0.]]),P)
 assert good.tolist()==[False,True,False];assert abs(d[1]-.4)<1e-9

def test_pending_far_ambiguity_uses_cache_but_real_conflict_stops():
 s=warm(straight());h=UnifiedLaneTracker(CONFIG)
 b=s.compute(straight(),(600,800),1_300_000_000,CAM)
 assert h.step(b,straight(),s.corridor,1_300_000_000,CAM,speed=.1,front_angle=0,feedback_stamp=1_300_000_000,motion_error_bound_m=0)['valid']
 far=detection(np.c_[np.linspace(9,11,30),np.ones(30)*1.6],np.c_[np.linspace(9,11,30),np.ones(30)*-1.6]);bad=dict(valid=False,reason='AMBIGUOUS_CORRIDOR')
 o=h.step(bad,far,s.corridor,1_333_000_000,CAM,speed=.1,front_angle=0,feedback_stamp=1_333_000_000,motion_error_bound_m=0)
 assert o['valid'] and o['mode']=='predicted_pending_handoff'
 near=detection(np.c_[np.linspace(4.5,6.5,30),np.ones(30)*.5],np.c_[np.linspace(4.5,6.5,30),np.ones(30)*-1.6])
 o=h.step(bad,near,s.corridor,1_366_000_000,CAM,speed=.1,front_angle=0,feedback_stamp=1_366_000_000,motion_error_bound_m=0)
 assert not o['valid'] and o['reason']=='NEAR_CORRIDOR_CONFLICT' and h.cache.path is None

def test_no_initial_path_no_prediction_and_stale_motion_stops():
 h=UnifiedLaneTracker(CONFIG);s=warm(straight());bad=dict(valid=False,reason='AMBIGUOUS_CORRIDOR')
 assert not h.step(bad,straight(),s.corridor,1_300_000_000,CAM,speed=.1,front_angle=0,feedback_stamp=1_300_000_000,motion_error_bound_m=0)['valid']
 b=s.compute(straight(),(600,800),1_400_000_000,CAM)
 assert not h.step(b,straight(),s.corridor,1_400_000_000,CAM,speed=.1,front_angle=0,feedback_stamp=1_000_000_000,motion_error_bound_m=0)['valid']


def disjoint_known_sides_fixture():
 from test_width_history import QUALITY,warm as warm_quality
 cfg={**QUALITY,'single_observed_target_max_m':7.,'single_fit_arc_m':1.5}
 geometry=warm_quality(cfg).corridor
 x=np.linspace(5.1,6.1,35);z=np.linspace(6.6,8.2,45)
 r=detection(np.c_[x,x*0+1.6],np.c_[z,z*0-1.6])
 h=UnifiedLaneTracker(cfg);h.ids={'left':1,'right':2};h.width=3.2
 for stamp in [2_000_000_000,2_033_333_333,2_066_666_666]:previous=h.track_known_observations(r,stamp)
 return h,geometry,r,previous,stamp


def test_confirmed_near_side_can_update_before_stale_cache_conflict():
 h,g,r,previous,stamp=disjoint_known_sides_fixture()
 bad=dict(valid=False,reason='NO_RELIABLE_SINGLE_BOUNDARY')
 offer=h.known_near_alternative(bad,r,g,stamp,CAM,previous,())
 assert offer is not None and offer[1]['observed_id']==1
 assert np.max(abs(offer[0][:,1]))<1e-6
 # A displaced old reference incorrectly places the current left edge inside.
 old=np.c_[np.linspace(5.1,6.1,35),np.full(35,.5)]
 assert near_evidence(r['candidates'],old,3.2,g.projector,CAM)['conflict']
 h.cache.advance(stamp,0.,0.,stamp);h.cache.observe(old,stamp,h.ids)
 out=h.step(bad,r,g,stamp+33_333_333,CAM,speed=.1,front_angle=0.,feedback_stamp=stamp+33_333_333,motion_error_bound_m=0.)
 assert out['valid'] and out['mode']=='observed_near_handoff'
 assert not out['stop_required'] and out['path_update_state']=='updated'
 assert out['handoff_evidence']['conflict']
 assert not out['current_handoff_evidence']['conflict']


def test_nearer_side_never_hides_actual_current_interior_line():
 h,g,r,previous,stamp=disjoint_known_sides_fixture()
 x=np.linspace(5.2,6.,30)
 extra=detection(np.c_[x,x*0+.5],np.c_[x,x*0-1.6])['candidates'][0];extra['id']=99
 r['candidates'].append(extra);bad=dict(valid=False,reason='NO_RELIABLE_SINGLE_BOUNDARY')
 assert h.known_near_alternative(bad,r,g,stamp,CAM,previous,()) is None
 old=np.c_[np.linspace(5.1,6.1,35),np.zeros(35)]
 h.cache.advance(stamp,0.,0.,stamp);h.cache.observe(old,stamp,h.ids)
 out=h.step(bad,r,g,stamp+33_333_333,CAM,speed=.1,front_angle=0.,feedback_stamp=stamp+33_333_333,motion_error_bound_m=0.)
 assert out['reason']=='NEAR_CORRIDOR_CONFLICT' and out['stop_required'] and out['speed']==0.


def test_near_handoff_requires_identity_quality_and_disjoint_support():
 bad=dict(valid=False,reason='NO_RELIABLE_SINGLE_BOUNDARY')
 for case in ['unknown','blocked','overlap','poor_quality','stale','ambiguous']:
  h,g,r,previous,stamp=disjoint_known_sides_fixture();blocked=();base=bad
  if case=='unknown':r['candidates'][0]['id']=99
  if case=='blocked':blocked=(1,)
  if case=='overlap':
   x=np.linspace(5.5,7.5,40)
   r['candidates'][1]=detection(np.c_[x,x*0+1.6],np.c_[x,x*0-1.6])['candidates'][1]
  if case=='poor_quality':r['candidates'][0]['residual_px']=20.
  if case=='stale':
   stamp+=1_000_000_000;previous=h.track_known_observations(r,stamp)
  if case=='ambiguous':base=dict(valid=False,reason='AMBIGUOUS_CORRIDOR')
  assert h.known_near_alternative(base,r,g,stamp,CAM,previous,blocked) is None,case


def test_verified_fragment_handoff_updates_geometry_role_without_refreshing_width():
 """A returning side must see the same accepted anchor as the path cache."""
 from test_single_boundary import single
 from test_width_history import QUALITY,warm as warm_quality
 s=warm_quality(QUALITY);h=UnifiedLaneTracker(QUALITY)
 stamp=2_000_000_000;base=s.compute(straight(),(600,800),stamp,CAM)
 assert h.step(base,straight(),s.corridor,stamp,CAM,speed=.1,front_angle=0,
     feedback_stamp=stamp,motion_error_bound_m=0)['valid']
 last_dual=s.corridor.last_dual;widths=s.corridor.widths.copy()
 r=single(straight(),'right');r['candidates'][0]['id']=99
 for j in range(1,4):
  stamp=2_000_000_000+j*33_333_333
  o=h.step(dict(valid=False,reason='NO_RELIABLE_SINGLE_BOUNDARY'),r,s.corridor,stamp,CAM,
      speed=.1,front_angle=0,feedback_stamp=stamp,motion_error_bound_m=0)
 assert o['valid'] and o['mode']=='observed_handoff'
 assert h.ids['right']==s.corridor.ids['right']==99
 assert s.corridor.observed[99][0]==stamp
 assert s.corridor.last_dual==last_dual and s.corridor.widths==widths
 # New upstream computation can use the confirmed role without reinitializing.
 next_base=s.compute(r,(600,800),stamp+33_333_333,CAM)
 assert next_base['valid'] and next_base['observed_id']==99


def test_new_id_not_confirmed_by_one_frame_or_competing_current_edge():
 from test_single_boundary import single
 for competing in [False,True]:
  s=warm(straight());h=UnifiedLaneTracker(CONFIG);stamp=1_300_000_000
  base=s.compute(straight(),(600,800),stamp,CAM)
  h.step(base,straight(),s.corridor,stamp,CAM,speed=.1,front_angle=0,feedback_stamp=stamp,motion_error_bound_m=0)
  r=single(straight(),'right');r['candidates'][0]['id']=99
  if competing:
   other=copy.deepcopy(r['candidates'][0]);other['id']=100;r['candidates'].append(other)
  for j in range(1,5 if competing else 2):
   stamp+=33_333_333
   o=h.step(dict(valid=False,reason='NO_RELIABLE_SINGLE_BOUNDARY'),r,s.corridor,stamp,CAM,
      speed=.1,front_angle=0,feedback_stamp=stamp,motion_error_bound_m=0)
  assert o['observation_mode']=='predicted_cached_path'
  assert h.ids['right']==s.corridor.ids['right']==2
  assert not o.get('identity_commit')


def test_current_proposal_cannot_hide_a_real_interior_competitor():
 s=warm(straight());h=UnifiedLaneTracker(CONFIG);r=straight();stamp=1_300_000_000
 base=s.compute(r,(600,800),stamp,CAM)
 h.step(base,r,s.corridor,stamp,CAM,speed=.1,front_angle=0,feedback_stamp=stamp,motion_error_bound_m=0)
 x=np.linspace(4.5,6.5,30);extra=detection(np.c_[x,x*0+.5],np.c_[x,x*0-1.6])['candidates'][0];extra['id']=99
 r['candidates'].append(extra);stamp+=33_333_333
 out=h.step(base,r,s.corridor,stamp,CAM,speed=.1,front_angle=0,feedback_stamp=stamp,motion_error_bound_m=0)
 assert not out['valid'] and out['reason']=='NEAR_CORRIDOR_CONFLICT'
 assert out['current_handoff_evidence']['conflict']


def test_segment_quality_keeps_far_support_separate_from_role_evidence():
 from sly_lane_following.handoff import segment_quality
 s=warm(straight());r=straight()
 q=segment_quality(r['candidates'],s.corridor,CAM,{'left':1,'right':2})
 assert {'near','preview'}<={v['band'] for v in q}
 assert all(v['radius_m'][1]<=7 for v in q if v['band']=='near')
 assert all(v['radius_m'][0]>7 and v['use']=='confirmed_role_preview_only' for v in q if v['band']=='preview')
 assert all(v['role_confirmed'] and v['observed_arc_m']>0 for v in q)
