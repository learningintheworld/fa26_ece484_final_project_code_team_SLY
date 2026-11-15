import numpy as np
import pytest
from sly_lane_following.control import ImageServo,CommandGate,front_to_steering_wheel
from sly_lane_following.core import LaneDetector
from test_core import boundary

CFG=dict(reference_verified=True,near_row=.85,far_row=.64,reference_near=.5,reference_far=.5,
         min_span_fraction=.08,k_near=.65,k_direction=.3,smoothing_s=.12,max_steering=.55,
         steering_rate=.6,speed=.4,turn_slowdown=2.)
def scene(xnear=384,xfar=384):
 return dict(vision_valid=True,center=[[xfar,300],[xnear,450]])
def test_steering_sign_and_limits():
 for sign in (-1,1):
  servo=ImageServo(CFG)
  for i in range(60):r=servo.compute(scene(384+sign*100,384+sign*100),(480,768),int((i+1)*1e9/30))
  assert r['steering_angle']*sign<0
  assert abs(r['steering_angle'])<=CFG['max_steering']
  assert 0<r['speed']<=CFG['speed']
def test_reference_and_current_evidence_are_required():
 s=ImageServo({**CFG,'reference_verified':False})
 assert s.compute(scene(),(480,768),1)['reason']=='IMAGE_REFERENCE_REQUIRED'
 s=ImageServo(CFG)
 assert s.compute(dict(vision_valid=False),(480,768),1)['speed']==0
 assert s.compute(dict(vision_valid=True,center=[[384,400],[384,410]]),(480,768),1)['speed']==0

def test_rate_limit_and_clock_rewind():
 s=ImageServo(CFG);last=0
 for i in range(1,30):
  r=s.compute(scene(700,700),(480,768),int(i*1e9/30))
  assert abs(r['steering_angle']-last)<=CFG['steering_rate']/30+1e-8
  last=r['steering_angle']
 assert s.compute(scene(),(480,768),1)['reason']=='CONTROL_TIME_DISCONTINUITY'

@pytest.mark.parametrize('fault',['old_request','old_feedback','invalid','override','competing','future','nan','backward'])
def test_fault_latches_and_fresh_input_does_not_restart(fault):
 g=CommandGate();assert g.arm(10,(10,.3,.1),10,0)[0]
 args=[10.1,(10.1,.3,.1),10.1,0];flags={}
 if fault=='old_request':args[1]=(9,.3,.1)
 if fault=='old_feedback':args[2]=9
 if fault=='invalid':args[1]=(10.1,0,0)
 if fault=='override':flags['override']=True
 if fault=='competing':flags['competing']=True
 if fault=='future':args[1]=(11,.3,.1)
 if fault=='nan':args[1]=(10.1,.3,float('nan'))
 if fault=='backward':args[0]=9
 assert g.step(*args,**flags)==(0,0)
 assert not g.enabled
 assert g.step(11,(11,.3,.1),11,0)==(0,0)
 assert g.arm(11,(11,.3,.1),11,0)[0]

def test_role_reversal_rejected_without_reinterpreting_shared_line():
 d=LaneDetector();d.roles={2:'right'}
 chosen,_,_,rejected=d.select([boundary(2,220),boundary(3,550)],768,480,384)
 assert chosen is None
 assert any(x['reason']=='BOUNDARY_ROLE_CONFLICT' for x in rejected)

def test_lost_identity_requires_explicit_reset():
 d=LaneDetector();d.identity_lost=True
 assert d.select([boundary(2,220),boundary(3,550)],768,480,384)[1]=='CORRIDOR_LOST_REARM_REQUIRED'
 d.reset()
 assert d.select([boundary(2,220),boundary(3,550)],768,480,384)[0] is not None

def test_vehicle_mapping_is_odd_and_has_a_valid_range():
 assert front_to_steering_wheel(.1)==-front_to_steering_wheel(-.1)
 assert front_to_steering_wheel(0)==0
 with pytest.raises(ValueError):front_to_steering_wheel(1.)

def test_tracked_corridor_can_move_away_from_image_center_in_bend():
 d=LaneDetector();d.roles={1:'left',2:'right'};d.previous_pair=(1,2)
 chosen,_,_,_=d.select([boundary(1,60,150),boundary(2,350,320)],768,480,384)
 assert chosen is not None and chosen['ids']==[1,2]
 # The same pair is unsuitable as a fresh ambiguous/off-centre initialization.
 assert LaneDetector().select([boundary(1,60,150),boundary(2,350,320)],768,480,384)[0] is None

def test_short_fixed_sample_window_uses_remaining_observed_range():
 s=ImageServo(CFG)
 r=s.compute(dict(vision_valid=True,center=[[380,380],[390,475]]),(480,768),1_000_000_000)
 assert r['valid']
 assert 380<=r['far'][1]<r['near'][1]<=475

def test_ros_float32_limit_rounding_is_clamped_not_a_false_fault():
 g=CommandGate(max_speed=.2)
 assert g.arm(1,(1,float(np.float32(.2)),.1),1,0)[0]
 assert g.step(1.1,(1.1,float(np.float32(.2)),.1),1.1,0)[0]==.2
 assert g.step(1.2,(1.2,.201,.1),1.2,0)==(0,0)

def test_only_unique_continuous_paint_fragments_join():
 d=LaneDetector()
 a=boundary(1,300,280,y_min=300,y_max=400)
 b=boundary(2,320,296,y_min=380,y_max=500)
 combined=d.join_fragments([a,b])
 assert len(combined)==1 and combined[0]['y_min']==300 and combined[0]['y_max']==500
 c=boundary(3,325,300,y_min=380,y_max=500)
 # A competing continuation is a branch, not permission to merge.
 assert len(d.join_fragments([a,b,c]))==3

def test_far_adjacent_paint_fragments_are_not_joined():
 d=LaneDetector()
 a=boundary(1,300,280,y_min=300,y_max=400)
 b=boundary(2,420,396,y_min=380,y_max=500)
 assert len(d.join_fragments([a,b]))==2

def test_merged_fragment_keeps_known_boundary_role():
 d=LaneDetector();d.roles={2:'right'}
 known=boundary(2,500,440,y_min=300,y_max=450)
 fragment=boundary(9,503,460,y_min=340,y_max=475)
 d.memory={2:(known,1_000_000_000),9:(fragment,1_000_000_000)};d.next_id=10
 current=boundary(10,503,440,y_min=300,y_max=475);current.pop('id')
 d.associate([current],stamp=1_033_333_333)
 assert current['id']==2

def test_tangent_direction_is_invariant_to_sample_spacing():
 cfg={**CFG,'direction_mode':'tangent'}
 # The same image line sampled at different row spacings has the same angle.
 a=ImageServo(cfg).compute(dict(vision_valid=True,center=[[330,300],[405,450]]),(480,768),1_000_000_000)
 b=ImageServo({**cfg,'near_row':.9,'far_row':.7}).compute(dict(vision_valid=True,center=[[330,300],[405,450]]),(480,768),1_000_000_000)
 assert a['valid'] and b['valid']
 assert a['direction_error']==pytest.approx(b['direction_error'])

def test_tangent_mode_respects_reference_and_steering_sign():
 cfg={**CFG,'direction_mode':'tangent','reference_far':.55,'reference_near':.50,'k_near':0.}
 # Actual line exactly matches the reference slope.
 center=[[.55*768,.64*480],[.5*768,.85*480]]
 r=ImageServo(cfg).compute(dict(vision_valid=True,center=center),(480,768),1_000_000_000)
 assert r['steering_angle']==pytest.approx(0.)
 r=ImageServo({**CFG,'direction_mode':'tangent','k_near':0.}).compute(scene(384,450),(480,768),1_000_000_000)
 assert r['direction_error']>0 and r['steering_angle']<0

def test_unknown_direction_mode_is_rejected():
 with pytest.raises(ValueError):ImageServo({**CFG,'direction_mode':'world_heading'})
