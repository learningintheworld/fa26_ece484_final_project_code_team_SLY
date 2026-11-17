import copy
import numpy as np
import pytest
from sly_lane_following.geometry import GroundProjector,VisualPurePursuit,normal_widths
from test_geometry import CFG,CAM

CONFIG={**CFG,'lookahead_m':4.5,'allow_single_boundary':True,'width_init_frames':3,'speed':.2,
        'single_max_s':20.,'single_max_distance_m':2.,'boundary_range_m':12.}
def detection(left,right):
    projector=GroundProjector(CONFIG);candidates=[]
    for identity,p in [(1,left),(2,right)]:
        px=projector.image(p,CAM)
        px=px[np.argsort(px[:,1])]
        candidates.append(dict(id=identity,points=px.tolist(),score=1.))
    return dict(vision_valid=True,selected=[1,2],candidates=candidates,reason='VISION_VALID_PIXELS_ONLY')

def straight():
    x=np.linspace(4,11,60)
    return detection(np.c_[x,np.full(len(x),1.6)],np.c_[x,np.full(len(x),-1.6)])

def warm(result,config=CONFIG):
    servo=VisualPurePursuit(config)
    for i in range(3):
        out=servo.compute(result,(600,800),int(1e9+i*1e8),CAM)
    assert out['valid'],out
    return servo

def single(result,side):
    r=copy.deepcopy(result);r['vision_valid']=False;r['reason']='NO_SUPPORTED_EGO_PAIR'
    r['candidates']=[c for c in r['candidates'] if c['id']==(1 if side=='left' else 2)]
    return r

@pytest.mark.parametrize('side',['left','right'])
def test_single_straight_is_centered_and_width_is_observed(side):
    r=straight();s=warm(r);o=s.compute(single(r,side),(600,800),int(1.3e9),CAM)
    assert o['valid'] and o['observation_mode']=='single_'+side
    assert o['lane_width_m']==pytest.approx(3.2,abs=.001)
    assert np.max(abs(np.array(o['metric_path'])[:,1]))<.001
    assert o['speed']<=CONFIG['speed']*.75

@pytest.mark.parametrize('direction',[-1,1])
@pytest.mark.parametrize('side',['left','right'])
def test_single_arc_normal_offset_matches_independent_circle(direction,side):
    radius=6.56
    theta=np.linspace(.65,1.25,70)
    # Concentric road boundaries, with known independent circle center.
    left_radius=radius-direction*1.6;right_radius=radius+direction*1.6
    left=np.c_[left_radius*np.sin(theta),direction*(radius-left_radius*np.cos(theta))]
    right=np.c_[right_radius*np.sin(theta),direction*(radius-right_radius*np.cos(theta))]
    r=detection(left,right);s=warm(r);o=s.compute(single(r,side),(600,800),int(1.3e9),CAM)
    assert o['valid'],o
    pts=np.array(o['metric_path'])
    error=abs(np.linalg.norm(pts-[0,direction*radius],axis=1)-radius)
    assert np.percentile(error,95)<.04

def test_both_missing_latches_until_explicit_reset():
    s=warm(straight())
    bad=dict(vision_valid=False,reason='NO_SUPPORTED_EGO_PAIR',candidates=[])
    assert not s.compute(bad,(600,800),int(1.3e9),CAM)['valid']
    assert not s.compute(straight(),(600,800),int(1.4e9),CAM)['valid']
    s.reset()
    for i in range(3):o=s.compute(straight(),(600,800),int(2e9+i*1e8),CAM)
    assert o['valid']

def test_wrong_identity_is_not_replaced_by_bright_neighbor():
    r=single(straight(),'left');r['candidates'][0]['id']=99
    assert not warm(straight()).compute(r,(600,800),int(1.3e9),CAM)['valid']

def test_single_distance_bound_stops_even_with_visible_boundary():
    s=warm(straight(),{**CONFIG,'single_max_distance_m':.025})
    r=single(straight(),'left')
    assert s.compute(r,(600,800),int(1.3e9),CAM)['valid']
    o=s.compute(r,(600,800),int(1.4e9),CAM)
    assert not o['valid'] and o['reason']=='SINGLE_BOUNDARY_LIMIT'

def test_returning_wrong_width_is_rejected():
    s=warm(straight());s.compute(single(straight(),'left'),(600,800),int(1.3e9),CAM)
    x=np.linspace(4,11,60);r=detection(np.c_[x,x*0+1.6],np.c_[x,x*0-2.6])
    o=s.compute(r,(600,800),int(1.4e9),CAM)
    assert not o['valid'] and o['reason']=='WIDTH_CHANGED'

def test_ambiguity_does_not_fall_back_to_single():
    r=single(straight(),'left');r['reason']='AMBIGUOUS_CORRIDOR'
    assert not warm(straight()).compute(r,(600,800),int(1.3e9),CAM)['valid']

def test_image_masking_and_both_missing_through_detector():
    import cv2,yaml
    from pathlib import Path
    from sly_lane_following.core import LaneDetector
    cfg=yaml.safe_load((Path(__file__).parents[1]/'config/highbay.yaml').read_text())
    camera={**CAM,'D':[0.]*5,'distortion_model':'plumb_bob'}
    projector=GroundProjector(CONFIG);x=np.linspace(4,12,80)
    polylines=[np.rint(projector.image(np.c_[x,np.full(len(x),y)],camera)).astype(np.int32) for y in [1.6,-1.6]]
    def frame(sides):
        img=np.full((600,800,3),35,np.uint8)
        for i in sides:cv2.polylines(img,[polylines[i]],False,(0,220,220),6)
        return img
    detector=LaneDetector(cfg);servo=VisualPurePursuit(CONFIG)
    for i in range(30):
        result=detector.process(frame([0,1]),camera,1_000_000_000+i*33_333_333)
        out=servo.compute(result,(600,800),1_000_000_000+i*33_333_333,camera)
    assert out['valid'],out
    for i in range(30,40):
        result=detector.process(frame([0]),camera,1_000_000_000+i*33_333_333)
        out=servo.compute(result,(600,800),1_000_000_000+i*33_333_333,camera)
        assert out['valid'] and out['observation_mode']=='single_left',out
    result=detector.process(frame([]),camera,1_000_000_000+40*33_333_333)
    out=servo.compute(result,(600,800),1_000_000_000+40*33_333_333,camera)
    assert not out['valid'] and out['metric_path']==[]

def test_far_diverging_markings_truncate_path_without_losing_near_corridor():
    s=warm(straight())
    x=np.linspace(4,11,90)
    r=detection(np.c_[x,x*0+1.6],np.c_[x,-1.6+.5*np.maximum(x-8,0)**2])
    o=s.compute(r,(600,800),int(1.3e9),CAM)
    assert o['valid'],o
    assert o['truncated_at_disagreement']
    assert max(p[0] for p in o['metric_path'])<9
    assert abs(o['target_rear_m'][1])<.05


def test_sparse_far_fragment_is_not_joined_into_reliable_near_boundary():
    import yaml
    from pathlib import Path
    from sly_lane_following.core import LaneDetector
    cfg=yaml.safe_load((Path(__file__).parents[1]/'config/highbay.yaml').read_text())
    def fragment(lo,hi,score):
        return dict(coef=[0.,0.,200.],height=600,y_min=lo,y_max=hi,
                    support=20,residual_px=0.,score=score)
    detector=LaneDetector(cfg)
    assert len(detector.join_fragments([fragment(320,370,.54),fragment(360,595,1.)]))==2
    assert len(detector.join_fragments([fragment(320,370,.95),fragment(360,595,1.)]))==1


def test_single_far_branch_is_outside_local_target_but_near_conflict_stops():
    r=single(straight(),'left');projector=GroundProjector(CONFIG)
    def add_conflict(start,end):
        q=copy.deepcopy(r);x=np.linspace(start,end,30)
        px=projector.image(np.c_[x,x*0+.5],CAM);px=px[np.argsort(px[:,1])]
        q['candidates'].append(dict(id=99,points=px.tolist(),score=1.))
        return q
    far=warm(straight()).compute(add_conflict(9,11),(600,800),int(1.3e9),CAM)
    assert far['valid'],far
    near=warm(straight()).compute(add_conflict(4.5,6),(600,800),int(1.3e9),CAM)
    assert not near['valid'] and near['reason']=='SINGLE_CORRIDOR_CONTRADICTION'


def test_single_boundary_entirely_beyond_horizon_is_invalid_not_a_crash():
    s=warm(straight());r=single(straight(),'left')
    x=np.linspace(10,11,30);px=GroundProjector(CONFIG).image(np.c_[x,x*0+1.6],CAM)
    r['candidates'][0]['points']=px[np.argsort(px[:,1])].tolist()
    o=s.compute(r,(600,800),int(1.3e9),CAM)
    assert not o['valid'] and o['reason']=='NO_RELIABLE_SINGLE_BOUNDARY'


def test_adapter_retains_one_stop_pulse_between_command_timer_ticks():
    from types import SimpleNamespace
    from sly_lane_following.adapter import Adapter
    from sly_lane_following.control import CommandGate
    gate=CommandGate();gate.enabled=True
    adapter=SimpleNamespace(gate=gate,age_clock=lambda:1.)
    def request(speed):
        return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1,nanosec=0)),
                               drive=SimpleNamespace(speed=speed,steering_angle=.1))
    Adapter.on_request(adapter,request(0.))
    Adapter.on_request(adapter,request(.2))
    assert not gate.enabled and gate.reason=='STOP_REQUEST'


def test_short_other_side_remnant_does_not_block_target_supporting_side():
    s=warm(straight());r=copy.deepcopy(straight())
    r.update(vision_valid=False,reason='NO_SUPPORTED_EGO_PAIR')
    x=np.linspace(5.2,6,30);px=GroundProjector(CONFIG).image(np.c_[x,x*0+1.6],CAM)
    r['candidates'][0]['points']=px[np.argsort(px[:,1])].tolist()
    o=s.compute(r,(600,800),int(1.3e9),CAM)
    assert o['valid'] and o['observation_mode']=='single_right',o
    assert abs(o['target_rear_m'][1])<.001


def test_two_usable_but_unpaired_sides_are_not_arbitrarily_selected():
    r=straight();r.update(vision_valid=False,reason='NO_SUPPORTED_EGO_PAIR')
    o=warm(straight()).compute(r,(600,800),int(1.3e9),CAM)
    assert not o['valid'] and o['reason']=='NO_RELIABLE_SINGLE_BOUNDARY'


def test_single_target_can_clamp_to_observed_endpoint_without_extrapolation():
    s=warm(straight());r=single(straight(),'left')
    x=np.linspace(4.8,7,40);px=GroundProjector(CONFIG).image(np.c_[x,x*0+1.6],CAM)
    r['candidates'][0]['points']=px[np.argsort(px[:,1])].tolist()
    o=s.compute(r,(600,800),int(1.3e9),CAM)
    assert o['valid'] and o['lookahead_actual_m']==pytest.approx(4.8,abs=.001)
    assert o['target_rear_m'][0]>=4.8-.001


def test_metric_common_support_loss_uses_reliable_side_with_history():
    s=warm(straight());r=straight()
    x=np.linspace(5.2,5.6,30);px=GroundProjector(CONFIG).image(np.c_[x,x*0+1.6],CAM)
    r['candidates'][0]['points']=px[np.argsort(px[:,1])].tolist()
    o=s.compute(r,(600,800),int(1.3e9),CAM)
    assert o['valid'] and o['observation_mode']=='single_right',o
    assert o['lane_width_m']==pytest.approx(3.2,abs=.001)
