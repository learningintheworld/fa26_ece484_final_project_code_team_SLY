import copy
import cv2
import numpy as np
from sly_lane_following.geometry import GroundProjector
from sly_lane_following.right_gap import RightGapBridge

CAM={'K':[400.,0.,384.,0.,400.,240.,0.,0.,1.]}
PROJECTOR=GroundProjector({'camera_xyz_rear':[1.8,0.,2.]})

def scene():
    p=np.c_[np.linspace(500,630,32),np.linspace(340,470,32)]
    c=dict(id=1,score=1.,points=p.tolist())
    mask=np.zeros((480,768),np.uint8)
    cv2.line(mask,(460,300),(480,320),255,3)
    cv2.line(mask,(500,340),(630,470),255,3)
    return c,mask

def advance(bridge,c,mask):
    return [bridge.propose(c,mask,int((1+i/30)*1e9),1,PROJECTOR,CAM) for i in range(3)]

def test_unique_gap_needs_temporal_confirmation_and_preserves_observation():
    c,m=scene();original=copy.deepcopy(c);b=RightGapBridge();o=advance(b,c,m)
    assert o[0] is None and o[1] is None and o[2] is not None,b.reason
    q=o[2];assert c==original and q['near_observed_points']==c['points']
    assert q['width_measurement_allowed'] is False
    assert q['points'][-1]==c['points'][-1]
    assert np.max(np.asarray(q['inferred_segments'][0])[:,1])<=c['points'][0][1]
    assert q['gap_metres']<=3 and q['bridge_error_bound_m']>0


def test_neighbor_left_identity_is_never_bridge_anchor():
    c,m=scene();c['id']=34
    assert all(x is None for x in advance(RightGapBridge(),c,m))


def test_parallel_competing_continuation_is_not_selected_by_iteration_order():
    c,m=scene();cv2.line(m,(468,300),(488,320),255,3);b=RightGapBridge()
    assert all(x is None for x in advance(b,c,m))
    assert b.reason=='AMBIGUOUS_GAP_CONTINUATION'


def test_no_paint_and_both_missing_do_not_make_a_bridge():
    c,m=scene();m[:]=0;b=RightGapBridge()
    assert all(x is None for x in advance(b,c,m))
    assert b.propose(None,m,int(2e9),1,PROJECTOR,CAM) is None


def test_long_gap_and_incompatible_direction_are_rejected():
    c,m=scene()
    for a,z in [((430,270),(450,290)),((450,320),(485,320))]:
        bad=np.zeros_like(m);cv2.line(bad,a,z,255,3)
        assert all(x is None for x in advance(RightGapBridge(),c,bad))


def test_time_gap_and_role_change_clear_confirmation():
    c,m=scene();b=RightGapBridge();assert advance(b,c,m)[-1] is not None
    assert b.propose(c,m,int(2e9),1,PROJECTOR,CAM) is None
    assert b.propose(c,m,int(2.03e9),34,PROJECTOR,CAM) is None
