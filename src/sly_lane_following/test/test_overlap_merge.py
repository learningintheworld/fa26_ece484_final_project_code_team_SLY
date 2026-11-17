"""Duplicate scans of one stripe must not create artificial corridor forks."""
import cv2
import numpy as np
from sly_lane_following.core import LaneDetector,curve_x


def candidate(points,axis='row'):
    p=np.array(points,float)
    return dict(knots=p.tolist(),coef=[0,0,0],height=480,y_min=float(p[0,1]),y_max=float(p[-1,1]),
                support=len(p),score=1.,residual_px=0.,scan_axis=axis)


def mask_for(cs,thickness=5):
    mask=np.zeros((480,768),np.uint8)
    for c in cs:
        rows=np.linspace(c['y_min'],c['y_max'],160)
        cv2.polylines(mask,[np.rint(np.c_[curve_x(c,rows),rows]).astype(np.int32)],False,255,thickness)
    return mask


def test_curved_shared_overlap_preserves_current_near_extension():
    # Regression shape from 20.237s; endpoint tangents differ on the same bend.
    a=candidate([[7,392],[70,403],[94,414],[112,426],[124,443]],'column')
    b=candidate([[69,403],[98,415],[113,427],[126,445],[134,475]])
    joined=LaneDetector().merge_observed_overlap([a,b],mask_for([a,b]))
    assert len(joined)==1 and joined[0]['y_min']==392 and joined[0]['y_max']==475
    assert joined[0]['merged_fragments']==2
    assert np.allclose(curve_x(joined[0],[450,465]),curve_x(b,[450,465]),atol=.2)


def test_separate_close_parallel_stripes_do_not_merge_across_dark_gap():
    a=candidate([[100,350],[100,470]])
    b=candidate([[105,350],[105,470]])
    assert len(LaneDetector().merge_observed_overlap([a,b],mask_for([a,b],1)))==2


def test_diverging_branch_and_crossing_remain_distinct():
    a=candidate([[100,350],[120,410],[140,470]])
    for pts in [[[130,350],[120,410],[140,470]],[[140,350],[120,410],[100,470]]]:
        b=candidate(pts)
        assert len(LaneDetector().merge_observed_overlap([a,b],mask_for([a,b])))==2


def test_short_or_unsupported_overlap_does_not_join():
    a=candidate([[100,350],[120,400]])
    b=candidate([[116,390],[148,470]])
    assert len(LaneDetector().merge_observed_overlap([a,b],mask_for([a,b])))==2
    b=candidate([[100,350],[120,400]])
    assert len(LaneDetector().merge_observed_overlap([a,b],np.zeros((480,768),np.uint8)))==2


def test_overlap_merge_is_opt_in():
    d=LaneDetector()
    def forbidden(*args):raise AssertionError('legacy/simulator configuration changed')
    d.merge_observed_overlap=forbidden
    assert d.candidates(np.zeros((480,768),np.uint8))==[]


def test_unobserved_connector_is_split_before_identity_assignment():
    # 14.77s regression: a row track jumped from near left to far right paint.
    near=candidate([[205,315],[112,477]])
    far=candidate([[217,290],[330,298]],'column')
    broken=candidate([[217,290],[205,315],[112,477]])
    mask=mask_for([near,far],3)
    parts=LaneDetector().split_mask_gaps([broken],mask)
    assert len(parts)==1 and parts[0]['y_min']>310
    assert parts[0]['y_max']==477 and parts[0]['split_unobserved_connector']


def test_short_occlusion_and_observed_curved_stripe_are_preserved():
    c=candidate([[100,300],[140,350],[150,470]])
    mask=mask_for([c],5);mask[370:377]=0
    assert LaneDetector().split_mask_gaps([c],mask)==[c]


def test_gap_check_keeps_two_separate_long_observations():
    c=candidate([[100,275],[100,475]])
    mask=mask_for([c],3);mask[360:380]=0
    parts=LaneDetector().split_mask_gaps([c],mask)
    assert len(parts)==2
    assert parts[0]['y_max']<362 and parts[1]['y_min']>378


def test_gap_check_does_not_create_a_boundary_from_empty_input():
    c=candidate([[100,275],[100,475]])
    assert LaneDetector().split_mask_gaps([c],np.zeros((480,768),np.uint8))==[]
