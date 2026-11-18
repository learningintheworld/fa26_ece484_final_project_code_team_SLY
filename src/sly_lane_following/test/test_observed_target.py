import copy
import numpy as np
import pytest
from sly_lane_following.geometry import VisualPurePursuit
from test_width_history import QUALITY,warm
from test_single_boundary import CAM,detection,single,straight

CONFIG={**QUALITY,'single_observed_target_max_m':7.,'single_target_clamp_m':1.5}

def distant(start=6.05):
    x=np.linspace(start,11.,90)
    return single(detection(np.c_[x,x*0+1.6],np.c_[x,x*0-1.6]),'right')

def test_supported_endpoint_uses_actual_distance_without_fabricated_points():
    r=distant();saved=copy.deepcopy(r);s=warm(CONFIG)
    out=s.compute(r,(600,800),int(2e9),CAM)
    assert out['valid'],out
    assert out['lookahead_requested_m']==4.5
    assert out['lookahead_actual_m']==pytest.approx(6.05)
    assert np.min(np.asarray(out['metric_path'])[:,0])>=6.05-1e-8
    assert r==saved
    assert not warm(QUALITY).compute(r,(600,800),int(2e9),CAM)['valid']

@pytest.mark.parametrize('start',[7.01,8.])
def test_farther_than_cap_stops_and_latches(start):
    s=warm(CONFIG);out=s.compute(distant(start),(600,800),int(2e9),CAM)
    assert not out['valid']
    assert not s.compute(straight(),(600,800),int(2.1e9),CAM)['valid']

@pytest.mark.parametrize('fault',['missing','ambiguous','noise','jump','unknown'])
def test_new_target_gate_does_not_bypass_other_guards(fault):
    s=warm(CONFIG);r=distant()
    if fault=='missing':r['candidates']=[]
    if fault=='ambiguous':r['reason']='AMBIGUOUS_CORRIDOR'
    if fault=='noise':r['candidates'][0]['residual_px']=20.
    if fault=='jump':
        for p in r['candidates'][0]['points']:p[0]+=90
    if fault=='unknown':r['candidates'][0]['id']=34
    out=s.compute(r,(600,800),int(2e9),CAM)
    assert not out['valid'],out

@pytest.mark.parametrize('cap',[float('inf'),float('nan'),4.,31.])
def test_invalid_cap_rejected(cap):
    with pytest.raises(ValueError,match='INVALID_OBSERVED_TARGET_LIMIT'):
        VisualPurePursuit({**CONFIG,'single_observed_target_max_m':cap})


def test_local_arc_is_a_prefix_without_insertion_or_cross_fragment_extension():
    from sly_lane_following.geometry import smooth_boundary
    s=VisualPurePursuit({**CONFIG,'single_fit_arc_m':1.5})
    p=np.c_[np.linspace(7.2,10.2,31),np.full(31,-2.)]
    q=s.corridor.single_boundary_window(p,3.2)
    assert np.array_equal(q,p[:len(q)])
    assert np.linalg.norm(np.diff(q,axis=0),axis=1).sum()<=1.5+1e-10
    assert len(q)>=15
    # A fragment ending before enough support must stop, not reach for another.
    short=p[:5];limited=s.corridor.single_boundary_window(short,3.2)
    assert np.array_equal(limited,short)
    with pytest.raises(ValueError,match='BOUNDARY_SUPPORT_TOO_SHORT'):smooth_boundary(limited)


def test_no_local_arc_option_preserves_highbay_legacy_crop():
    from sly_lane_following.geometry import CorridorGeometry,GroundProjector
    c=CorridorGeometry(QUALITY,GroundProjector(QUALITY))
    p=np.c_[np.linspace(7.2,10.2,31),np.full(31,-2.)]
    q=c.single_boundary_window(p,3.2)
    assert np.array_equal(q,p[np.linalg.norm(p,axis=1)<=7.7])


@pytest.mark.parametrize('span',[float('nan'),float('inf'),.1])
def test_invalid_local_arc_rejected(span):
    with pytest.raises(ValueError,match='INVALID_SINGLE_FIT_ARC'):
        VisualPurePursuit({**CONFIG,'single_fit_arc_m':span})
