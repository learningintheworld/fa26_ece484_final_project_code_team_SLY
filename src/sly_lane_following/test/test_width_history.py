"""Observed width and single-side continuation, independent geometric scenes."""
import copy
import numpy as np
import pytest
from sly_lane_following.geometry import VisualPurePursuit,GroundProjector
from test_single_boundary import CONFIG,CAM,straight,single,detection

QUALITY={**CONFIG,'quality_width_history':True,'single_max_s':3.,'single_max_distance_m':.6,
         'width_history_frames':180,'single_error_budget_m':.25,'width_drift_mps':.02,'reacquire_frames':4}

def warm(config=QUALITY):
    s=VisualPurePursuit(config)
    for i in range(10):o=s.compute(straight(),(600,800),int((1+i*.1)*1e9),CAM)
    assert o['valid'],o
    return s

def advance(s,r,start=2.,end=6.):
    outputs=[]
    for t in np.arange(start,end,.1):outputs.append(s.compute(r,(600,800),round(t*1e9),CAM))
    return outputs

def pair_with_new_left(offset=0.):
    r=straight();r['candidates'][0]['id']=9
    if offset:
        x=np.linspace(4,11,60);px=GroundProjector(QUALITY).image(np.c_[x,x*0+1.6+offset],CAM)
        r['candidates'][0]['points']=px[np.argsort(px[:,1])].tolist()
    r.update(selected=[9,2],pairs=[dict(ids=[9,2])]);return r

def test_reliable_single_can_outlive_old_limit_but_budget_eventually_stops():
    s=warm();r=single(straight(),'right');out=advance(s,r,end=24.)
    assert all(x['valid'] for x in out[:40])
    assert any(x.get('beyond_legacy_limit') for x in out)
    fail=next(x for x in out if not x['valid']);assert fail['reason']=='SINGLE_UNCERTAINTY_LIMIT'
    assert not s.compute(straight(),(600,800),int(24.1e9),CAM)['valid']

def test_same_time_noisy_observation_is_rejected_without_waiting_for_timeout():
    s=warm();r=single(straight(),'right');r['candidates'][0]['residual_px']=20.
    o=s.compute(r,(600,800),int(2e9),CAM)
    assert not o['valid'] and o['reason']=='SINGLE_UNCERTAINTY_LIMIT'

def test_short_dual_remnant_does_not_refresh_width_age():
    s=warm();last=s.corridor.last_dual;history=s.corridor.widths.copy();r=straight()
    x=np.linspace(5.2,6.,40);px=GroundProjector(QUALITY).image(np.c_[x,x*0+1.6],CAM)
    r['candidates'][0]['points']=px[np.argsort(px[:,1])].tolist()
    out=advance(s,r,end=2.4)
    assert all(x['valid'] for x in out),out
    assert s.corridor.last_dual==last and s.corridor.widths==history
    assert out[-1].get('width_updated') is False
    assert out[-1]['width_radius_m']>s.corridor.width_radius
    assert out[-1]['target_error_budget_m']==pytest.approx(out[-1]['width_radius_m']/2+out[-1]['observation_error_budget_m'])

def test_returning_side_needs_repeated_geometry_and_keeps_observed_anchor():
    s=warm();advance(s,single(straight(),'right'),end=2.4)
    r=pair_with_new_left()
    for i in range(3):
        o=s.compute(r,(600,800),round((2.4+.1*i)*1e9),CAM)
        assert o['valid'] and o['observation_mode']=='single_right',o
        assert s.corridor.ids=={'left':1,'right':2}
    o=s.compute(r,(600,800),int(2.7e9),CAM)
    assert o['valid'] and o['reassociated_pair']==[9,2]
    assert s.corridor.ids=={'left':9,'right':2}

def test_unsupported_return_and_wrong_width_do_not_rearm():
    for offset in [.8,-.8]:
        s=warm();advance(s,single(straight(),'right'),end=2.4)
        o=s.compute(pair_with_new_left(offset),(600,800),int(2.4e9),CAM)
        assert not o['valid'] and o['reason']=='REACQUISITION_WIDTH_CONFLICT'
        assert not s.compute(straight(),(600,800),int(2.5e9),CAM)['valid']

def test_two_return_hypotheses_and_role_swap_stop():
    r=pair_with_new_left();other=copy.deepcopy(r['candidates'][0]);other['id']=10
    r['candidates'].append(other);r['pairs'].append(dict(ids=[10,2]))
    o=warm().compute(r,(600,800),int(2e9),CAM)
    assert not o['valid'] and o['reason']=='AMBIGUOUS_CORRIDOR'
    r=pair_with_new_left();r.update(selected=[2,9],pairs=[dict(ids=[2,9])])
    assert warm().compute(r,(600,800),int(2e9),CAM)['reason']=='REACQUISITION_IDENTITY_CONFLICT'

def test_both_missing_and_current_temporal_jump_latch():
    bad=dict(vision_valid=False,reason='NO_SUPPORTED_EGO_PAIR',candidates=[])
    for r in [bad,single(straight(),'right')]:
        if r is not bad:
            for p in r['candidates'][0]['points']:p[0]+=60
        s=warm();o=s.compute(r,(600,800),int(2e9),CAM)
        assert not o['valid'],o
        assert not s.compute(straight(),(600,800),int(2.1e9),CAM)['valid']

def test_time_rewind_discards_width_and_identity():
    s=warm();o=s.compute(straight(),(600,800),int(1e9),CAM)
    assert o['reason']=='CONTROL_TIME_DISCONTINUITY'
    assert not s.corridor.widths and not s.corridor.ids

@pytest.mark.parametrize('direction',[-1,1])
@pytest.mark.parametrize('side',['left','right'])
def test_nominal_budget_covers_independent_arc_geometry(direction,side):
    from sly_lane_following.geometry import GroundProjector
    radius=6.56;theta=np.linspace(.65,1.25,70)
    lr=radius-direction*1.6;rr=radius+direction*1.6
    left=np.c_[lr*np.sin(theta),direction*(radius-lr*np.cos(theta))]
    right=np.c_[rr*np.sin(theta),direction*(radius-rr*np.cos(theta))]
    # A wider synthetic camera makes BOTH circle boundaries visible; the old
    # geometry-only fixture put the inner boundary outside the actual image.
    camera={**CAM,'K':[200.,0.,400.,0.,300.,300.,0.,0.,1.]}
    r=dict(vision_valid=True,reason='VISION_VALID_PIXELS_ONLY',selected=[1,2],candidates=[])
    projector=GroundProjector(QUALITY)
    for identity,points in [(1,left),(2,right)]:
        px=projector.image(points,camera);assert ((px[:,0]>0)&(px[:,0]<800)&(px[:,1]>0)&(px[:,1]<600)).all()
        r['candidates'].append(dict(id=identity,score=1.,points=px[np.argsort(px[:,1])].tolist()))
    s=VisualPurePursuit({**QUALITY,'width_min_span_px':20.,'width_min_arc_m':.6})
    for i in range(10):o=s.compute(r,(600,800),int((1+.1*i)*1e9),camera)
    assert o['valid'],o
    for t in np.arange(2.,6.,.1):
        o=s.compute(single(r,side),(600,800),round(t*1e9),camera)
        assert o['valid'],o
        p=np.array(o['target_rear_m']);error=abs(np.linalg.norm(p-[0,direction*radius])-radius)
        assert error<=o['target_error_budget_m']


def test_far_returning_fragment_does_not_claim_identity_or_block_observed_target():
    s=warm();r=pair_with_new_left();x=np.linspace(9.,11.,30)
    px=GroundProjector(QUALITY).image(np.c_[x,x*0+1.6],CAM)
    r['candidates'][0]['points']=px[np.argsort(px[:,1])].tolist()
    o=s.compute(r,(600,800),int(2e9),CAM)
    assert o['valid'] and o['observation_mode']=='single_right'
    assert s.corridor.ids=={'left':1,'right':2}


@pytest.mark.parametrize('key,value',[('width_drift_mps',0.),('width_drift_mps',float('nan')),('single_error_budget_m',float('inf'))])
def test_invalid_error_budget_configuration_fails_closed(key,value):
    with pytest.raises(ValueError,match='INVALID_WIDTH_UNCERTAINTY_CONFIG'):
        VisualPurePursuit({**QUALITY,key:value})


def test_near_width_measurements_exclude_distant_divergence_without_changing_budget():
    from sly_lane_following.real_corridor import near_width_observations
    from sly_lane_following.geometry import normal_widths
    x=np.linspace(4.,12.,160)
    left=np.c_[x,1.6+.12*np.maximum(x-8.,0.)**2]
    right=np.c_[x,x*0-1.6]
    whole=normal_widths(left,right)
    values,arcs,window=near_width_observations(left,right,3.)
    assert np.percentile(abs(whole-np.median(whole)),95)>.5
    assert len(values)>=6 and min(arcs)>1.5
    assert np.median(values)==pytest.approx(3.2,abs=.005)
    assert np.max(abs(values-3.2))<.01
    assert window==pytest.approx([4.,7.])
    # The policy's configured budget and growth rate are not tuned by scope.
    assert QUALITY['single_error_budget_m']==.25 and QUALITY['width_drift_mps']==.02


def test_local_width_conflict_is_not_hidden_by_a_better_distant_section():
    from sly_lane_following.geometry import normal_widths
    s=warm();r=straight();x=np.linspace(4.,12.,160)
    left=np.c_[x,2.4-.8*np.clip((x-8.)/2.,0.,1.)];right=np.c_[x,x*0-1.6]
    values=normal_widths(left,right)
    by_id={b['id']:b for b in r['candidates']}
    with pytest.raises(ValueError,match='WIDTH_CHANGED'):
        s.corridor.propose_width(float(np.median(values)),float(np.median(abs(values-np.median(values)))),
                                {'left':left,'right':right},by_id,{'left':1,'right':2},int(2e9))
    assert s.corridor.latched


def test_local_normal_endpoints_do_not_need_a_common_forward_interval():
    from sly_lane_following.real_corridor import near_width_observations
    radius=6.56;theta=np.linspace(.65,1.25,100)
    left=np.c_[(radius-1.6)*np.sin(theta),radius-(radius-1.6)*np.cos(theta)]
    right=np.c_[(radius+1.6)*np.sin(theta),radius-(radius+1.6)*np.cos(theta)]
    assert left[:,0].max()<right[:,0].min()
    values,arcs,window=near_width_observations(left,right,1.2)
    assert len(values)>=6 and min(arcs)>.6
    assert np.max(abs(values-3.2))<.02
