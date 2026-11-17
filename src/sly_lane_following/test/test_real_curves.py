"""Real RGB shallow-arc regression; the simulator keeps row-only defaults."""
from pathlib import Path
import cv2
import numpy as np
import pytest
import yaml
from sly_lane_following.core import LaneDetector,curve_x
from sly_lane_following.geometry import GroundProjector,VisualPurePursuit
from test_single_boundary import CONFIG,straight,warm,single

REAL=Path(__file__).parents[1]/'config/real_bag.yaml'

def test_shallow_visible_arc_is_recovered_without_extrapolation():
    mask=np.zeros((480,768),np.uint8)
    x=np.arange(20,520);y=400+.00027*(x-20)**2
    cv2.polylines(mask,[np.rint(np.c_[x,y]).astype(np.int32)],False,255,5)
    assert not LaneDetector().candidates(mask)
    detector=LaneDetector(yaml.safe_load(REAL.read_text()));cs=detector.candidates(mask)
    assert cs
    best=max(cs,key=lambda c:c['y_max']-c['y_min'])
    rows=np.linspace(best['y_min']+3,best['y_max']-3,30)
    expected=20+np.sqrt((rows-400)/.00027)
    assert np.percentile(abs(curve_x(best,rows)-expected),95)<7
    assert 399<=best['y_min']<=405 and best['y_max']<=470


def test_empty_or_flat_horizontal_stripe_is_not_a_driveable_curve():
    detector=LaneDetector(yaml.safe_load(REAL.read_text()))
    mask=np.zeros((480,768),np.uint8)
    assert not detector.candidates(mask)
    cv2.line(mask,(10,420),(750,420),255,5)
    assert not detector.candidates(mask)


def test_tight_scan_junction_needs_current_paint():
    detector=LaneDetector({'column_scan':True})
    def curve(points):
        p=np.array(points,float)
        return dict(knots=p.tolist(),coef=[0,0,0],height=480,y_min=p[0,1],y_max=p[-1,1],
                    support=20,score=1.,residual_px=0.)
    far=curve([[100,350],[196,396],[202,402]])
    near=curve([[204,409],[204,415],[174,475]])
    blank=np.zeros((480,768),np.uint8)
    assert len(detector.join_fragments([far,near],blank))==2
    cv2.line(blank,(202,402),(204,409),255,3)
    assert len(detector.join_fragments([far,near],blank))==1
    # Legacy/simulator call still has its original tangent limit.
    assert len(detector.join_fragments([far,near]))==2


def test_real_endpoint_clamp_is_bounded_and_never_extrapolates():
    from test_geometry import CAM
    cfg={**CONFIG,'single_target_clamp_m':1.5}
    result=single(straight(),'left');x=np.linspace(5.6,8,40)
    px=GroundProjector(cfg).image(np.c_[x,np.full(len(x),1.6)],CAM)
    result['candidates'][0]['points']=px[np.argsort(px[:,1])].tolist()
    assert not warm(straight()).compute(result,(600,800),int(1.3e9),CAM)['valid']
    out=warm(straight(),cfg).compute(result,(600,800),int(1.3e9),CAM)
    assert out['valid'] and out['lookahead_actual_m']==pytest.approx(5.6,abs=.001)
    x=np.linspace(6.1,8,40);px=GroundProjector(cfg).image(np.c_[x,np.full(len(x),1.6)],CAM)
    result['candidates'][0]['points']=px[np.argsort(px[:,1])].tolist()
    assert not warm(straight(),cfg).compute(result,(600,800),int(1.3e9),CAM)['valid']


def test_real_width_memory_expiry_latches_and_does_not_resume():
    from test_geometry import CAM
    cfg={**CONFIG,'single_max_s':.15}
    servo=warm(straight(),cfg);r=single(straight(),'left')
    assert servo.compute(r,(600,800),int(1.3e9),CAM)['valid']
    out=servo.compute(r,(600,800),int(1.4e9),CAM)
    assert not out['valid'] and out['reason']=='SINGLE_BOUNDARY_LIMIT'
    assert not servo.compute(straight(),(600,800),int(1.5e9),CAM)['valid']
