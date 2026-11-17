import math
from pathlib import Path
import numpy as np
import pytest
import yaml
from sly_lane_following.geometry import GroundProjector,VisualPurePursuit

CFG=yaml.safe_load((Path(__file__).parents[1]/'config/sim_pure_pursuit.yaml').read_text())
CFG['allow_single_boundary']=False
CAM=dict(width=800,height=600,K=[476.7,0,400,0,476.7,300,0,0,1])

def scene(points):
 pixels=GroundProjector(CFG).image(points,CAM)
 pixels=pixels[np.argsort(pixels[:,1])]
 return dict(vision_valid=True,center=pixels.tolist())

def test_projection_axes_metric_scale_and_roundtrip():
 p=GroundProjector(CFG)
 world=np.array([[4,0],[5,1],[6,-1]])
 image=p.image(world,CAM);back,valid=p.project(image,CAM)
 assert valid.all();assert np.allclose(back,world)
 assert image[1,0]<400 and image[2,0]>400
 assert np.allclose(p.project([[400,588]],CAM)[0],[[1.264+1.636*476.7/288,0]])

def test_horizon_and_behind_camera_are_not_valid_ground():
 points,valid=GroundProjector(CFG).project([[400,300],[400,100]],CAM)
 assert not valid.any()

@pytest.mark.parametrize('lateral',[-.6,0,.6])
def test_pursuit_sign_and_expected_geometry(lateral):
 c={**CFG,'smoothing_s':0.,'steering_rate':100.,'lookahead_m':4.5}
 r=VisualPurePursuit(c).compute(scene([[3.9,lateral],[6,lateral]]),(600,800),10**9,CAM)
 assert r['valid'];assert r['lookahead_actual_m']==pytest.approx(4.5)
 assert r['steering_angle']==pytest.approx(math.atan(2*1.75*lateral/4.5**2))

@pytest.mark.parametrize('sign',[-1,1])
def test_circle_requires_correct_curvature(sign):
 radius=6.56;theta=np.linspace(.58,1.1,60)
 points=np.c_[radius*np.sin(theta),sign*radius*(1-np.cos(theta))]
 c={**CFG,'smoothing_s':0.,'steering_rate':100.,'lookahead_m':4.5}
 r=VisualPurePursuit(c).compute(scene(points),(600,800),10**9,CAM)
 assert r['valid'];assert r['target_steering']==pytest.approx(sign*math.atan(1.75/radius),abs=.001)

@pytest.mark.parametrize('distance',[1.,20.])
def test_target_stays_within_observed_section(distance):
 c={**CFG,'lookahead_m':distance}
 r=VisualPurePursuit(c).compute(scene([[4,0],[6,0]]),(600,800),10**9,CAM)
 assert r['valid'];assert r['lookahead_actual_m']==pytest.approx(4 if distance==1 else 6)

def test_invalid_vision_discards_path_and_time_reset_stops():
 s=VisualPurePursuit(CFG);good=scene([[4,0],[6,0]])
 assert s.compute(good,(600,800),10**9,CAM)['valid']
 assert s.compute(good,(600,800),10**9,CAM)['reason']=='CONTROL_TIME_DISCONTINUITY'
 r=s.compute(dict(vision_valid=False,reason='AMBIGUOUS_CORRIDOR'),(600,800),2*10**9,CAM)
 assert r['speed']==0 and r['metric_path']==[] and not r['valid']

def test_short_visible_support_is_not_extrapolated():
 r=VisualPurePursuit(CFG).compute(scene([[4.4,0],[4.5,0]]),(600,800),10**9,CAM)
 assert not r['valid'] and r['reason']=='CONTROL_VIEW_TOO_SHORT'

def test_e4_uses_its_own_geometry():
 cfg=yaml.safe_load((Path(__file__).parents[1]/'config/e4_pure_pursuit.yaml').read_text())
 assert cfg['wheelbase']==pytest.approx(1.2746+1.2904)
 assert cfg['camera_xyz_rear']==pytest.approx([.535+1.2904,0,1.683+.268])

def test_bad_extrinsics_rejected():
 with pytest.raises(ValueError):GroundProjector({**CFG,'camera_xyz_rear':[0,0,0]})
