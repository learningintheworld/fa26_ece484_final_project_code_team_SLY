import numpy as np
import pytest
from sly_lane_following.finite_path import FinitePathCache,PathCommand
from test_single_boundary import CAM

P=np.c_[np.linspace(5.,6.5,20),np.zeros(20)]
def warmed(**kw):
 c=FinitePathCache(2.565,**kw);assert c.advance(1_000_000_000,.2,0.,1_000_000_000);c.observe(P,1_000_000_000,{'right':1});return c

def test_translation_and_anchor_age_no_prediction_recycling():
 c=warmed()
 for i in range(1,10):
  t=1_000_000_000+i*100_000_000;assert c.advance(t,.2,0.,t)
  p,e=c.predict(t,'FRAME_DROPPED',conflict=False,motion_error_bound_m=0.)
  np.testing.assert_allclose(p,P-[i*.02,0],atol=1e-10)
  assert e['cache_observation_stamp']==1_000_000_000
 assert np.array_equal(c.path,P)

def test_arc_motion_matches_analytic_transform():
 c=warmed();ang=.2;w=.2*np.tan(ang)/2.565;theta=w*.1
 c.advance(1_100_000_000,.2,ang,1_100_000_000)
 p,_=c.predict(1_100_000_000,'FRAME_DROPPED',conflict=False,motion_error_bound_m=0.)
 xy=np.array([.2/w*np.sin(theta),.2/w*(1-np.cos(theta))]);r=np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
 np.testing.assert_allclose(p,(P-xy)@r,atol=1e-10)

@pytest.mark.parametrize('mode',['stale','timegap','reverse','nan','duplicate'])
def test_bad_feedback_invalidates_cache(mode):
 c=warmed();t=1_100_000_000;speed=.2;f=t
 if mode=='stale':f=1_000_000_000-1
 if mode=='timegap':t=f=1_200_000_000
 if mode=='reverse':speed=-.1
 if mode=='nan':speed=float('nan')
 if mode=='duplicate':t=f=1_000_000_000
 assert not c.advance(t,speed,0,f)
 assert c.path is None

def test_conflict_expires_cache_and_normal_feedback_cannot_rearm():
 c=warmed();t=1_100_000_000;c.advance(t,.2,0,t)
 with pytest.raises(ValueError,match='CONFLICT'):c.predict(t,'AMBIGUOUS_CORRIDOR',conflict=False,motion_error_bound_m=0.)
 t+=100_000_000;c.advance(t,.2,0,t)
 with pytest.raises(ValueError):c.predict(t,'FRAME_DROPPED',conflict=False,motion_error_bound_m=0.)

@pytest.mark.parametrize('kw', [{'max_age_s':.15},{'max_travel_m':.025}])
def test_age_or_distance_never_extended(kw):
 c=warmed(**kw)
 for t in [1_100_000_000,1_200_000_000]:c.advance(t,.2,0,t)
 with pytest.raises(ValueError,match='EXPIRED'):c.predict(t,'FRAME_DROPPED',conflict=False,motion_error_bound_m=0.)

def test_unknown_uncertainty_and_seven_meter_limit():
 c=warmed();t=1_100_000_000;c.advance(t,.2,0,t)
 with pytest.raises(ValueError,match='UNCERTAIN'):c.predict(t,'FRAME_DROPPED',conflict=False,motion_error_bound_m=float('nan'))
 with pytest.raises(ValueError,match='7M'):c.observe(P+[3,0],t,{'right':1})

def test_prediction_command_keeps_pp_formula_and_caps_speed():
 from test_single_boundary import CONFIG
 cfg=dict(CONFIG);cfg['allow_single_boundary']=False
 f=PathCommand(cfg);p=P+[0,.5]
 o=f.compute(p,1_000_000_000,CAM,{'observation_mode':'predicted_cached_path'})
 assert o['valid'] and o['speed']<=.1
 x,y=o['target_rear_m'];assert o['target_steering']==pytest.approx(np.arctan(2*cfg['wheelbase']*y/(x*x+y*y)))
 assert not f.compute(P+[3,0],1_030_000_000,CAM,{})['valid']


def test_extended_default_keeps_age_and_distance_bounded():
 c=warmed()
 for i in range(1,16):
  t=1_000_000_000+i*100_000_000;c.advance(t,.1,0.,t)
  p,e=c.predict(t,'FRAME_DROPPED',conflict=False,motion_error_bound_m=0.)
 assert e['cache_age_s']==1.5 and e['cache_travel_m']<.3
 t+=100_000_000;c.advance(t,.1,0.,t)
 with pytest.raises(ValueError,match='EXPIRED'):c.predict(t,'FRAME_DROPPED',conflict=False,motion_error_bound_m=0.)
 c=warmed()
 for i in range(1,5):
  t=1_000_000_000+i*100_000_000;c.advance(t,1.,0.,t)
 with pytest.raises(ValueError,match='EXPIRED'):c.predict(t,'FRAME_DROPPED',conflict=False,motion_error_bound_m=0.)
