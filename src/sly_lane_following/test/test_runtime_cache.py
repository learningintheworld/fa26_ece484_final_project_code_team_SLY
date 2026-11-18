import numpy as np
import pytest
from sly_lane_following.runtime_cache import RuntimePathCache

class Corridor:
    def __init__(self):self.reset()
    def reset(self):self.latched=False;self.widths=[3.3]*8;self.ids={'left':1,'right':2};self.last_stamp=None
    def compute(self,result,stamp,camera):
        if self.latched:raise ValueError('CORRIDOR_REARM_REQUIRED')
        if result.get('fail'):
            self.latched=True;raise ValueError(result['fail'])
        return np.c_[np.linspace(3.,6.,30),np.zeros(30)],dict(observation_mode='dual')
    def boundary(self,points,camera):return np.asarray(points)

def make():return RuntimePathCache(Corridor(),dict(wheelbase=1.75,lookahead_m=3.5))
def feed(c,t,v=.1,w=0.):c.feedback_sample(int(t*1e9),v,w)
def step(c,t,fail=None,**kwargs):
    feed(c,t);return c.compute(dict(fail=fail,**kwargs),int(t*1e9),{})

def test_dropout_recovers_without_refreshing_cache_age():
    c=make();step(c,1.)
    for i in range(1,31):
        t=1+i*.04;p,e=step(c,t,'NO_RELIABLE_SINGLE_BOUNDARY')
        assert e['cache_observation_stamp']==1000000000
        assert abs(p[0,0]-(3-.1*(t-1)))<1e-8
    assert not c.current.latched
    p,e=step(c,2.24);assert e['cache_ready']
    assert c.cache.observation_stamp==2240000000

def test_expiry_and_recovery_do_not_rearm():
    c=make();step(c,1.)
    for i in range(1,38):step(c,1+i*.04,'NO_RELIABLE_SINGLE_BOUNDARY')
    with pytest.raises(ValueError,match='CACHE_EXPIRED'):step(c,2.52,'NO_RELIABLE_SINGLE_BOUNDARY')
    with pytest.raises(ValueError,match='REARM'):step(c,2.56)

def test_stale_feedback_and_real_ambiguity_stop():
    c=make();step(c,1.)
    with pytest.raises(ValueError,match='MOTION_STALE'):c.compute(dict(fail='NO_RELIABLE_SINGLE_BOUNDARY'),1200000000,{})
    c=make();step(c,1.)
    with pytest.raises(ValueError,match='AMBIGUOUS'):step(c,1.04,'AMBIGUOUS_CORRIDOR')
    with pytest.raises(ValueError,match='REARM'):step(c,1.08)

def test_current_near_conflict_stops():
    c=make();step(c,1.)
    # Right side offset predicts centre 0.6 m LEFT of the cached centre.
    candidate=dict(id=2,score=1.,points=np.c_[np.linspace(3,6,30),np.full(30,-1.05)].tolist())
    with pytest.raises(ValueError,match='CACHE_NEAR_CONFLICT'):
        step(c,1.04,'NO_RELIABLE_SINGLE_BOUNDARY',candidates=[candidate])

def test_measured_turn_rotates_cache_not_requested_steering():
    c=make();step(c,1.)
    for i in range(1,6):feed(c,1+i*.02,v=.2,w=.1)
    p,e=c.compute(dict(fail='NO_RELIABLE_SINGLE_BOUNDARY'),1100000000,{})
    assert abs(e['cache_pose'][2]-.01)<1e-8
    assert p[0,1]<0

def test_initialization_is_not_a_driving_fault():
    c=make()
    for i in range(3):
        with pytest.raises(ValueError,match='INITIALIZING'):
            step(c,1+i*.04,'INITIALIZING_METRIC_WIDTH')
        c.current.latched=False  # fake corridor raises with latch; real init does not
        assert not c.hard_stop
    p,e=step(c,1.12);assert e['cache_ready']

def test_new_paint_inside_cached_near_corridor_is_not_ignored():
    c=make();step(c,1.)
    candidate=dict(id=99,score=1.,points=np.c_[np.linspace(3,6,30),np.zeros(30)].tolist())
    with pytest.raises(ValueError,match='CACHE_NEAR_CONFLICT'):
        step(c,1.04,'NO_RELIABLE_SINGLE_BOUNDARY',candidates=[candidate])


def test_junction_identity_uses_distinct_curve_not_shared_tip():
    from sly_lane_following.core import LaneDetector
    old={'knots': [[297.5, 415.0], [403.5, 433.0], [463.5, 451.0], [509.0, 469.0], [546.5, 487.0], [578.0, 505.0], [607.0, 523.0], [634.0, 541.0], [658.5, 559.0], [694.5, 583.0], [707.0, 595.0]], 'y_min': 415.0, 'y_max': 595.0, 'height': 600, 'coef': [-2969.3307695974936, 6197.574552972393, -2530.730508476612], 'score': 0.967741935483871}
    branches=[{'knots': [[464.0, 349.0], [481.5, 367.0], [500.0, 385.0], [518.0, 403.0], [536.5, 421.0], [554.0, 439.0], [572.0, 457.0], [590.5, 475.0], [608.5, 493.0], [626.5, 511.0], [644.5, 529.0], [677.0, 565.0], [707.0, 595.0]], 'y_min': 349.0, 'y_max': 595.0, 'height': 600, 'coef': [-66.92291273608576, 700.4178889201103, 78.04432216523631], 'score': 0.8809523809523809}, {'knots': [[291.5, 415.0], [401.0, 433.0], [461.5, 451.0], [507.0, 469.0], [544.5, 487.0], [576.5, 505.0], [605.5, 523.0], [633.0, 541.0], [657.5, 559.0], [665.0, 565.0]], 'y_min': 415.0, 'y_max': 565.0, 'height': 600, 'coef': [-4188.034188034189, 8159.2934472934385, -3316.4483855650456], 'score': 1.0}]
    d=LaneDetector(dict(association_distinct_support=True))
    d.memory={8:(old,451738000000)};d.roles={8:'right'};d.previous_pair=(13,8)
    d.associate(branches,stamp=451803000000)
    kept=next(c for c in branches if c['id']==8)
    assert kept['y_min']==415. and kept['knots'][0][0]<350
    assert kept['distinct_identity_support']==8
