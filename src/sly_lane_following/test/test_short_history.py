import numpy as np
import pytest
from sly_lane_following.short_history import RawRightCache,rigid_fit,robust_motion,align_right

def add(c,stamp,identity=1,source='raw'):
    c.add(stamp,identity,np.zeros((40,40),np.uint8),np.ones((40,40),np.uint8),
          np.c_[np.arange(5.),np.zeros(5)],source=source)

def test_original_only_readonly_and_hard_expiry():
    c=RawRightCache();add(c,0)
    assert c.get(500_000_000,1,.5) is not None
    assert c.get(500_000_001,1,.5) is None
    add(c,600_000_000)
    with pytest.raises(ValueError):c.items[-1].ground[0]=7
    with pytest.raises(ValueError,match='PREDICTION'):add(c,700_000_000,source='aligned')
    with pytest.raises(ValueError,match='AGE_LIMIT'):c.get(700_000_000,1,.6)

def test_restart_identity_and_missing_current_clear_history():
    c=RawRightCache();add(c,100);add(c,50);assert len(c.items)==1
    add(c,70,2);assert len(c.items)==1
    assert c.get(80,None,.1) is None and not c.items
    add(c,90);assert c.get(100,1,.1,ambiguous=True) is None and not c.items

def test_direct_rigid_motion_recovers_ground_points():
    rng=np.random.default_rng(7);a=rng.uniform([5,-3],[14,3],(80,2));angle=.04
    r=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]]);b=a@r.T+[-.25,.1]
    rr,t,models,stats=robust_motion(a,b)
    assert np.max(abs(a@rr.T+t-b))<1e-10
    assert stats['heldout_p95_m']<1e-10 and len(models)==12

def test_degenerate_or_inconsistent_motion_rejected():
    a=np.c_[np.linspace(5,10,40),np.zeros(40)]
    with pytest.raises(ValueError,match='SPATIAL'):robust_motion(a,a)
    with pytest.raises(ValueError,match='TOO_FEW'):robust_motion(a[:5],a[:5])

def test_wrong_identity_ambiguity_or_no_features_never_aligns():
    c=RawRightCache();add(c,0);o=c.items[0]
    for identity,amb in [(None,False),(34,False),(1,True)]:
        with pytest.raises(ValueError,match='CURRENT_RIGHT'):align_right(o,100_000_000,o.gray,o.motion_mask,None,None,identity,ambiguous=amb)
    with pytest.raises(ValueError,match='TOO_FEW_IMAGE'):align_right(o,100_000_000,o.gray,o.motion_mask,None,None,1)
