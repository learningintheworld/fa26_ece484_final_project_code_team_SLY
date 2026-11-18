"""Returning fragments must not reset identity or waive a near contradiction."""
import numpy as np
import pytest
from test_width_history import warm,pair_with_new_left
from test_single_boundary import CAM,straight,detection

def test_rejected_geometry_is_not_a_camera_time_gap():
    s=warm();ids=s.corridor.ids.copy();r=pair_with_new_left(.8)
    for t in np.arange(2.,2.9,.1):
        # Repeated explicit offline retry; a live latch remains tested below.
        s.corridor.latched=False
        out=s.compute(r,(600,800),round(t*1e9),CAM)
        assert not out['valid'] and out['reason']!='CONTROL_TIME_DISCONTINUITY'
        assert s.corridor.ids==ids
    assert s.corridor.latched
    assert not s.compute(straight(),(600,800),int(3e9),CAM)['valid']
    out=s.compute(straight(),(600,800),int(4e9),CAM)
    assert out['reason']=='CONTROL_TIME_DISCONTINUITY'
    assert not s.corridor.ids

def test_short_new_side_defers_identity_without_blocking_current_anchor():
    s=warm();x=np.linspace(5.,5.7,24);z=np.linspace(4.,11.,60)
    r=detection(np.c_[x,x*0+1.6],np.c_[z,z*0-1.6])
    r['candidates'][0]['id']=9;r.update(selected=[9,2],pairs=[dict(ids=[9,2])])
    o=s.compute(r,(600,800),int(2e9),CAM)
    assert o['valid'] and o['observation_mode']=='single_right'
    assert s.corridor.ids=={'left':1,'right':2}
    assert not s.corridor.pending_count

def test_short_interior_line_still_contradicts_single_corridor():
    s=warm();x=np.linspace(4.,5.,24);z=np.linspace(4.,11.,60)
    r=detection(np.c_[x,x*0+.4],np.c_[z,z*0-1.6])
    r['candidates'][0]['id']=9;r.update(selected=[9,2],pairs=[dict(ids=[9,2])])
    o=s.compute(r,(600,800),int(2e9),CAM)
    assert not o['valid'] and o['reason']=='SINGLE_CORRIDOR_CONTRADICTION'
import cv2
from sly_lane_following.core import LaneDetector

def recovery_fixture(enabled=True):
    d=LaneDetector({'fragment_identity_handoff':enabled});d.roles={1:'right'};d.last_stamp=int(1.03e9)
    y=np.arange(90,140);x=90+.004*(y-110)**2
    old=dict(id=1,height=160,knots=np.c_[x,y].tolist(),y_min=90,y_max=139,score=1.)
    d.memory={1:(old,int(1e9))};d.fragment_links={1:{'count':3}};mask=np.zeros((160,200),np.uint8)
    cv2.polylines(mask,[np.rint(np.c_[x,y]).astype('int32')],False,255,5)
    return d,mask,old

def test_tracked_recovery_uses_current_mask_and_retains_normal_candidates():
    d,mask,old=recovery_fixture();out=d.recover_tracked_mask([],mask)
    assert len(out)==1 and out[0]['tracked_mask_recovery']
    assert np.ptp(np.array(out[0]['knots'])[:,1])>=45
    assert d.recover_tracked_mask([old],mask)==[old]
    # Default/simulator behavior, stale seed, blank image never recover.
    d.cfg['fragment_identity_handoff']=False;assert d.recover_tracked_mask([],mask)==[]
    d.cfg['fragment_identity_handoff']=True;d.last_stamp=int(2e9);assert d.recover_tracked_mask([],mask)==[]
    d.last_stamp=int(1.03e9);assert d.recover_tracked_mask([],mask*0)==[]

def test_recovery_does_not_bridge_a_mask_gap_or_guess_competing_seed():
    d,mask,old=recovery_fixture();mask[104:114]=0
    out=d.recover_tracked_mask([],mask)
    for c in out:
        y=np.array(c['knots'])[:,1]
        assert not (y.min()<104 and y.max()>113)
    d,mask,old=recovery_fixture();d.roles[2]='left';d.memory[2]=(dict(old,id=2),int(1e9))
    assert d.recover_tracked_mask([],mask)==[]

@pytest.mark.parametrize('image_pair',[True,False])
def test_return_confirmation_survives_far_center_rejection(monkeypatch,image_pair):
    from sly_lane_following.geometry import CorridorGeometry
    from test_width_history import QUALITY
    s=warm({**QUALITY,'single_observed_target_max_m':7.})
    original=CorridorGeometry.dual
    def reject_far_center(self,result,*args):
        if result.get('selected')==[9,2]:raise ValueError('CENTER_DISAGREEMENT')
        return original(self,result,*args)
    monkeypatch.setattr(CorridorGeometry,'dual',reject_far_center)
    x=np.linspace(7.,11.,50);z=np.linspace(5.5,11.,60)
    r=detection(np.c_[x,x*0+1.6],np.c_[z,z*0-1.6])
    r['candidates'][0]['id']=9;r.update(selected=[9,2],pairs=[dict(ids=[9,2])])
    history=s.corridor.widths.copy()
    if not image_pair:r.update(vision_valid=False,selected=[],pairs=[],reason='NO_SUPPORTED_EGO_PAIR')
    for i in range(4):
        o=s.compute(r,(600,800),round((2.+i*.1)*1e9),CAM)
        assert o['valid'] and o['observation_mode']=='single_right'
    assert s.corridor.ids=={'left':9,'right':2}
    assert s.corridor.widths==history and not s.corridor.latched


def test_returning_pair_can_be_verified_without_image_row_pair():
    s=warm();r=pair_with_new_left()
    r.update(vision_valid=False,selected=[],pairs=[],reason='NO_SUPPORTED_EGO_PAIR')
    for i in range(4):
        o=s.compute(r,(600,800),round((2.+i*.1)*1e9),CAM)
        assert o['valid']
    assert s.corridor.ids=={'left':9,'right':2}
    # A second equally plausible returning boundary cannot be chosen.
    import copy
    s=warm();b=copy.deepcopy(r['candidates'][0]);b['id']=10;r['candidates'].append(b)
    assert s.compute(r,(600,800),int(2e9),CAM)['reason']=='AMBIGUOUS_CORRIDOR'
