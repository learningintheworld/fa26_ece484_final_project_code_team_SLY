"""Early preference must not bypass width, ambiguity or target support guards."""
import copy
import numpy as np
import pytest
from test_width_history import QUALITY,warm
from test_single_boundary import CAM,straight,detection,single

EARLY={**QUALITY,'early_reliable_side':True}

def short_left(offset=0.):
    x=np.linspace(6.3,11.,60);z=np.linspace(4.,11.,60)
    r=detection(np.c_[x,x*0+1.6+offset],np.c_[z,z*0-1.6])
    r['pairs']=[dict(ids=[1,2])]
    return r

def activate():
    s=warm(EARLY);r=short_left();outs=[]
    for i in range(3):outs.append(s.compute(r,(600,800),round((2.+.1*i)*1e9),CAM))
    return s,outs

def test_preference_needs_three_frames_and_preserves_id_and_width_age():
    s,outs=activate()
    assert all(x['valid'] for x in outs),outs
    assert not outs[0].get('early_side_selected') and not outs[1].get('early_side_selected')
    assert outs[2]['early_side_selected']=='right'
    assert outs[2]['observation_mode']=='single_right'
    assert s.corridor.ids=={'left':1,'right':2}
    assert outs[2]['width_updated'] is False
    assert outs[2]['width_age_s']>0
    assert np.max(abs(np.asarray(outs[2]['metric_path'])[:,1]))<.001

def test_disabled_policy_has_no_early_selection():
    s=warm(QUALITY)
    for i in range(6):
        o=s.compute(short_left(),(600,800),round((2.+.1*i)*1e9),CAM)
        assert o['valid'] and not o.get('early_side_selected')

@pytest.mark.parametrize('kind',['missing','ambiguity','wrong_width'])
def test_preferred_side_does_not_hide_faults_or_auto_rearm(kind):
    s,_=activate();r=short_left()
    if kind=='missing':r.update(candidates=[],selected=[],vision_valid=False,reason='NO_SUPPORTED_EGO_PAIR')
    if kind=='ambiguity':
        from test_nearfield import ambiguous
        r=ambiguous();x=np.linspace(4.,10.,80)
        alternate=detection(np.c_[x,x*0+2.4],np.c_[x,x*0-.8])
        alternate['candidates'][0]['id']=4;alternate['candidates'][1]['id']=3
        r['candidates']=r['candidates'][:2]+alternate['candidates']
        r['pairs']=[dict(ids=[1,2]),dict(ids=[4,3])]
    if kind=='wrong_width':r=short_left(.8)
    o=s.compute(r,(600,800),int(2.3e9),CAM)
    assert not o['valid'],o
    assert not s.compute(short_left(),(600,800),int(2.4e9),CAM)['valid']

def test_time_gap_clears_side_votes_and_width():
    s,_=activate();o=s.compute(short_left(),(600,800),int(3e9),CAM)
    assert o['reason']=='CONTROL_TIME_DISCONTINUITY'
    assert s.corridor.side_vote is None and not s.corridor.widths


def test_unsupported_single_target_still_stops():
    s,_=activate();x=np.linspace(6.5,11.,60)
    r=single(detection(np.c_[x,x*0+1.6],np.c_[x,x*0-1.6]),'right')
    o=s.compute(r,(600,800),int(2.3e9),CAM)
    assert not o['valid'] and o['reason']=='NO_RELIABLE_SINGLE_BOUNDARY'
    assert s.cfg['lookahead_m']==4.5
    assert 'single_target_clamp_m' not in s.cfg  # Original default0.5 remains.
