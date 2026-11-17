"""Current near observations may resolve far branches, never a near fork."""
import copy
import numpy as np
from sly_lane_following.core import LaneDetector
from sly_lane_following.geometry import VisualPurePursuit
from test_control import boundary
from test_single_boundary import CONFIG,CAM,straight,warm,detection


def ambiguous(fork_x=6.,near_offset=0.):
    x=np.linspace(4,10,80)
    result=detection(np.c_[x,np.full(len(x),1.6)],np.c_[x,np.full(len(x),-1.6)])
    other=detection(np.c_[x,np.full(len(x),1.6)],np.c_[x,-1.6-near_offset-.35*np.maximum(0,x-fork_x)**2])['candidates'][1]
    other['id']=3;result['candidates'].append(other)
    result.update(vision_valid=False,selected=[],reason='AMBIGUOUS_CORRIDOR',pairs=[dict(ids=[1,2]),dict(ids=[1,3])])
    return result


def test_far_branch_is_deferred_only_on_current_supported_prefix():
    servo=warm(straight());before=copy.deepcopy(servo.corridor.widths)
    out=servo.compute(ambiguous(),(600,800),int(1.3e9),CAM)
    assert out['valid'],out
    assert out['observation_mode']=='mixed_near_confirmed'
    assert servo.corridor.widths==before
    assert np.max(abs(np.array(out['metric_path'])[:,1]))<.001
    assert out['reliable_forward_end_m']<7.3
    assert abs(out['lookahead_actual_m']-CONFIG['lookahead_m'])<.01


def test_wrong_width_branch_is_filtered_against_known_corridor():
    for result in [ambiguous(fork_x=3.),ambiguous(near_offset=.8)]:
        out=warm(straight()).compute(result,(600,800),int(1.3e9),CAM)
        assert out['valid'] and abs(out['target_rear_m'][1])<.001


def test_near_confirmation_requires_known_shared_boundary_and_history():
    result=ambiguous();result['pairs'][1]['ids'][0]=99
    assert not warm(straight()).compute(result,(600,800),int(1.3e9),CAM)['valid']
    assert not VisualPurePursuit(CONFIG).compute(ambiguous(),(600,800),int(1.3e9),CAM)['valid']


def test_near_boundary_owns_identity_when_far_shape_changes():
    d=LaneDetector();d.previous_pair=(1,2);d.roles={1:'left',2:'right',3:'right'}
    def curve(identity,points):
        c=boundary(identity,500,y_min=300,y_max=595);c.update(knots=points,y_min=points[0][1],y_max=points[-1][1]);return c
    near=curve(2,[[300,300],[420,500],[500,559],[530,595]])
    far=curve(3,[[360,300],[420,500],[480,547]])
    d.memory={2:(near,10**9),3:(far,10**9)};d.next_id=4
    full=curve(0,[[360,300],[420,500],[490,547],[500,559],[530,595]])
    short=curve(0,[[300,300],[420,500],[480,547]])
    for c in [full,short]:c.pop('id')
    d.associate([full,short],stamp=1_033_333_333)
    assert full['id']==2
    assert short['id']!=2


def test_near_id_does_not_attach_to_distant_parallel_stripe():
    d=LaneDetector();old=boundary(2,500,y_min=350,y_max=475)
    d.previous_pair=(1,2);d.roles={2:'right'};d.memory={2:(old,10**9)};d.next_id=3
    other=boundary(0,550,y_min=350,y_max=475);other.pop('id')
    d.associate([other],stamp=1_033_333_333)
    assert other['id']!=2


def test_crossing_overlap_fragments_are_not_averaged_together():
    d=LaneDetector()
    a=boundary(1,300,280,y_min=300,y_max=400)
    b=boundary(2,320,296,y_min=380,y_max=500)
    b.update(knots=[[296,380],[330,390],[302,400],[320,500]])
    assert len(d.join_fragments([a,b]))==2


def test_far_tail_of_selected_identity_is_checked_before_rejecting_width():
    servo=warm(straight());result=ambiguous(fork_x=5.0)
    result.update(vision_valid=True,selected=[1,3],reason='VISION_VALID_PIXELS_ONLY')
    before=copy.deepcopy(servo.corridor.widths)
    out=servo.compute(result,(600,800),int(1.3e9),CAM)
    assert out['valid'],out
    assert out['observation_mode']=='mixed_near_confirmed'
    assert servo.corridor.widths==before


def test_simultaneous_far_fragments_preserve_known_near_pair():
    result=ambiguous()
    far=copy.deepcopy(result['candidates'][0]);far['id']=4
    far['points']=far['points'][:45]
    result['candidates'].append(far)
    result['pairs'].append(dict(ids=[4,2]))
    out=warm(straight()).compute(result,(600,800),int(1.3e9),CAM)
    assert out['valid'],out
    assert out['observation_mode']=='mixed_near_confirmed'


def test_single_pair_far_tail_cannot_corrupt_known_near_width():
    servo=warm(straight());result=ambiguous(fork_x=5.)
    # Keep the known right identity while its far tail takes a different branch.
    result['candidates'][1]['points']=result['candidates'][2]['points']
    result.update(vision_valid=True,selected=[1,2],reason='VISION_VALID_PIXELS_ONLY',pairs=[dict(ids=[1,2])])
    out=servo.compute(result,(600,800),int(1.3e9),CAM)
    assert out['valid'] and out['observation_mode']=='mixed_near_confirmed',out
    result['candidates'][1]['points']=ambiguous(near_offset=.8)['candidates'][2]['points']
    assert not warm(straight()).compute(result,(600,800),int(1.3e9),CAM)['valid']


def test_pending_fragment_switch_needs_near_agreement_with_known_pair():
    result=ambiguous()
    result.update(reason='REACQUIRING_CORRIDOR',pairs=[dict(ids=[1,3])],
                  rejected_pairs=[dict(ids=[1,2],reason='SPANS_ANOTHER_BOUNDARY')])
    out=warm(straight()).compute(result,(600,800),int(1.3e9),CAM)
    assert out['valid'] and out['observation_mode']=='mixed_near_confirmed',out
    result['candidates'][2]['points']=ambiguous(near_offset=.8)['candidates'][2]['points']
    assert warm(straight()).compute(result,(600,800),int(1.3e9),CAM)['valid']


def test_two_plausible_corridors_at_target_are_not_resolved_by_identity():
    result=ambiguous()
    x=np.linspace(4.,10.,80)
    alternative=detection(np.c_[x,np.full(len(x),2.4)],np.c_[x,np.full(len(x),-.8)])
    alternative['candidates'][0]['id']=4;alternative['candidates'][1]['id']=3
    result['candidates']=result['candidates'][:2]+alternative['candidates']
    result['pairs']=[dict(ids=[1,2]),dict(ids=[4,3])]
    out=warm(straight()).compute(result,(600,800),int(1.3e9),CAM)
    assert not out['valid'] and out['speed']==0


def test_unpaired_short_remnant_keeps_original_single_side_fallback():
    servo=warm(straight());result=straight()
    x=np.linspace(5.2,6.,30)
    short=detection(np.c_[x,x*0+1.6],np.c_[x,x*0-1.6])['candidates'][0]
    result['candidates'][0]=short
    new=copy.deepcopy(short);new['id']=3;result['candidates'].append(new)
    result.update(vision_valid=False,reason='REACQUIRING_CORRIDOR',selected=[],pairs=[dict(ids=[3,2])])
    out=servo.compute(result,(600,800),int(1.3e9),CAM)
    assert out['valid'] and out['observation_mode']=='single_right',out
