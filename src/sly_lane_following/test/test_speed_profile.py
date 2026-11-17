import numpy as np
from sly_lane_following.speed_profile import SmoothSpeed,preview_speed


def test_acceleration_and_jerk_limits_for_stop_go_and_speed_changes():
    s=SmoothSpeed();speeds=[];acc=[];jerks=[]
    for target in [1.,.1,.35,1.,0.]:
        for _ in range(500):
            v,a,j=s.step(target,.05);speeds.append(v);acc.append(a);jerks.append(j)
    assert min(speeds)>=0 and max(speeds)<=1.000001
    assert max(acc)<=.200001 and min(acc)>=-.250001
    assert max(abs(np.array(jerks)))<=.300001
    assert speeds[-1]<.001


def test_current_preview_reduces_speed_before_bend_and_single_stays_slow():
    x=np.linspace(4.,12.,81);straight=np.c_[x,x*0]
    fast,_,_=preview_speed(straight,0.,.2,'dual');assert fast==1.
    curve=np.c_[x,.12*np.maximum(x-5.,0.)**2]
    slow,_,_=preview_speed(curve,0.,.2,'dual');assert .1<=slow<fast
    assert preview_speed(straight,0.,.1,'single_left')[0]==.1
    assert preview_speed(straight,0.,.1,'mixed_near_left')[0]==.1


def test_single_distance_accounts_for_faster_transitions_and_stale_feedback():
    from sly_lane_following.speed_profile import single_speed_bound
    assert single_speed_bound(None,0,1000000000)==1.
    assert single_speed_bound((1000000000,.8),970000000,1030000000)>.8
    assert single_speed_bound((1000000000,.1),970000000,1030000000)==.2
    assert single_speed_bound((0,.1),970000000,1030000000)==1.
