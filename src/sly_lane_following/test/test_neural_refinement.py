"""RGB refinement may remove neural pixels, never invent a missed stripe."""
import cv2
import numpy as np
from sly_lane_following.core import LaneDetector, camera_maps
from test_core import paint_frame


def detector(image, prediction, refine):
    h,w=image.shape[:2]
    camera=dict(width=w,height=h,K=[450.,0.,w/2,0.,450.,h/2,0.,0.,1.],D=[0.]*5,distortion_model='plumb_bob')
    d=LaneDetector(dict(enet_refine_paint=refine))
    d.maps,_=camera_maps(camera,image.shape)
    d.neural_mask=lambda _: prediction.copy()
    return d


def test_refinement_removes_background_without_adding_neural_pixels():
    image=paint_frame(neighbor_brightness=0);pred=np.full(image.shape[:2],255,np.uint8)
    pred[:,image.shape[1]//2:]=0
    d=detector(image,pred,True);refined=d.mask(image)
    assert np.count_nonzero(refined)>0
    assert np.count_nonzero(refined)<np.count_nonzero(pred)
    assert not np.any(refined[pred==0])
    gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    assert not np.any(refined[gray<d.cfg['gray_min']])


def test_neural_miss_and_no_paint_both_remain_empty():
    image=paint_frame(neighbor_brightness=0)
    assert not detector(image,np.zeros(image.shape[:2],np.uint8),True).mask(image).any()
    blank=np.full_like(image,35)
    assert not detector(blank,np.full(image.shape[:2],255,np.uint8),True).mask(blank).any()


def test_unselected_real_profile_keeps_original_neural_mask():
    image=paint_frame(neighbor_brightness=0);pred=np.zeros(image.shape[:2],np.uint8);pred[350:370,310:330]=255
    assert np.array_equal(detector(image,pred,False).mask(image),pred)


def test_dense_support_confirms_only_continuously_visible_curve():
    from copy import deepcopy
    mask=np.zeros((240,320),np.uint8)
    cv2.line(mask,(80,210),(240,80),255,3)
    candidate=dict(score=.6,y_min=80.,y_max=210.,knots=[[240.,80.],[80.,210.]])
    complete=deepcopy(candidate)
    LaneDetector.confirm_mask_support([complete],mask)
    assert complete['score'] >= .98
    assert complete['knots'] == candidate['knots']
    mask[135:152]=0
    broken=deepcopy(candidate)
    LaneDetector.confirm_mask_support([broken],mask)
    assert broken['score'] == .6
    assert broken['mask_unsupported_run_px'] > 3


def test_dense_support_cannot_borrow_neighbouring_paint():
    mask=np.zeros((240,320),np.uint8)
    cv2.line(mask,(120,210),(280,80),255,3)
    candidate=dict(score=.6,y_min=80.,y_max=210.,knots=[[240.,80.],[80.,210.]])
    LaneDetector.confirm_mask_support([candidate],mask)
    assert candidate['score'] == .6


def test_far_crossing_is_only_a_hypothesis_for_metric_confirmation():
    from test_core import boundary
    d=LaneDetector(dict(defer_far_pair_crossing=True))
    d.previous_pair=(1,2);d.roles={1:'left',2:'right'}
    left=boundary(1,180,380);right=boundary(2,580,370)
    chosen,reason,pairs,rejected=d.select([left,right],768,480,384)
    assert chosen is None and reason=='AMBIGUOUS_CORRIDOR'
    assert pairs==[dict(ids=[1,2],deferred_far_crossing=True)]
    assert rejected[0]['reason']=='CROSSING_OR_THIN_PAIR'
    assert d.previous_pair==(1,2)
    d.reset()
    assert not d.select([left,right],768,480,384)[2]


def test_near_crossing_and_unknown_identities_still_rejected():
    from test_core import boundary
    d=LaneDetector(dict(defer_far_pair_crossing=True))
    d.previous_pair=(1,2);d.roles={1:'left',2:'right'}
    assert not d.select([boundary(1,380,380),boundary(2,390,370)],768,480,384)[2]
    assert not d.select([boundary(3,180,380),boundary(4,580,370)],768,480,384)[2]


def test_narrow_paint_keeps_interior_without_joining_different_stripes():
    image=np.full((480,768,3),35,np.uint8)
    image[300:478,290:310]=145
    image[300:478,325:345]=145
    pred=np.zeros(image.shape[:2],np.uint8);pred[300:478,285:350]=255
    mask=detector(image,pred,True).mask(image)
    assert mask[400,300] and mask[400,335]
    assert not mask[400,310:325].any()
    assert not mask[pred==0].any()


def test_short_optical_flow_overlap_does_not_hide_direct_observation(monkeypatch):
    from test_core import boundary
    import copy
    def run(enabled,shift=0.):
        d=LaneDetector(dict(association_observed_overlap=enabled))
        near=boundary(6,500,440,y_min=535,y_max=595)
        far=boundary(7,464,338,y_min=433,y_max=559)
        d.roles={6:'right'};d.previous_pair=(1,6)
        d.memory={6:(near,1000000000),7:(far,1000000000)};d.next_id=8
        d.gray_previous=np.zeros((480,768),np.uint8)
        calls=[0]
        def flow(a,b,points,*args,**kwargs):
            calls[0]+=1;out=points.copy();out[:,:,1]+=1 if calls[0]%2 else -1
            return out,np.ones((len(points),1),np.uint8),np.zeros((len(points),1),np.float32)
        monkeypatch.setattr(cv2,'calcOpticalFlowPyrLK',flow)
        current=boundary(0,470+shift,338+shift,y_min=433,y_max=565);current.pop('id')
        d.associate([current],np.zeros((480,768),np.uint8),1033333333)
        return current['id']
    assert run(False)==7
    assert run(True)==6
    assert run(True,shift=60.)!=6


def test_recent_observation_window_recovers_clipped_fragment_but_expires():
    from test_core import boundary
    def run(age):
        d=LaneDetector(dict(association_observed_overlap=True))
        full=boundary(6,500,338,y_min=433,y_max=595)
        short=boundary(6,500,446,y_min=541,y_max=595)
        far=boundary(7,464,338,y_min=433,y_max=559)
        d.roles={6:'right'};d.previous_pair=(1,6)
        d.memory={6:(short,1000000000),7:(far,1000000000)};d.next_id=8
        d.observation_history={6:[(full,1033333333-int(age*1e9))]}
        current=boundary(0,470,338,y_min=433,y_max=565);current.pop('id')
        d.associate([current],stamp=1033333333)
        return current
    assert run(.067)['id']==6
    assert run(.067)['association_history_age_s']<.25
    assert run(.3)['id']==7


def test_recent_identity_window_never_supplies_missing_observation():
    from test_core import boundary
    d=LaneDetector(dict(association_observed_overlap=True));old=boundary(6,500)
    d.previous_pair=(1,6);d.roles={6:'right'};d.memory={6:(old,1000000000)}
    d.observation_history={6:[(old,1000000000)]}
    cs=[];d.associate(cs,stamp=1033333333)
    assert cs==[]
    d.reset();assert d.observation_history=={}


def test_row_priority_uses_recent_near_identity_not_an_unrelated_branch():
    d=LaneDetector(dict(preserve_row_identity=True))
    old=dict(knots=[[449.,349.],[695.,595.]],y_min=349.,y_max=595.)
    d.previous_pair=(1,2);d.last_stamp=1033333333
    d.observation_history={2:[(old,1000000000)]}
    current=[(695.-i*6,595.-i*6) for i in range(10)]
    neighbour=[(742.-i*9,595.-i*6) for i in range(10)]
    assert d.row_identity_priority(current)==0
    assert d.row_identity_priority(neighbour)==1
    # No current near evidence, stale geometry, and two equally plausible
    # roles must not get a history-based priority.
    assert d.row_identity_priority(current[:3])==1
    d.last_stamp=1400000000
    assert d.row_identity_priority(current)==1
    d.last_stamp=1033333333;d.observation_history[1]=[(old,1000000000)]
    assert d.row_identity_priority(current)==1
    d.reset()
    assert not d.candidates(np.zeros((600,800),np.uint8))


def test_row_priority_disabled_profile_keeps_equal_competition():
    d=LaneDetector();d.last_stamp=1033333333;d.previous_pair=(1,2)
    d.memory={2:(dict(knots=[[449.,349.],[695.,595.]],y_min=349.,y_max=595.),1000000000)}
    assert d.row_identity_priority([(695.-i*6,595.-i*6) for i in range(10)])==1
