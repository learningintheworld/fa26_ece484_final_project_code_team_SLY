import copy
import cv2
import numpy as np
from sly_lane_following import physical_identity as module
from sly_lane_following.physical_identity import connect_observed,PhysicalStripeTracker,estimate_motion,association_score


def scene():
 im=np.full((150,300),30,np.uint8);cv2.line(im,(30,80),(260,80),220,5)
 a=dict(id=34,points=np.c_[np.linspace(30,120,16),np.full(16,80.)].tolist())
 b=dict(id=43,points=np.c_[np.linspace(175,260,16),np.full(16,80.)].tolist())
 return im,[a,b]


def test_current_paint_connects_without_mutating_input():
 im,c=scene();before=copy.deepcopy(c);out=connect_observed(im,c)
 assert len(out)==1 and set(out[0]['source_ids'])=={34,43}
 assert out[0]['joins'][0]['source']=='current_raw_white_cross_sections'
 assert c==before


def test_real_paint_gap_is_not_filled():
 im,c=scene();im[:,143:151]=30
 assert len(connect_observed(im,c))==2


def test_independently_confirmed_left_and_right_cannot_merge():
 im,c=scene();assert len(connect_observed(im,c,{34:'left',43:'right'}))==2


def test_branch_defers_instead_of_choosing_nearest():
 im,c=scene();d=copy.deepcopy(c[1]);d['id']=44;d['points']=(np.array(d['points'])+[0,1]).tolist();c.append(d)
 assert all(len(g['source_ids'])==1 for g in connect_observed(im,c))


def test_crossing_geometry_cannot_pass_parallel_association():
 x=np.linspace(1,4,20);a=np.c_[x,np.zeros(20)];b=np.c_[np.full(20,2.5),np.linspace(-1,1,20)]
 assert association_score(a,b) is None


class Projector:
 def project(self,p,camera):
  p=np.array(p,float)/30;return p,np.ones(len(p),bool)


def test_unknown_identity_does_not_invent_role(monkeypatch):
 monkeypatch.setattr(module,'estimate_motion',lambda *args:dict(valid=True,rotation=np.eye(2).tolist(),translation=[0,0]))
 im,c=scene();tracker=PhysicalStripeTracker(Projector())
 r=tracker.update(im,c,{},1_000_000_000,excluded_ids=[34]);identity=r['groups'][0]['physical_id']
 r=tracker.update(im,[c[1]],{},1_030_000_000)
 assert r['groups'][0]['physical_id']==identity
 assert r['groups'][0]['role'] is None and r['groups'][0]['needs_role_reconfirmation']


def test_known_side_transfers_with_observation_not_numeric_id(monkeypatch):
 monkeypatch.setattr(module,'estimate_motion',lambda *args:dict(valid=True,rotation=np.eye(2).tolist(),translation=[0,0]))
 im,c=scene();tracker=PhysicalStripeTracker(Projector())
 r=tracker.update(im,[c[1]],{},1_000_000_000,confirmed_roles={43:'right'})
 old_id=r['groups'][0]['physical_id'];c[1]['id']=99
 r=tracker.update(im,[c[1]],{},1_030_000_000)
 assert r['groups'][0]['physical_id']==old_id and r['groups'][0]['role']=='right'


def test_annotation_preserves_existing_center_and_commands():
 im,c=scene();result=dict(candidates=c,selected=[],vision_valid=False,center=[],command={'speed':0.})
 before=copy.deepcopy(result);r=PhysicalStripeTracker(Projector()).annotate(result,im,{},1000000000)
 assert result==before
 assert {k:v for k,v in r.items() if k!='physical_identity'}==before


def test_bad_time_or_no_texture_cannot_create_motion():
 image=np.zeros((480,768),np.uint8)
 for dt in [-.1,0,.26]:assert not estimate_motion(image,image,Projector(),{},dt)['valid']
 assert not estimate_motion(image,image,Projector(),{},.1)['valid']


def test_observed_join_can_split_without_forgetting_physical_identity(monkeypatch):
 monkeypatch.setattr(module,'estimate_motion',lambda *args:dict(valid=True,rotation=np.eye(2).tolist(),translation=[0,0]))
 im,c=scene();tracker=PhysicalStripeTracker(Projector())
 first=tracker.update(im,c,{},1_000_000_000,excluded_ids=[34])
 identity=first['groups'][0]['physical_id']
 im[:,143:151]=30
 second=tracker.update(im,c,{},1_030_000_000)
 assert len(second['groups'])==2
 assert all(g['physical_id']==identity and not g['joins'] for g in second['groups'])
 assert all(g['role'] is None and g['needs_role_reconfirmation'] for g in second['groups'])


def test_new_dual_geometry_can_reconfirm_but_single_hint_cannot(monkeypatch):
 monkeypatch.setattr(module,'estimate_motion',lambda *args:dict(valid=True,rotation=np.eye(2).tolist(),translation=[0,0]))
 im,c=scene();c=[c[1]];tracker=PhysicalStripeTracker(Projector())
 tracker.update(im,c,{},1_000_000_000,excluded_ids=[43])
 for i in range(3):
  r=tracker.update(im,c,{},int((1.03+i*.03)*1e9),confirmed_roles={43:'right'})
  assert r['groups'][0]['role'] is None
 other=copy.deepcopy(c[0]);other['id']=99;other['points']=(np.array(other['points'])+[0,-40]).tolist()
 for i in range(3):
  r=tracker.update(im,c+[other],{},int((1.15+i*.03)*1e9),confirmed_roles={43:'right',99:'left'},verified_dual_ids=[43,99])
 assert next(g for g in r['groups'] if 43 in g['source_ids'])['role']=='right'


def test_same_physical_stripe_cannot_reconfirm_both_lane_sides(monkeypatch):
 monkeypatch.setattr(module,'estimate_motion',lambda *args:dict(valid=True,rotation=np.eye(2).tolist(),translation=[0,0]))
 im,c=scene();tracker=PhysicalStripeTracker(Projector())
 tracker.update(im,c,{},1_000_000_000,excluded_ids=[34])
 for i in range(4):
  r=tracker.update(im,c,{},int((1.03+i*.03)*1e9),confirmed_roles={34:'left',43:'right'},verified_dual_ids=[34,43])
  assert all(g['role'] is None for g in r['groups'])
