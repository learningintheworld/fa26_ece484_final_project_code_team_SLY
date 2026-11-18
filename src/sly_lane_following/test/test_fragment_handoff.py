import copy
import numpy as np
from sly_lane_following.core import LaneDetector

def fragment(identity,lo,hi,slope=.9,offset=180):
 y=np.linspace(lo,hi,16);return dict(id=identity,y_min=lo,y_max=hi,height=480,score=.95,knots=np.c_[slope*y+offset,y].tolist(),residual_px=0.)

def tracker():
 d=LaneDetector({'fragment_identity_handoff':True});d.roles={1:'right'};return d

def train(d,extra=()):
 for i in range(3):d.fragment_identity_handoff([fragment(1,414,476),fragment(10,276,332),*copy.deepcopy(extra)],int((1+i*.033)*1e9))

def test_handoff_preserves_current_pixels_and_role():
 d=tracker();train(d);b=fragment(10,277,335);points=copy.deepcopy(b['knots']);d.fragment_identity_handoff([b],int(1.1e9))
 assert b['id']==1 and b['fragment_handoff_from_id']==10
 assert b['knots']==points and b['y_min']==277
 assert d.roles=={1:'right'}

def test_no_handoff_without_repeated_fresh_unique_observations():
 for mode in ['once','stale','competing']:
  d=tracker()
  if mode=='once':d.fragment_identity_handoff([fragment(1,414,476),fragment(10,276,332)],int(1e9))
  else:train(d,[fragment(11,276,332,offset=185)] if mode=='competing' else [])
  b=fragment(10,277,335);d.fragment_identity_handoff([b],int((1.5 if mode=='stale' else 1.1)*1e9));assert b['id']==10

def test_neighbor_and_direction_conflicts_do_not_form_link():
 for b in [fragment(10,276,332,offset=240),fragment(10,276,332,slope=-.9,offset=800)]:
  d=tracker()
  for i in range(3):d.fragment_identity_handoff([fragment(1,414,476),copy.deepcopy(b)],int((1+i*.033)*1e9))
  assert not d.fragment_links

def test_default_and_highbay_behavior_is_unchanged():
 d=LaneDetector();d.roles={1:'right'};train(d);b=fragment(10,277,335);d.fragment_identity_handoff([b],int(1.1e9));assert b['id']==10

def test_verified_link_survives_one_gap_edge_without_renewing_evidence():
 d=tracker();train(d);seen=d.fragment_links[1]['stamp']
 d.fragment_identity_handoff([fragment(1,433,476),fragment(10,276,332)],int(1.1e9))
 assert d.fragment_links[1]['stamp']==seen
 b=fragment(10,277,335);d.fragment_identity_handoff([b],int(1.13e9));assert b['id']==1
 d=tracker();train(d)
 d.fragment_identity_handoff([fragment(1,433,476),fragment(10,276,332)],int(1.1e9))
 b=fragment(10,277,335);d.fragment_identity_handoff([b],int(1.4e9));assert b['id']==10
