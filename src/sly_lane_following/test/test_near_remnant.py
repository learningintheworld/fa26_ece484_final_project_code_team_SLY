import cv2
import numpy as np
from sly_lane_following.core import LaneDetector

def fixture():
 d=LaneDetector({'fragment_identity_handoff':True});d.last_stamp=1_033_000_000;d.roles={12:'right'}
 y=np.linspace(442,476,30);x=316+(y-442)*2.7
 d.memory={12:(dict(id=12,height=480,y_min=442,y_max=476,knots=np.c_[x,y].tolist(),score=1.),1_000_000_000)}
 mask=np.zeros((480,768),np.uint8);p=np.array([[334,448],[405,474]],np.int32);cv2.polylines(mask,[p],False,255,5)
 return d,mask

def test_current_near_remnant_does_not_need_a_far_fragment_link():
 d,m=fixture();out=d.recover_tracked_mask([],m)
 assert len(out)==1 and out[0]['mask_anchor_id']==12 and out[0]['tracked_mask_recovery']
 for x,y in out[0]['knots']:assert m[round(y),round(x)]

def test_near_recovery_never_acquires_unknown_or_competing_identity():
 d,m=fixture();d.roles={};assert not d.recover_tracked_mask([],m)
 d,m=fixture();d.roles[13]='left';d.memory[13]=d.memory[12];assert not d.recover_tracked_mask([],m)
 d,m=fixture();d.last_stamp+=200_000_000;assert not d.recover_tracked_mask([],m)
 d,m=fixture();d.cfg['fragment_identity_handoff']=False;assert not d.recover_tracked_mask([],m)
