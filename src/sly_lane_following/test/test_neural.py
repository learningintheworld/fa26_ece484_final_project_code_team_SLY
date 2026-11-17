"""Geometry-preserving preprocessing and optional backend isolation."""
import sys
import cv2
import numpy as np
import pytest
from sly_lane_following.neural import letterbox, restore_probability
from sly_lane_following.core import LaneDetector


def test_letterbox_keeps_aspect_and_ignore_padding():
    image=np.zeros((480,768,3),np.uint8);label=np.zeros((480,768),np.uint8)
    label[:,380:388]=1
    x,y,m=letterbox(image,label)
    assert x.shape==(384,640) and m==(13,0,614,384,768,480)
    assert (y[:,:13]==255).all() and (y[:,627:]==255).all()
    restored=restore_probability((y==1).astype(float),m)
    center=np.flatnonzero(restored[240]>.5).mean()
    assert abs(center-383.5)<2


def test_coordinates_round_trip_portrait():
    im=np.zeros((640,320,3),np.uint8)
    _,_,m=letterbox(im)
    p=np.zeros((384,640),np.float32);x,y,w,h,_,_=m
    p[y:y+h,x:x+w]=1
    assert np.allclose(restore_probability(p,m),1)


def test_neural_mask_still_excludes_top_and_invalid_remap():
    d=LaneDetector(); yy,xx=np.mgrid[:480,:768]
    d.maps=(xx.astype(np.float32),yy.astype(np.float32))
    d.neural_mask=lambda im:np.full(im.shape[:2],255,np.uint8)
    m=d.mask(np.zeros((480,768,3),np.uint8))
    assert not m[:259].any() and not m[:,:2].any()
    assert m[350,350]==255 and m.shape==(480,768)


def test_unknown_backend_rejected():
    with pytest.raises(ValueError,match='UNKNOWN_MASK_BACKEND'):
        LaneDetector({'mask_backend':'typo'})


def test_default_never_loads_neural_model():
    assert LaneDetector().neural_mask is None


def test_checkpoint_roi_enforced_without_trimming_input_or_changing_geometry():
    from sly_lane_following.neural import NeuralMask
    m=object.__new__(NeuralMask);m.inference_roi_top=275/480
    seen=[]
    def full_probability(im):
        seen.append(im.shape)
        return np.ones(im.shape[:2],np.float32)
    m.probability=full_probability
    out=m(np.zeros((480,768,3),np.uint8))
    assert seen==[(480,768,3)] and not out[:275].any() and (out[275:]==255).all()
    scaled=m(np.zeros((240,384,3),np.uint8))
    assert not scaled[:138].any() and (scaled[138:]==255).all()


def test_legacy_checkpoint_without_roi_keeps_original_mask():
    from sly_lane_following.neural import NeuralMask
    m=object.__new__(NeuralMask);m.inference_roi_top=0.
    m.probability=lambda im:np.full(im.shape[:2],.75,np.float32)
    assert (m(np.zeros((480,768,3),np.uint8))==255).all()
