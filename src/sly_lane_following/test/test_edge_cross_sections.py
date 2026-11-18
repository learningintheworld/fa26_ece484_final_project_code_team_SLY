import cv2
import numpy as np
from sly_lane_following.core import LaneDetector,curve_x

CONFIG=dict(column_scan=True,fit_mode='piecewise',merge_observed_overlap=True,
            split_unsupported_connections=True,roi_top=.55)

def test_bottom_clipped_column_uses_complete_row():
    m=np.zeros((480,768),np.uint8)
    cv2.line(m,(540,300),(756,480),255,11)
    cs=LaneDetector(CONFIG).candidates(m)
    c=max(cs,key=lambda c:c['y_max'])
    # Complete row cuts remain available right up to the scan's last row.
    assert c['y_max']>=473
    y=c['y_max'];expected=540+1.2*(y-300)
    assert abs(float(curve_x(c,[y])[0])-expected)<2

def test_right_clipped_row_uses_complete_column_or_truncates():
    m=np.zeros((480,768),np.uint8)
    cv2.line(m,(600,320),(820,480),255,11)
    cs=LaneDetector(CONFIG).candidates(m)
    c=max(cs,key=lambda c:c['y_max']);y=c['y_max']
    assert y<440  # No residual row at the image's right edge is a centre.
    assert abs(float(curve_x(c,[y])[0])-(600+1.375*(y-320)))<2

def test_no_complete_cross_section_cannot_generate_boundary():
    m=np.zeros((480,768),np.uint8);m[:,760:]=255
    assert LaneDetector(CONFIG).candidates(m)==[]
    assert LaneDetector(CONFIG).candidates(np.zeros_like(m))==[]

def test_uncropped_stripe_is_unchanged_and_neighbour_stays_separate():
    m=np.zeros((480,768),np.uint8)
    for x in [450,600]:cv2.line(m,(x,300),(x+60,470),255,7)
    cs=LaneDetector(CONFIG).candidates(m)
    assert len(cs)==2
    assert abs(float(curve_x(cs[0],[420])[0]-curve_x(cs[1],[420])[0]))>140

def test_row_only_profile_retains_existing_edge_behaviour():
    m=np.zeros((480,768),np.uint8);m[:,760:]=255
    assert LaneDetector({'fit_mode':'piecewise'}).candidates(m)
