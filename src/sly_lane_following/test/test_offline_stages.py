"""Offline preview does not change the driving gate or waive current evidence."""
import importlib.util
from pathlib import Path
import numpy as np
from sly_lane_following.geometry import VisualPurePursuit
from test_width_history import QUALITY,warm
from test_single_boundary import straight,single,CAM
spec=importlib.util.spec_from_file_location('offline_stages',Path(__file__).resolve().parents[3]/'scripts/analyze_real_stages.py')
mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)

def preview():
 s=VisualPurePursuit(QUALITY);s.corridor=mod.PreviewCorridor(QUALITY,s.projector)
 for i in range(10):o=s.compute(straight(),(600,800),round((1+i*.1)*1e9),CAM)
 assert o['valid'];return s

def test_preview_age_is_flagged_but_vehicle_gate_still_stops():
 a=preview();b=warm();r=single(straight(),'right')
 for t in np.arange(2,24,.1):
  x=a.compute(r,(600,800),round(t*1e9),CAM);y=b.compute(r,(600,800),round(t*1e9),CAM)
 assert x['valid'] and x['temporal_budget_exceeded'] and x['offline_only']
 assert not y['valid']

def test_good_current_dual_does_not_age_out():
 s=warm()
 for t in np.arange(2,24,.1):o=s.compute(straight(),(600,800),round(t*1e9),CAM)
 assert o['valid'] and o['width_updated']

def test_preview_preserves_current_quality_and_ambiguity_checks():
 for fault in ['noisy','missing','ambiguous']:
  r=single(straight(),'right')
  if fault=='noisy':r['candidates'][0]['residual_px']=20.
  if fault=='missing':r['candidates']=[]
  if fault=='ambiguous':r['reason']='AMBIGUOUS_CORRIDOR'
  o=preview().compute(r,(600,800),int(2e9),CAM)
  assert not o['valid']
