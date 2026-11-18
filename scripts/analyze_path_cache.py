"""Offline continuous-RGB experiment, hypothetical motion, no ROS publishers."""
import copy,importlib.util,sys,json,sqlite3,math,hashlib,os
from pathlib import Path
import cv2,numpy as np,yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sly_lane_following.core import LaneDetector,camera_maps
from sly_lane_following.geometry import VisualPurePursuit
from sly_lane_following.finite_path import FinitePathCache,PathCommand
R=Path(__file__).resolve().parents[1];O=R/'results/path_cache_v1';sys.path.insert(0,str(R/'scripts'))
from analyze_real_stages import PreviewCorridor
spec=importlib.util.spec_from_file_location('sly_lane_following._before_core',O/'before_core.py');base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
rows=list(map(json.loads,(R/'results/transition_v1/final/frames.jsonl').open()));cam=json.loads((R/'results/stage2/reference/camera.json').read_text());cfg=yaml.safe_load((R/'src/sly_lane_following/config/e4_pure_pursuit.yaml').read_text());vc=yaml.safe_load((R/'src/sly_lane_following/config/real_enet.yaml').read_text());vc['enet_checkpoint']=str(R/vc['enet_checkpoint']);neural=LaneDetector(vc);maps,anchor=camera_maps(cam,(480,768,3));neural.maps=maps;neural.anchor=anchor
bag=Path('/data/gem/oak_front_image');meta=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information'];db=sqlite3.connect((bag/meta['relative_file_paths'][0]).resolve().as_uri()+'?mode=ro',uri=True);iid=db.execute("select id from topics where name='/oak/rgb/image_raw'").fetchone()[0];start=db.execute('select min(timestamp) from messages').fetchone()[0];bridge=CvBridge()
def raw_for(row):
 target=start+round(row['time_s']*1e9);blob=db.execute('select data from messages where topic_id=? and timestamp between ? and ? order by abs(timestamp-?) limit 1',(iid,target-1000,target+1000,target)).fetchone()[0];msg=deserialize_message(blob,Image);assert msg.header.stamp.sec*10**9+msg.header.stamp.nanosec==row['stamp_ns'];return bridge.imgmsg_to_cv2(msg,'bgr8')
def seed(cls,lo):
 prev=next(r for r in reversed(rows) if r['time_s']<lo);valid=next(r for r in reversed(rows) if r['time_s']<lo and r['control']['valid']);hist=[r for r in rows if lo-1<=r['time_s']<lo];d=cls({**vc,'mask_backend':'opencv'});d.maps=maps;d.anchor=anchor;d.map_key=(cam['width'],cam['height'],tuple(cam['K']),tuple(cam['D']),cam['distortion_model']);d.roles={i:s for s,i in prev['prediction']['boundary_ids'].items()};d.previous_pair=tuple(prev['prediction']['boundary_ids'][s] for s in ['left','right']);d.previous=copy.deepcopy(prev['candidates']);d.last_stamp=d.role_seen_stamp=prev['stamp_ns'];d.last_valid_stamp=valid['stamp_ns'];d.next_id=max(c['id'] for r in rows if r['time_s']<lo for c in r['candidates'])+1;d.gray_previous=cv2.cvtColor(cv2.remap(raw_for(prev),*maps,cv2.INTER_LINEAR),cv2.COLOR_BGR2GRAY)
 for r in hist:
  cs=copy.deepcopy(r['candidates']);d.fragment_identity_handoff(cs,r['stamp_ns'])
  for c in r['candidates']:
   d.memory[c['id']]=(copy.deepcopy(c),r['stamp_ns']);d.observation_history[c['id']]=(d.observation_history.get(c['id'],[])+[(copy.deepcopy(c),r['stamp_ns'])])[-5:]
 s=VisualPurePursuit(cfg);s.corridor=PreviewCorridor(cfg,s.projector);q=s.corridor;q.ids=prev['prediction']['boundary_ids'].copy();o=valid['control'];q.widths=[o['lane_width_m']]*cfg['width_init_frames'];q.last_dual=valid['stamp_ns']-round(o.get('width_age_s',0)*1e9);q.width_radius=max(.1,o.get('width_radius_m',.1)-cfg['width_drift_mps']*o.get('width_age_s',0));q.input_stamp=q.last_stamp=prev['stamp_ns'];s.last_stamp=prev['stamp_ns'];s.steering=prev['control']['steering_angle']
 for i,(c,t) in d.memory.items():
  if i in q.ids.values():q.observed[i]=(t,np.asarray(c['points']))
 return d,s

def run_control(d,s,raw,mask,stamp):
 d.mask=lambda image:mask
 r=d.process(raw,cam,stamp);q=s.corridor;q.latched=False
 for c in r['candidates']:
  if c['id'] in q.ids.values() and 'fragment_reference_points' in c:q.observed[c['id']]=(c['fragment_reference_stamp_ns'],np.asarray(c['fragment_reference_points']))
 out=s.compute(r,raw.shape,stamp,cam)
 for c in r['candidates']:
  if c['id'] in q.ids.values():q.observed[c['id']]=(stamp,np.asarray(c['points']))
 return r,out

class Trial:
 def __init__(self):self.cache=FinitePathCache(cfg['wheelbase']);self.follower=PathCommand(cfg);self.last_angle=0.;self.last_speed=.15
 def step(self,o,stamp,ids,forced=False):
  # Counterfactual perfect actuator: feedback equals the PREVIOUS request.
  # Recorded images do not change in response; this is not closed loop.
  self.cache.advance(stamp,self.last_speed,self.last_angle,stamp)
  fresh=o['valid'] and not forced and o.get('lookahead_actual_m',100)<=7.
  if fresh:
   self.cache.observe(o['metric_path'],stamp,ids);p=np.asarray(o['metric_path']);extra={k:v for k,v in o.items() if k not in ['metric_path','valid','speed','steering_angle','reason']};extra['observation_mode']=o.get('observation_mode','observed');mode='observed'
  else:
   why='FRAME_DROPPED' if forced else ('TARGET_OUTSIDE_7M' if o['valid'] else o['reason'])
   try:p,extra=self.cache.predict(stamp,why,conflict=why not in self.cache.SUPPORT_FAILURES,motion_error_bound_m=0.);mode='predicted'
   except ValueError as e:
    self.follower.servo.reset();self.last_speed=0.;return dict(valid=False,speed=0.,steering_angle=0.,reason=str(e),source_failure=why,mode='stopped')
  out=self.follower.compute(p,stamp,cam,extra);out['mode']=mode
  if out['valid']:self.last_angle=out['steering_angle'];self.last_speed=out['speed']
  out['motion_source']='hypothetical_feedback_equals_previous_requests';out['not_closed_loop']=True
  return out

def panel(raw,mask,r,old,new,cache,t):
 a=raw.copy();a[mask>0]=(.55*a[mask>0]+.45*np.array([65,255,65])).astype('uint8');ids=r['ids']
 for c in r['candidates']:
  col=(0,155,255) if c['id']==ids.get('right') else (255,160,0) if c['id']==ids.get('left') else (150,150,150);p=np.rint(c['points']).astype('int32');cv2.polylines(a,[p],False,col,2);cv2.putText(a,str(c['id']),tuple(np.clip(p[len(p)//2],[0,20],[730,460])),0,.5,col,1)
 b=raw.copy();cim=raw.copy()
 def draw(im,out,col):
  # Ground inset is essential because the inferred center can project below RGB FOV.
  x0,y0=480,25;im[y0:y0+210,x0:x0+275]=240
  def xy(p):return (int(x0+135-p[1]*27),int(y0+195-p[0]*14))
  cv2.line(im,xy([0,0]),xy([12,0]),(180,180,180),1)
  for x in [4,7,10]:cv2.putText(im,f'{x}m',(x0+140,xy([x,0])[1]),0,.4,(80,80,80),1)
  if out['valid']:
   path=np.asarray(out['metric_path']);pix=np.rint(neural_projector.image(path,cam)).astype('int32');cv2.polylines(im,[pix],False,col,2);cv2.polylines(im,[np.array([xy(p) for p in path])],False,col,2);cv2.circle(im,xy(out['target_rear_m']),5,(0,0,255),-1)
  else:cv2.putText(im,'NO PATH',(x0+55,y0+95),0,.6,(0,0,180),2)
 draw(b,old,(255,0,255));draw(cim,cache,(255,200,0) if cache.get('mode')=='predicted' else (255,0,255))
 panels=[raw,a,b,cim];titles=['RGB','v3 + NEW boundaries','BEFORE center','NEW + bounded cache']
 for im,title in zip(panels,titles):cv2.putText(im,title,(10,25),0,.6,(0,0,255),2)
 out=np.vstack([np.hstack(panels[:2]),np.hstack(panels[2:])]);out=cv2.copyMakeBorder(out,65,0,0,0,cv2.BORDER_CONSTANT,value=(250,250,250));cv2.putText(out,f'{t:.3f}s  RECORDED RGB + HYPOTHETICAL MOTION (not closed loop)',(12,25),0,.65,(0,0,0),1);cv2.putText(out,f"new={new['reason']} | cache={cache['reason']} | v={cache['speed']:.3f}",(12,52),0,.6,(0,0,0),1);return out

from sly_lane_following.geometry import GroundProjector
neural_projector=GroundProjector(cfg);allresults=[];summaries={}
reuse={x['stamp_ns']:x for x in map(json.loads,(O/'frames.jsonl').open())} if os.environ.get('SLY_CACHE_REUSE')=='1' else {}
for name,lo,hi in [('65',64.9,66.7),('108',108.0037,109.7)]:
 before,sb=seed(base.LaneDetector,lo);after,sa=seed(LaneDetector,lo);natural=Trial();drop=Trial();results=[];vid=cv2.VideoWriter(str(O/f'window_{name}.avi'),cv2.VideoWriter_fourcc(*'MJPG'),30,(1536,1025));selected=[65.437,65.468,65.803,66.238,66.269] if name=='65' else [108.502,108.602,108.702,108.935,109.382,109.401]
 local=[r for r in rows if lo<=r['time_s']<=hi];nearest={min(local,key=lambda r:abs(r['time_s']-t))['stamp_ns'] for t in selected}
 for row in local:
  t,stamp=row['time_s'],row['stamp_ns'];raw=raw_for(row);und=cv2.remap(raw,*maps,cv2.INTER_LINEAR)
  if stamp in reuse:
   rr=reuse[stamp];ob,oa=rr['before'],rr['after'];ra=dict(candidates=rr['candidates'],reason=rr['recognition_reason']);sa.corridor.ids=rr['ids'];mask=neural.mask(und)
  else:
   mask=neural.mask(und);rb,ob=run_control(before,sb,raw,mask,stamp);ra,oa=run_control(after,sa,raw,mask,stamp)
  c=natural.step(oa,stamp,sa.corridor.ids);forced=(65.03<=t<=65.23) if name=='65' else (108.15<=t<=108.35);dc=drop.step(oa,stamp,sa.corridor.ids,forced)
  result=dict(time_s=t,stamp_ns=stamp,before=ob,after=oa,cache=c,dropout=dc,forced_drop=forced,candidates=ra['candidates'],ids=sa.corridor.ids.copy(),recognition_reason=ra['reason']);results.append(result)
  im=panel(und,mask,result,ob,oa,c,t);vid.write(im)
  if stamp in nearest:cv2.imwrite(str(O/f'{t:.3f}_comparison.jpg'),im)
 vid.release();allresults.extend(results)
 from collections import Counter
 summaries[name]=dict(frames=len(results),before_valid=sum(r['before']['valid'] for r in results),after_valid=sum(r['after']['valid'] for r in results),cache_valid=sum(r['cache']['valid'] for r in results),cache_predicted=sum(r['cache'].get('mode')=='predicted' for r in results),before_reasons=dict(Counter(r['before']['reason'] for r in results)),after_reasons=dict(Counter(r['after']['reason'] for r in results)),cache_reasons=dict(Counter(r['cache']['reason'] for r in results)),forced_frames=sum(r['forced_drop'] for r in results),forced_continued=sum(r['forced_drop'] and r['dropout']['valid'] for r in results));print(name,json.dumps(summaries[name]),flush=True)
(O/'frames.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in allresults));(O/'summary.json').write_text(json.dumps(summaries,indent=2))
