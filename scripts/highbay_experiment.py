"""Highbay experiments only. Geometry is never imported by the driving node.
Run through dev.sh exec. Requires existing sim and a disarmed simulation adapter.
"""
import argparse, ast, hashlib, json, math, os, shutil, signal, struct, subprocess, time
from pathlib import Path
import xml.etree.ElementTree as ET
import cv2
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
from ackermann_msgs.msg import AckermannDriveStamped
from std_srvs.srv import SetBool

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/highbay_control'
SIM=Path('/workspace/gem_simulator')
PKG=ROOT/'src/sly_lane_following'

def wrap(x):return math.atan2(math.sin(x),math.cos(x))
def save(path,value):path.write_text(json.dumps(value,indent=2,allow_nan=False))

def origin(element):
    m=np.eye(4)
    if element is None:return m
    xyz=np.fromstring(element.get('xyz','0 0 0'),sep=' ')
    r,p,y=np.fromstring(element.get('rpy','0 0 0'),sep=' ')
    cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    m[:3,:3]=[[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]
    m[:3,3]=xyz;return m

def stl_vertices(path):
    blob=path.read_bytes()
    if len(blob)>=84 and 84+50*struct.unpack_from('<I',blob,80)[0]==len(blob):
        dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')])
        return np.frombuffer(blob,offset=84,dtype=dtype)['vertices'].reshape(-1,3).astype(float)
    return np.array([[float(v) for v in s.split()[1:]] for s in blob.decode().splitlines() if s.strip().startswith('vertex')])

def footprint():
    if not (OUT/'model.urdf').exists():
        result=subprocess.run(['bash','--noprofile','--norc','-c','source "$1"; xacro "$2"','sly-model',str(SIM/'install/local_setup.bash'),str(SIM/'gem_description/urdf/gem.urdf.xacro')],text=True,capture_output=True,check=True)
        (OUT/'model.urdf').write_text(result.stdout)
    model=ET.parse(OUT/'model.urdf').getroot();transforms={'base_footprint':np.eye(4)}
    joints=list(model.findall('joint'))
    for _ in range(len(joints)):
        for j in joints:
            parent=j.find('parent').get('link');child=j.find('child').get('link')
            if parent in transforms:transforms[child]=transforms[parent]@origin(j.find('origin'))
    points=[];details=[]
    for link in model.findall('link'):
        if link.get('name') not in transforms:continue
        for collision in link.findall('collision'):
            m=transforms[link.get('name')]@origin(collision.find('origin'));g=collision.find('geometry');mesh=g.find('mesh');sphere=g.find('sphere');box=g.find('box');cylinder=g.find('cylinder')
            if mesh is not None:
                v=stl_vertices(Path(mesh.get('filename').removeprefix('file://')))
                v*=np.fromstring(mesh.get('scale','1 1 1'),sep=' ')
            elif sphere is not None:
                # The official wheel collision shape is a sphere, not a point.
                radius=float(sphere.get('radius'));a=np.linspace(0,2*np.pi,96,endpoint=False)
                v=np.c_[radius*np.cos(a),radius*np.sin(a),np.zeros(len(a))]
            elif cylinder is not None:
                radius=float(cylinder.get('radius'));half=float(cylinder.get('length'))/2;a=np.linspace(0,2*np.pi,96,endpoint=False)
                v=np.vstack([np.c_[radius*np.cos(a),radius*np.sin(a),np.full(len(a),z)] for z in [-half,half]])
            elif box is not None:
                size=np.fromstring(box.get('size'),sep=' ')/2
                v=np.array([[x,y,z] for x in [-size[0],size[0]] for y in [-size[1],size[1]] for z in [-size[2],size[2]]])
            else:raise ValueError('Unsupported collision geometry: '+ET.tostring(g).decode())
            q=(np.c_[v,np.ones(len(v))]@m.T)[:,:2];points.append(q);details.append(link.get('name'))
    hull=cv2.convexHull(np.vstack(points).astype(np.float32)).reshape(-1,2)
    # Sample the conservative collision hull perimeter at <= 5 cm intervals.
    dense=np.vstack([np.linspace(a,b,max(2,int(np.ceil(np.linalg.norm(b-a)/.05))+1)) for a,b in zip(hull,np.roll(hull,-1,axis=0))])
    save(OUT/'footprint.json',dict(qualification='conservative convex hull of all official collision geometry; perimeter sampled at <= 0.05 m',links=details,hull=hull.tolist()))
    return dense

class Track:
    def __init__(self,route="combination"):
        self.route=route
        ns={'c':'http://www.collada.org/2005/11/COLLADASchema'};r=ET.parse(SIM/'gem_gazebo/models/highbay_track/meshes/highbay_track.dae');self.lines=[]
        for name in ['Lane_Left','Lane_Right']:
            pts=np.fromstring(r.find('.//c:geometry[@id="'+name+'-mesh"]//c:float_array',ns).text,sep=' ').reshape(-1,3)
            m=np.fromstring(r.find('.//c:visual_scene/c:node[@id="'+name+'"]/c:matrix',ns).text,sep=' ').reshape(4,4)
            self.lines.append(((np.c_[pts,np.ones(len(pts))]@m.T)[:,:2]).reshape(-1,4,2).mean(axis=1))
        self.center=(self.lines[0]+self.lines[1])/2;self.delta=np.roll(self.center,-1,axis=0)-self.center
        self.length=np.linalg.norm(self.delta,axis=1);self.s=np.r_[0,np.cumsum(self.length)];self.total=self.s[-1]
        self.start=self.project(-8,0)[0]
        self.goal_pose=[-8,0,0] if route=="full-lap" else [-11.6,-3.28084,math.pi]
        self.goal=self.project(*self.goal_pose[:2])[0]
        self.required=self.total if route=="full-lap" else (self.goal-self.start)%self.total
        self.reset_progress()
        polys=sorted([x.astype(np.float32) for x in self.lines],key=lambda p:abs(cv2.contourArea(p)),reverse=True)
        self.outer,self.inner=polys;self.body=footprint()
        save(OUT/'route.json',dict(start=[-8,0,0],goal=self.goal_pose,route=self.route,total_length_m=self.total,required_progress_m=self.required,center=self.center.tolist(),qualification='evaluation and terminal stop only; never supplied to controller'))
    def reset_progress(self):
        self.previous_s=self.start
        self.unwrapped_progress=0.
    def accumulate_progress(self,s):
        # Unwrap across the closed mesh seam and start line; never infer a
        # completed lap merely because the car is spatially near the start.
        delta=(s-self.previous_s+self.total/2)%self.total-self.total/2
        self.unwrapped_progress+=delta;self.previous_s=s
        return self.unwrapped_progress
    def reached_goal(self,state,metric):
        return (metric["progress_m"]>=self.required and
                abs(metric["heading_error_rad"])<math.radians(15) and
                (self.route!="full-lap" or math.hypot(state["x"]+8,state["y"])<.5))
    def project(self,x,y):
        q=np.array([x,y]);t=np.clip(np.einsum('ij,ij->i',q-self.center,self.delta)/np.maximum(self.length**2,1e-9),0,1)
        closest=self.center+t[:,None]*self.delta;idx=np.linalg.norm(q-closest,axis=1).argmin();d=self.delta[idx]
        h=math.atan2(d[1],d[0]);e=-math.sin(h)*(x-closest[idx,0])+math.cos(h)*(y-closest[idx,1])
        return float(self.s[idx]+t[idx]*self.length[idx]),float(e),h,closest[idx]
    def metrics(self,state):
        x,y,yaw=state['x'],state['y'],state['yaw'];s,e,h,_=self.project(x,y);rot=np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
        body=self.body@rot.T+[x,y]
        margin=min(min(cv2.pointPolygonTest(self.outer,(float(p[0]),float(p[1])),True),-cv2.pointPolygonTest(self.inner,(float(p[0]),float(p[1])),True)) for p in body)
        progress=(s-self.start)%self.total
        if self.route=="full-lap":progress=self.accumulate_progress(s)
        elif progress>self.total-1:progress-=self.total
        return dict(progress_m=float(progress),lateral_error_m=e,heading_error_rad=wrap(yaw-h),footprint_margin_m=float(margin))

class Observer(Node):
    def __init__(self):
        super().__init__('sly_highbay_experiment',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.bridge=CvBridge();self.raw_recent={};self.state={};self.control={};self.vision='WAIT';self.raw=None;self.info=None;self.raw_stamp=0.;self.debug=None;self.debug_stamp=0.;self.active=False;self.writer=None;self.frames=0;self.last_frame=-1.;self.start=0.;self.output=None;self.latencies=[]
        self.create_subscription(Odometry,'/odom',self.odom,10)
        self.create_subscription(Image,'/camera/image_raw',self.image,qos_profile_sensor_data)
        self.create_subscription(CameraInfo,'/camera/camera_info',self.camera,qos_profile_sensor_data)
        self.create_subscription(Image,'/sly/lane/debug_image',self.image_debug,5)
        self.create_subscription(DiagnosticArray,'/sly/control/status',self.status,10)
        self.create_subscription(DiagnosticArray,'/sly/lane/status',self.lane,10)
        self.create_subscription(AckermannDriveStamped,'/sly/control/request',self.request,10)
        from std_msgs.msg import Float64
        self.terminal_pub=self.create_publisher(Float64,'/sly/control/terminal_speed_limit',5)
        self.enable=self.create_client(SetBool,'/sly/control/enable')
    def now(self):return self.get_clock().now().nanoseconds/1e9
    def odom(self,m):
        p=m.pose.pose.position;q=m.pose.pose.orientation
        self.state.update(x=p.x,y=p.y,z=p.z,speed=m.twist.twist.linear.x,yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
    def image(self,m):
        self.raw=self.bridge.imgmsg_to_cv2(m,'bgr8');self.raw_stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        key=m.header.stamp.sec*1000000000+m.header.stamp.nanosec
        self.raw_recent[key]=self.raw
        while len(self.raw_recent)>45:self.raw_recent.pop(next(iter(self.raw_recent)))
    def camera(self,m):self.info=dict(width=m.width,height=m.height,K=list(m.k),D=list(m.d),distortion_model=m.distortion_model)
    def status(self,m):
        for s in m.status:
            if s.name=='sly/control':self.state.update(adapter_platform=s.hardware_id,adapter_reason=s.message,enabled=dict((v.key,v.value) for v in s.values).get('enabled')=='True')
            if s.name=='sly/control':
                values={v.key:v.value for v in s.values}
                self.state['command_speed']=float(values.get('speed',0))
                for key in ('command_acceleration','command_jerk'):
                    if key in values:self.state[key]=float(values[key])
    def lane(self,m):
        for s in m.status:
            if s.name=='sly/lane/vision':
                self.vision=s.message
                try:self.control=ast.literal_eval(dict((v.key,v.value) for v in s.values).get('control','{}'))
                except (ValueError,SyntaxError):self.control={}
                if self.active and self.control and not self.control.get('valid') and not (self.output/'fault_raw.jpg').exists():
                    key=m.header.stamp.sec*1000000000+m.header.stamp.nanosec
                    if key in self.raw_recent:
                        cv2.imwrite(str(self.output/'fault_raw.jpg'),self.raw_recent[key])
                        cv2.imwrite(str(self.output/'fault_raw.png'),self.raw_recent[key])
                        window=self.output/'fault_window';window.mkdir(exist_ok=True)
                        for stamp,frame in self.raw_recent.items():
                            if stamp<=key:cv2.imwrite(str(window/f'{stamp}.png'),frame)
                        (self.output/'fault_raw_stamp.json').write_text(json.dumps(dict(stamp_ns=key,reason=self.control.get('reason'))))
    def request(self,m):
        self.state.update(request_speed=m.drive.speed,steering=m.drive.steering_angle)
        if self.active:self.latencies.append(max(0.,self.now()-m.header.stamp.sec-m.header.stamp.nanosec/1e9))
    def image_debug(self,m):
        t=m.header.stamp.sec+m.header.stamp.nanosec/1e9;self.debug=self.bridge.imgmsg_to_cv2(m,'bgr8');self.debug_stamp=t
        if not self.active or t-self.last_frame<1/15-.002:return
        if self.writer is None:self.writer=cv2.VideoWriter(str(self.output/'closed_loop.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),15,(self.debug.shape[1],self.debug.shape[0]))
        image=self.debug.copy();cv2.putText(image,f'Highbay | elapsed {t-self.start:.2f}s | {self.state.get("adapter_reason","WAIT")}',(10,105),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
        self.writer.write(image);self.frames+=1;self.last_frame=t
    def pump(self,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.02)
    def set_enabled(self,flag):
        if not self.enable.wait_for_service(timeout_sec=5):raise RuntimeError('Simulation adapter missing')
        f=self.enable.call_async(SetBool.Request(data=flag));end=time.monotonic()+5
        while not f.done() and time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.02)
        if not f.done():raise RuntimeError('Enable service timeout')
        return f.result()
    def set_pose(self,x,y,yaw):
        if abs(self.state.get('speed',0))>.01:raise RuntimeError('Cannot teleport a moving simulator')
        req=f'name: "gem", position: {{x: {x}, y: {y}, z: 0.216}}, orientation: {{z: {math.sin(yaw/2)}, w: {math.cos(yaw/2)}}}'
        env=dict(os.environ,GZ_PARTITION='sly_highbay_m3')
        p=subprocess.run(['gz','service','-s','/world/highbay_testbed/set_pose','--reqtype','gz.msgs.Pose','--reptype','gz.msgs.Boolean','--timeout','3000','--req',req],env=env,text=True,capture_output=True,check=True)
        if 'true' not in p.stdout:raise RuntimeError(p.stdout+p.stderr)
        self.pump(.8)
        if math.hypot(self.state['x']-x,self.state['y']-y)>.03:raise RuntimeError('Simulator pose did not settle')


def diagnose(n,track):
    from sly_lane_following.core import LaneDetector
    directory=OUT/'visibility';directory.mkdir(exist_ok=False)
    old=ROOT/'results/milestone3';rows=[json.loads(s) for s in (old/'sim_trial_13/telemetry.jsonl').open()];trace=[json.loads(s) for s in (old/'sim_trace_13.jsonl').open()]
    first_loss=next(r for i,r in enumerate(trace) if i>20 and not r['vision_valid'] and trace[i-1]['vision_valid'])
    # The old telemetry lacks absolute ROS stamps. Align its first latched
    # stop to the first invalid image; uncertainty is one controller tick.
    epoch=first_loss['stamp_ns']/1e9-rows[-1]['elapsed_sim_s']
    edge=next(r for r in trace if r['stamp_ns']/1e9>=epoch and r.get('right') and max(p[0] for p in r['right'])>780)
    edge_time=edge['stamp_ns']/1e9-epoch
    selected=[('before_turn',min(rows,key=lambda r:abs(r['x']+.1))),
              ('edge_onset',min(rows,key=lambda r:abs(r['elapsed_sim_s']-edge_time))),
              ('before_stop',rows[-1])]
    save(directory/'pose_selection.json',dict(epoch_estimate=epoch,edge_elapsed_s=edge_time,alignment_uncertainty_s=.1,poses=selected))
    cfg=yaml.safe_load((PKG/'config/highbay.yaml').read_text());report=[]
    for stage,pose in selected:
        _,_,heading,center=track.project(pose['x'],pose['y'])
        cases=[('actual',pose['x'],pose['y'],pose['yaw'])]
        for lateral in [-.25,0.,.25]:
            for degrees in [-5,0,5]:cases.append((f'center_lat{lateral:+.2f}_yaw{degrees:+d}',center[0]-math.sin(heading)*lateral,center[1]+math.cos(heading)*lateral,heading+math.radians(degrees)))
        for label,x,y,yaw in cases:
            n.set_pose(x,y,yaw);raw=n.raw.copy();detector=LaneDetector(cfg)
            for i in range(10):result=detector.process(raw,n.info,1_000_000_000+i*33_333_333)
            stem=f'{stage}_{label}';cv2.imwrite(str(directory/(stem+'.jpg')),raw);cv2.imwrite(str(directory/(stem+'_overlay.jpg')),result['debug_image'])
            item=dict(stage=stage,case=label,requested_pose=[float(x),float(y),float(yaw)],actual_pose={k:n.state[k] for k in ('x','y','z','yaw')},camera=n.info,stamp=n.raw_stamp,vision_valid=result['vision_valid'],reason=result['reason'],candidates=[dict(y_min=c['y_min'],y_max=c['y_max'],points=c['points']) for c in result['candidates']],raw_image=stem+'.jpg')
            report.append(item);print(json.dumps({k:item[k] for k in ('stage','case','vision_valid','reason')}),flush=True)
    save(directory/'summary.json',report);n.set_pose(-8,0,0)


def trial(n,track,name,config,perception_config=PKG/'config/highbay.yaml'):
    output=OUT/name;output.mkdir(exist_ok=False);save(output/'config.json',config);(output/'control.yaml').write_text(yaml.safe_dump(config))
    source=output/'source';shutil.copytree(PKG,source,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    shutil.copy2(__file__,output/'highbay_experiment.py');n.set_enabled(False);n.pump(.5);n.set_pose(-8,0,0);track.reset_progress()
    perception=yaml.safe_load(perception_config.read_text());shutil.copy2(perception_config,output/'perception.yaml');save(output/'camera.json',n.info)
    import sys
    runner=[sys.executable,'-m','sly_lane_following.node'] if perception.get('mask_backend')=='enet' else ['ros2','run','sly_lane_following','lane_node']
    log=(output/'perception.log').open('w');command=runner+['--ros-args','-p','use_sim_time:=true','-p','config:='+str(perception_config),'-p','control_config:='+str(output/'control.yaml'),'-p','image_topic:=/camera/image_raw','-p','camera_info_topic:=/camera/camera_info','-p','reliable_input:=true','-p','trace_path:='+str(output/'trace.jsonl')]
    proc=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    n.state.pop('request_speed',None);n.control={};n.vision='WAIT';n.output=output;n.frames=0;n.writer=None;n.last_frame=-1.;n.latencies=[]
    if perception.get('mask_backend')=='enet':(output/'raw').mkdir()
    last_raw=-1.
    records=[];reason='NO_VALID_REQUEST';armed=False;seen_enabled=False;start_wall=time.monotonic();last_record=-1.;next_print=0.;max_progress=0.;last_progress_wall=start_wall
    timeout=2*track.required/(config['speed']/(1+config['turn_slowdown']*config['max_steering']))+30
    try:
        while time.monotonic()-start_wall<20:
            rclpy.spin_once(n,timeout_sec=.03)
            if n.control.get('valid') and n.state.get('request_speed',0)>0:break
        else:raise RuntimeError('No valid current image control before enable')
        # Discovery of the new request publisher can reach the observer
        # before it reaches the adapter. Wait while stationary before arming.
        n.pump(1.)
        reply=n.set_enabled(True)
        discovery_deadline=time.monotonic()+10
        # Retry only a rejected initial arm while stationary. Once enabled,
        # no fault recovery/rearm is allowed anywhere in the driving loop.
        while not reply.success and reply.message in ('NO_REQUEST','STALE_REQUEST','STALE_FEEDBACK') and time.monotonic()<discovery_deadline and abs(n.state.get('speed',1))<.01:
            n.pump(.5);reply=n.set_enabled(True)
        if not reply.success:
            reason='ARM_REJECTED_'+reply.message
            raise RuntimeError(reply.message)
        armed=True;n.start=n.now();n.active=True;reason='SIM_TIMEOUT'
        with (output/'telemetry.jsonl').open('w') as stream:
            while time.monotonic()-start_wall<timeout*5+60:
                rclpy.spin_once(n,timeout_sec=.02);t=n.now()-n.start
                if t-last_record<.05:continue
                last_record=t;metric=track.metrics(n.state);row=dict(sim_time=n.now(),elapsed_sim_s=t,vision_reason=n.vision,control=n.control,**n.state,**metric);records.append(row);stream.write(json.dumps(row)+'\n')
                if perception.get('mask_backend')=='enet' and t-last_raw>=2. and n.raw is not None:
                    cv2.imwrite(str(output/'raw'/f'{t:09.3f}.jpg'),n.raw);last_raw=t
                if n.state.get('enabled'):seen_enabled=True
                if t>=next_print:print(json.dumps(dict(trial=name,t=round(t,1),s=round(metric['progress_m'],2),lateral=round(metric['lateral_error_m'],3),heading_deg=round(math.degrees(metric['heading_error_rad']),1),margin=round(metric['footprint_margin_m'],3),vision=n.vision)),flush=True);next_print+=15
                if seen_enabled and not n.state.get('enabled'):reason='LATCHED_STOP';break
                if metric['footprint_margin_m'] < -.001:reason='FOOTPRINT_OUTSIDE';break
                if metric['progress_m']>max_progress+1.:reason='PROGRESS_JUMP';break
                if metric['progress_m']>max_progress+.02:max_progress=metric['progress_m'];last_progress_wall=time.monotonic()
                if config.get('adaptive_speed'):
                    from std_msgs.msg import Float64
                    remaining=max(0.,track.required-metric['progress_m'])
                    n.terminal_pub.publish(Float64(data=min(1.,max(.03,remaining/4.))))
                if track.reached_goal(n.state,metric):reason='ROUTE_COMPLETE';break
                if t>=timeout:break
                if time.monotonic()-last_progress_wall>40:reason='NO_PROGRESS';break
        if n.raw is not None:cv2.imwrite(str(output/'end_raw.jpg'),n.raw)
        if n.debug is not None:cv2.imwrite(str(output/'end_debug.jpg'),n.debug)
    finally:
        if n.raw is not None:cv2.imwrite(str(output/'end_raw.jpg'),n.raw)
        if n.debug is not None:cv2.imwrite(str(output/'end_debug.jpg'),n.debug)
        if config.get('adaptive_speed') and reason=='ROUTE_COMPLETE':
            from std_msgs.msg import Float64
            n.terminal_pub.publish(Float64(data=0.))
            deadline=time.monotonic()+20;last_terminal=-1.
            with (output/'telemetry.jsonl').open('a') as stream:
                while time.monotonic()<deadline:
                    rclpy.spin_once(n,timeout_sec=.02)
                    if n.now()-last_terminal>=.05:
                        last_terminal=n.now()
                        row=dict(sim_time=n.now(),elapsed_sim_s=n.now()-n.start,phase='stopping',vision_reason=n.vision,control=n.control,**n.state,**track.metrics(n.state))
                        records.append(row);stream.write(json.dumps(row)+'\n')
                    if abs(n.state.get('speed',1))<.0001:break
            if abs(n.state.get('speed',1))>=.0001:reason='NORMAL_STOP_TIMEOUT'
        n.set_enabled(False)
        stop_wall=time.monotonic();last_stop=-1.
        with (output/'telemetry.jsonl').open('a') as stream:
            while time.monotonic()-stop_wall<2.:
                rclpy.spin_once(n,timeout_sec=.02)
                if armed and n.now()-last_stop>=.05:
                    last_stop=n.now();row=dict(sim_time=n.now(),elapsed_sim_s=n.now()-n.start,phase='stopping',vision_reason=n.vision,control=n.control,**n.state,**track.metrics(n.state))
                    records.append(row);stream.write(json.dumps(row)+'\n')
        n.active=False
        if n.writer:n.writer.release();n.writer=None
        if proc.poll() is None:
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGTERM)
                try:proc.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()

        log.close()
        margins=[r['footprint_margin_m'] for r in records];steering=[r.get('steering',0.) for r in records]
        driving=[r for r in records if r.get('phase')!='stopping']
        invalid_frames=[]
        if driving:
            for line in (output/'trace.jsonl').open():
                frame=json.loads(line);stamp=frame['stamp_ns']/1e9
                if n.start <= stamp <= driving[-1]['sim_time'] and not frame.get('control',{}).get('valid'):
                    invalid_frames.append(dict(stamp_ns=frame['stamp_ns'],reason=frame.get('control',{}).get('reason')))
        save(output/'invalid_driving_frames.json',invalid_frames)
        summary=dict(name=name,reason=reason,passed=reason=='ROUTE_COMPLETE' and not invalid_frames and abs(n.state.get('speed',1))<.01 and bool(margins) and min(margins)>=-.001,
            elapsed_sim_s=next((r['elapsed_sim_s'] for r in reversed(records) if r.get('phase')!='stopping'),0),route_required_m=track.required,route=track.route,goal_pose=track.goal_pose,
            progress_m=max((r['progress_m'] for r in records),default=0.),footprint_margin_min_m=min(margins) if margins else None,
            lateral_error_max_m=max((abs(r['lateral_error_m']) for r in records),default=0.),heading_error_max_deg=max((abs(math.degrees(r['heading_error_rad'])) for r in records),default=0.),
            steering_total_variation=float(np.abs(np.diff(steering)).sum()),stopped_speed=n.state.get('speed'),frames=n.frames,
            request_latency_ms_median=float(np.median(n.latencies)*1000) if n.latencies else None,request_latency_ms_p95=float(np.percentile(n.latencies,95)*1000) if n.latencies else None,
            invalid_driving_frames=len(invalid_frames),
            loss_samples=sum(not r['control'].get('valid',False) for r in records if r.get('phase')!='stopping'),
            single_samples=sum(r['control'].get('observation_mode','').startswith('single') for r in records if r.get('phase')!='stopping'),config=config,
            video='15 fps sampled debug stream; timing comes from displayed ROS timestamps',geometry_use='evaluation and terminal stop only',repeat_qualified=False)
        save(output/'summary.json',summary);print('RESULT '+json.dumps(summary),flush=True)
    return summary


def main():
    global OUT
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('mode',choices=['diagnose','speed','angle','trial','pp']);p.add_argument('--name');p.add_argument('--adaptive-speed',action='store_true');p.add_argument('--perception-config',type=Path,default=PKG/'config/highbay.yaml');p.add_argument('--route',choices=['combination','full-lap'],default='combination');p.add_argument('--lookahead',type=float,default=3.5);p.add_argument('--speed',type=float,default=.2);p.add_argument('--near-gain',type=float,default=2.5);p.add_argument('--direction-gain',type=float,default=.1);p.add_argument('--output',type=Path,default=OUT);a=p.parse_args();OUT=a.output.resolve();OUT.mkdir(parents=True,exist_ok=True)
    if os.environ.get('ROS_DOMAIN_ID')!='84' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise RuntimeError('Use the local project dev.sh environment, never a vehicle domain')
    track=Track(a.route);rclpy.init();n=Observer();deadline=time.monotonic()+15
    while time.monotonic()<deadline and ('x' not in n.state or n.info is None or n.raw is None or 'adapter_platform' not in n.state):n.pump(.2)
    if 'x' not in n.state or n.info is None or n.raw is None:raise RuntimeError('Highbay sensors unavailable')
    if n.state.get('adapter_platform')!='sim':raise RuntimeError('Experiments require the simulation adapter; refusing any hardware service')
    n.set_enabled(False);n.pump(.5);config=yaml.safe_load((PKG/'config/sim_control.yaml').read_text())
    try:
        if a.mode=='diagnose':diagnose(n,track)
        elif a.mode=='pp':
            config=yaml.safe_load((PKG/'config/sim_pure_pursuit.yaml').read_text())
            trial(n,track,a.name or f'pp_{a.lookahead}',dict(config,speed=a.speed,lookahead_m=a.lookahead,adaptive_speed=a.adaptive_speed),a.perception_config.resolve())
        elif a.mode=='speed':
            for speed in [.4,.2,.1]:trial(n,track,f'speed_{speed:.1f}',dict(config,speed=speed,direction_mode='difference'))
        elif a.mode=='angle':
            for near in [1.5,2.5,3.5]:
                for direction in [.05,.1,.2]:
                    result=trial(n,track,f'angle_n{near:.1f}_d{direction:.2f}',dict(config,speed=a.speed,k_near=near,k_direction=direction,direction_mode='tangent'))
                    if result['passed']:
                        print('FIRST COMPLETE ROUTE: freeze candidate before three independent restarts',flush=True);return
        else:
            if not a.name:raise ValueError('--name is required')
            trial(n,track,a.name,dict(config,speed=a.speed,k_near=a.near_gain,k_direction=a.direction_gain,direction_mode='tangent'))
    finally:n.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
