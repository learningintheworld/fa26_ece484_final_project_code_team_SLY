"""Synthetic image-to-command fault tests, isolated ROS domain; no hardware."""
import json,os,signal,subprocess,time
import cv2
import numpy as np
from pathlib import Path
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image,CameraInfo
from nav_msgs.msg import Odometry,Path as RosPath
from ackermann_msgs.msg import AckermannDrive
from diagnostic_msgs.msg import DiagnosticArray
from std_srvs.srv import SetBool,Trigger
from test_core import paint_frame
if os.environ.get('ROS_DOMAIN_ID') not in {str(i) for i in range(90,100)} or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
 raise RuntimeError('Mock test requires an isolated local ROS domain 90–99; never run on the vehicle control domain')
rclpy.init();n=Node('sly_pipeline_check');bridge=CvBridge();commands=[];state={};tests=[]
images=n.create_publisher(Image,'/oak/rgb/image_raw',10)
infos=n.create_publisher(CameraInfo,'/oak/rgb/camera_info',10)
feedback=n.create_publisher(Odometry,'/odom',10)
n.create_subscription(AckermannDrive,'/ackermann_cmd',lambda m:commands.append(m.speed),10)
paths=[]
n.create_subscription(RosPath,'/sly/lane/local_path',lambda m:paths.append(m),10)
def status(m):
 for s in m.status:
  if s.name=='sly/control':state['reason']=s.message
n.create_subscription(DiagnosticArray,'/sly/control/status',status,10)
c=n.create_client(SetBool,'/sly/control/enable')
reset_client=n.create_client(Trigger,'/sly/lane/reset')
root=Path('src/sly_lane_following/config').resolve()
def spawn(entry,args):return subprocess.Popen(['ros2','run','sly_lane_following',entry,'--ros-args']+args,start_new_session=True)
perception=spawn('lane_node',['-p','control_config:='+str(root/os.environ.get('SLY_TEST_CONTROL','sim_control.yaml'))])
adapter=spawn('lane_adapter',['-p','platform:=sim','-p','dry_run:=false'])
def pump(seconds,mode='good'):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  stamp=n.get_clock().now().to_msg()
  if mode!='no_image':
   pixels=paint_frame(neighbor_brightness=0);intrinsics=[450.,0.,384.,0.,450.,240.,0.,0.,1.]
   if os.environ.get('SLY_TEST_CONTROL')=='sim_pure_pursuit.yaml':
    pixels=np.full((600,800,3),35,np.uint8);intrinsics=[476.7,0.,400.,0.,476.7,300.,0.,0.,1.]
    if mode!='both_missing':
     # Independent pinhole construction: constant 3.3 m wide straight lane.
     for side in ([1] if mode=='single' else [1,-1]):
      endpoints=[(round(400-476.7*side*1.65/(x-1.264)),round(300+476.7*1.636/(x-1.264))) for x in (3.9,14.)]
      cv2.line(pixels,endpoints[0],endpoints[1],(145,145,145),7)
   im=bridge.cv2_to_imgmsg(pixels,'bgr8');im.header.stamp=stamp;im.header.frame_id='camera'
   if mode=='stale':im.header.stamp.sec-=2
   info=CameraInfo();info.header=im.header;info.height,info.width=pixels.shape[:2];info.k=intrinsics;info.d=[0.]*5;info.distortion_model='plumb_bob'
   images.publish(im);infos.publish(info)
  feedback.publish(Odometry());rclpy.spin_once(n,timeout_sec=.035)
def arm():
 f=c.call_async(SetBool.Request(data=True));start=time.monotonic()
 while not f.done() and time.monotonic()-start<3:pump(.04)
 assert f.done() and f.result().success,str(f.result() if f.done() else 'timeout')
try:
 assert c.wait_for_service(timeout_sec=10);pump(2)
 assert not any(v>0 for v in commands)
 if os.environ.get('SLY_TEST_CONTROL')=='sim_pure_pursuit.yaml':
  assert any(p.poses and p.header.frame_id=='sly_rear_axle_ground' and p.header.stamp.sec>0 for p in paths),'No stamped metric path under valid input'
 faults=['no_image','stale','node_stopped']
 if os.environ.get('SLY_TEST_CONTROL')=='sim_pure_pursuit.yaml':
  arm();pump(.6,'single');assert any(v>0 for v in commands[-8:]),'Single visible boundary stopped valid tracking'
  pump(.7,'both_missing');assert commands[-5:]==[0.]*5,'Both boundaries missing did not stop'
  pump(.5);assert commands[-5:]==[0.]*5,'Recovered paint rearmed control'
  tests.append(dict(fault='both_missing',stopped=True,remained_disarmed=True))
  f=reset_client.call_async(Trigger.Request())
  reset_deadline=time.monotonic()+3
  while not f.done() and time.monotonic()<reset_deadline:pump(.04)
  assert f.done() and f.result().success,'Perception reset failed or timed out'
 for fault in faults:
  pump(1);arm();pump(.3);assert any(v>0 for v in commands[-8:]),'No motion under good input'
  if fault=='node_stopped':os.killpg(perception.pid,signal.SIGINT);perception.wait(timeout=5)
  pump(1.2,'good' if fault=='node_stopped' else fault)
  assert commands[-5:]==[0.]*5,(fault,commands[-5:])
  reason=state['reason']
  if fault!='node_stopped':assert paths and not paths[-1].poses,'Invalid input retained a usable metric path'
  pump(.5)
  assert commands[-5:]==[0.]*5,'Fresh data rearmed the vehicle'
  tests.append(dict(fault=fault,reason=reason,stopped=True,remained_disarmed=True))
 report=dict(passed=True,environment='synthetic RGB/CameraInfo and odometry; no vehicle',tests=tests)
 Path(os.environ.get('SLY_TEST_REPORT','results/milestone3/ros_pipeline_check.json')).write_text(json.dumps(report,indent=2));print(json.dumps(report))
finally:
 for p in [perception,adapter]:
  if p.poll() is None:os.killpg(p.pid,signal.SIGINT);p.wait(timeout=5)
 n.destroy_node();rclpy.shutdown()
