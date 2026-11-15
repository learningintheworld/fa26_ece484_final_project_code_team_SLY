"""Synthetic image-to-command fault tests, isolated ROS domain; no hardware."""
import json,os,signal,subprocess,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image,CameraInfo
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive
from diagnostic_msgs.msg import DiagnosticArray
from std_srvs.srv import SetBool
from test_core import paint_frame
if os.environ.get('ROS_DOMAIN_ID') not in {str(i) for i in range(90,100)} or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
 raise RuntimeError('Mock test requires an isolated local ROS domain 90–99; never run on the vehicle control domain')
rclpy.init();n=Node('sly_pipeline_check');bridge=CvBridge();commands=[];state={};tests=[]
images=n.create_publisher(Image,'/oak/rgb/image_raw',10)
infos=n.create_publisher(CameraInfo,'/oak/rgb/camera_info',10)
feedback=n.create_publisher(Odometry,'/odom',10)
n.create_subscription(AckermannDrive,'/ackermann_cmd',lambda m:commands.append(m.speed),10)
def status(m):
 for s in m.status:
  if s.name=='sly/control':state['reason']=s.message
n.create_subscription(DiagnosticArray,'/sly/control/status',status,10)
c=n.create_client(SetBool,'/sly/control/enable')
root=Path('src/sly_lane_following/config').resolve()
def spawn(entry,args):return subprocess.Popen(['ros2','run','sly_lane_following',entry,'--ros-args']+args,start_new_session=True)
perception=spawn('lane_node',['-p','control_config:='+str(root/'sim_control.yaml')])
adapter=spawn('lane_adapter',['-p','platform:=sim','-p','dry_run:=false'])
def pump(seconds,mode='good'):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  stamp=n.get_clock().now().to_msg()
  if mode!='no_image':
   im=bridge.cv2_to_imgmsg(paint_frame(neighbor_brightness=0),'bgr8');im.header.stamp=stamp;im.header.frame_id='camera'
   if mode=='stale':im.header.stamp.sec-=2
   info=CameraInfo();info.header=im.header;info.width=768;info.height=480;info.k=[450.,0.,384.,0.,450.,240.,0.,0.,1.];info.d=[0.]*5;info.distortion_model='plumb_bob'
   images.publish(im);infos.publish(info)
  feedback.publish(Odometry());rclpy.spin_once(n,timeout_sec=.035)
def arm():
 f=c.call_async(SetBool.Request(data=True));start=time.monotonic()
 while not f.done() and time.monotonic()-start<3:pump(.04)
 assert f.done() and f.result().success,str(f.result() if f.done() else 'timeout')
try:
 assert c.wait_for_service(timeout_sec=10);pump(2)
 assert not any(v>0 for v in commands)
 for fault in ['no_image','stale','node_stopped']:
  pump(1);arm();pump(.3);assert any(v>0 for v in commands[-8:]),'No motion under good input'
  if fault=='node_stopped':os.killpg(perception.pid,signal.SIGINT);perception.wait(timeout=5)
  pump(1.2,'good' if fault=='node_stopped' else fault)
  assert commands[-5:]==[0.]*5,(fault,commands[-5:])
  reason=state['reason'];pump(.5)
  assert commands[-5:]==[0.]*5,'Fresh data rearmed the vehicle'
  tests.append(dict(fault=fault,reason=reason,stopped=True,remained_disarmed=True))
 report=dict(passed=True,environment='synthetic RGB/CameraInfo and odometry; no vehicle',tests=tests)
 Path('results/milestone3/ros_pipeline_check.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
finally:
 for p in [perception,adapter]:
  if p.poll() is None:os.killpg(p.pid,signal.SIGINT);p.wait(timeout=5)
 n.destroy_node();rclpy.shutdown()
