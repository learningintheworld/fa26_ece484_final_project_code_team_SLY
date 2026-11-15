"""Run in an isolated ROS domain, without Gazebo or real hardware."""
import json,subprocess,time,os,signal
from pathlib import Path
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped,AckermannDrive
from diagnostic_msgs.msg import DiagnosticArray
from std_srvs.srv import SetBool
if os.environ.get('ROS_DOMAIN_ID') not in {str(i) for i in range(90,100)} or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
 raise RuntimeError('Mock test requires an isolated local ROS domain 90–99; never run on the vehicle control domain')
rclpy.init();n=Node('sly_gate_check');req=n.create_publisher(AckermannDriveStamped,'/sly/control/request',10)
feedback=n.create_publisher(Odometry,'/odom',10);state={};commands=[];tests=[]
n.create_subscription(AckermannDrive,'/ackermann_cmd',lambda m:commands.append((time.monotonic(),m.speed)),10)
def status(m):
 for s in m.status:
  if s.name=='sly/control':state['reason']=s.message
n.create_subscription(DiagnosticArray,'/sly/control/status',status,10)
c=n.create_client(SetBool,'/sly/control/enable')
p=subprocess.Popen(['ros2','run','sly_lane_following','lane_adapter','--ros-args','-p','platform:=sim','-p','dry_run:=false'],start_new_session=True)
def pump(seconds,mode='good'):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  stamp=n.get_clock().now().to_msg()
  if mode!='no_request':
   m=AckermannDriveStamped();m.header.stamp=stamp;m.drive.speed=.3;m.drive.steering_angle=.1
   if mode=='stale':m.header.stamp.sec-=2
   if mode=='invalid':m.drive.speed=0.
   req.publish(m)
  if mode!='no_feedback':feedback.publish(Odometry())
  rclpy.spin_once(n,timeout_sec=.02)
def enable(flag):
 f=c.call_async(SetBool.Request(data=flag))
 end=time.monotonic()+2
 while not f.done() and time.monotonic()<end:pump(.03)
 assert f.done() and f.result().success,str(f.result() if f.done() else 'timeout')
try:
 assert c.wait_for_service(timeout_sec=10)
 pump(1)
 assert not any(v>0 for _,v in commands),'Motion before explicit arm'
 for mode,expected in [('no_request','NO_REQUEST'),('stale','STALE_REQUEST'),('no_feedback','STALE_FEEDBACK'),('invalid','STOP_REQUEST')]:
  pump(.7);enable(True);pump(.25)
  assert any(v>0 for _,v in commands[-5:]),'No armed motion'
  pump(.8,mode)
  assert all(v==0 for _,v in commands[-5:]),mode
  observed=state.get('reason');assert observed==expected,(mode,observed)
  pump(.3);assert all(v==0 for _,v in commands[-5:]),'Automatic restart'
  tests.append(dict(fault=mode,reason=observed,stopped=True,remained_disarmed=True))
 enable(True);pump(.2);other=n.create_publisher(AckermannDrive,'/ackermann_cmd',10)
 pump(1)
 assert state.get('reason')=='COMPETING_PUBLISHER',state
 n.destroy_publisher(other);pump(.5)
 assert all(v==0 for _,v in commands[-5:])
 tests.append(dict(fault='competing_publisher',stopped=True,remained_disarmed=True))
 enable(False)
 out=Path('results/milestone3/ros_gate_check.json');out.write_text(json.dumps(dict(passed=True,tests=tests),indent=2));print(out.read_text())
finally:
 os.killpg(p.pid,signal.SIGINT)
 try:p.wait(timeout=5)
 except subprocess.TimeoutExpired:p.terminate();p.wait(timeout=5)
 n.destroy_node();rclpy.shutdown()
