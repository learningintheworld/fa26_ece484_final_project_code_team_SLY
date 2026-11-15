"""PACMod adapter contract test with synthetic reports, isolated ROS domain only."""
import json,subprocess,time,os,signal
from pathlib import Path
import rclpy,yaml
from rclpy.node import Node
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped
from std_srvs.srv import SetBool
from pacmod2_msgs.msg import VehicleSpeedRpt,GlobalRpt,SystemRptFloat,SystemRptInt,GlobalCmd,SystemCmdFloat,SystemCmdInt,PositionWithSpeed
if os.environ.get('ROS_DOMAIN_ID') not in {str(i) for i in range(90,100)} or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
 raise RuntimeError('Mock test requires an isolated local ROS domain 90–99; never run on the vehicle control domain')
rclpy.init();n=Node('sly_e4_mock');out=Path('results/milestone3');c=n.create_client(SetBool,'/sly/control/enable')
profile=out/'mock_e4_profile.yaml';profile.write_text(yaml.safe_dump(dict(hardware_verified=True,max_speed=.2,max_front_steering=.25,deadman_button=4,stop_brake=.2,max_throttle=.2,speed_kp=.5)))
state=dict(enabled=False,gear=2,override=False,deadman=True,speed_valid=True);received=[]
pubs={}
for name,typ in [('vehicle_speed',VehicleSpeedRpt),('global',GlobalRpt),('steering',SystemRptFloat),('brake',SystemRptFloat),('shift',SystemRptInt)]:
 pubs[name]=n.create_publisher(typ,'/pacmod/'+name+'_rpt',10)
joy=n.create_publisher(Joy,'/joy',10);req=n.create_publisher(AckermannDriveStamped,'/sly/control/request',10)
def callback(key,m):
 received.append((time.monotonic(),key,getattr(m,'command',getattr(m,'enable',None))))
 if key=='global':state['enabled']=m.enable
 if key=='shift':state['gear']=m.command
for key,typ in [('global',GlobalCmd),('accel',SystemCmdFloat),('brake',SystemCmdFloat),('shift',SystemCmdInt),('steering',PositionWithSpeed)]:
 n.create_subscription(typ,'/pacmod/'+key+'_cmd',lambda m,k=key:callback(k,m),10)
p=subprocess.Popen(['ros2','run','sly_lane_following','lane_adapter','--ros-args','-p','platform:=e4','-p','dry_run:=false','-p','hardware_config:='+str(profile.resolve())],start_new_session=True)
def pump(seconds,request=True):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  stamp=n.get_clock().now().to_msg()
  for key,typ in [('vehicle_speed',VehicleSpeedRpt),('global',GlobalRpt),('steering',SystemRptFloat),('brake',SystemRptFloat),('shift',SystemRptInt)]:
   m=typ();m.header.stamp=stamp
   if key=='vehicle_speed':m.vehicle_speed=0.;m.vehicle_speed_valid=state['speed_valid']
   if key=='global':m.enabled=state['enabled'];m.override_active=state['override']
   if key=='shift':m.output=state['gear']
   pubs[key].publish(m)
  j=Joy();j.header.stamp=stamp;j.buttons=[0,0,0,0,int(state['deadman'])];joy.publish(j)
  if request:
   m=AckermannDriveStamped();m.header.stamp=stamp;m.drive.speed=.2;m.drive.steering_angle=.1;req.publish(m)
  rclpy.spin_once(n,timeout_sec=.02)
def arm():
 f=c.call_async(SetBool.Request(data=True));t=time.monotonic()
 while not f.done() and time.monotonic()-t<3:pump(.03)
 assert f.done() and f.result().success,str(f.result() if f.done() else 'timeout')
try:
 assert c.wait_for_service(timeout_sec=10);pump(.7)
 assert not received,'Hardware published before explicit enable'
 arm();pump(.5)
 assert any(k=='accel' and v>0 for _,k,v in received),'No throttle after arm'
 assert any(k=='steering' for _,k,v in received)
 received.clear();pump(.9,request=False)
 assert any(k=='brake' and v>0 for _,k,v in received),'No active braking on timeout'
 assert any(k=='global' and v is False for _,k,v in received),'No disable after stationary report'
 received.clear();pump(.4)
 assert not any(k=='accel' and v>0 for _,k,v in received),'Automatic restart'
 arm();pump(.3);state['speed_valid']=False;received.clear();pump(.3)
 assert any(k=='brake' and v>0 for _,k,v in received),'Invalid speed did not stop'
 assert not any(k=='global' and v is False for _,k,v in received),'Invalid speed falsely treated as standstill'
 state['speed_valid']=True;pump(.2);arm();pump(.3)
 state['override']=True;received.clear();pump(.2)
 assert any(k=='global' and v is False for _,k,v in received)
 state['override']=False;received.clear();pump(.3)
 assert not received,'Adapter retained ownership after manual takeover'
 report=dict(passed=True,environment='isolated ROS domain, synthetic PACMod only; not physical validation',
             checks=['no commands before enable','throttle and steering after explicit arm','active brake on timeout',
                     'disengage after stationary feedback','no automatic restart','invalid speed stops','manual override releases ownership'])
 (out/'e4_mock_check.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
finally:
 os.killpg(p.pid,signal.SIGINT)
 try:p.wait(timeout=5)
 except subprocess.TimeoutExpired:p.terminate();p.wait(timeout=5)
 n.destroy_node();rclpy.shutdown()
