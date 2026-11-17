"""PACMod protocol test: isolated synthetic reports; never vehicle validation."""
import json, subprocess, time, os, signal
from pathlib import Path
import rclpy, yaml
from rclpy.node import Node
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped
from std_srvs.srv import SetBool
from pacmod2_msgs.msg import VehicleSpeedRpt,GlobalRpt,SystemRptFloat,SystemRptInt,GlobalCmd,SystemCmdFloat,SystemCmdInt,PositionWithSpeed
if os.environ.get('ROS_DOMAIN_ID') not in {str(i) for i in range(90,100)} or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
 raise RuntimeError('Requires isolated local ROS domain 90–99, never vehicle domain')
rclpy.init();n=Node('sly_e4_mock');out=Path('results/e4_prepare_v1');out.mkdir(parents=True,exist_ok=True)
c=n.create_client(SetBool,'/sly/control/enable')
profile=out/'SYNTHETIC_ONLY.yaml'
profile.write_text(yaml.safe_dump(dict(hardware_verified=True,max_speed=.2,max_front_steering=.25,deadman_button=4,stop_brake=.2,shift_brake=.6,max_throttle=.2,speed_kp=.5)))
state=dict(enabled=False,gear=2,override=False,deadman=True,speed_valid=True,speed=0.,brake=0.,brake_ack=True,gear_ack=True,request=True)
received=[];checks=[];pubs={}
for key,typ in [('vehicle_speed',VehicleSpeedRpt),('global',GlobalRpt),('steering',SystemRptFloat),('brake',SystemRptFloat),('shift',SystemRptInt)]:
 pubs[key]=n.create_publisher(typ,'/pacmod/'+key+'_rpt',10)
joy=n.create_publisher(Joy,'/joy',10);req=n.create_publisher(AckermannDriveStamped,'/sly/control/request',10)
def callback(key,m):
 value=getattr(m,'command',getattr(m,'enable',None))
 received.append((time.monotonic(),key,value))
 if key=='global':state['enabled']=m.enable
 if key=='brake' and state['brake_ack']:state['brake']=m.command
 if key=='shift' and state['gear_ack']:state['gear']=m.command
for key,typ in [('global',GlobalCmd),('accel',SystemCmdFloat),('brake',SystemCmdFloat),('shift',SystemCmdInt),('steering',PositionWithSpeed)]:
 n.create_subscription(typ,'/pacmod/'+key+'_cmd',lambda m,k=key:callback(k,m),10)
p=subprocess.Popen(['ros2','run','sly_lane_following','lane_adapter','--ros-args','-p','platform:=e4','-p','dry_run:=false','-p','hardware_config:='+str(profile.resolve())],start_new_session=True)
def pump(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  stamp=n.get_clock().now().to_msg()
  for key,typ in [('vehicle_speed',VehicleSpeedRpt),('global',GlobalRpt),('steering',SystemRptFloat),('brake',SystemRptFloat),('shift',SystemRptInt)]:
   m=typ();m.header.stamp=stamp
   if key=='vehicle_speed':m.vehicle_speed=state['speed'];m.vehicle_speed_valid=state['speed_valid']
   if key=='global':m.enabled=state['enabled'];m.override_active=state['override']
   if key=='shift':m.output=state['gear']
   if key=='brake':m.output=state['brake']
   pubs[key].publish(m)
  j=Joy();j.header.stamp=stamp;j.buttons=[0,0,0,0,int(state['deadman'])];joy.publish(j)
  if state['request']:
   m=AckermannDriveStamped();m.header.stamp=stamp;m.drive.speed=.2;m.drive.steering_angle=.1;req.publish(m)
  rclpy.spin_once(n,timeout_sec=.02)
def service(enabled,success=True):
 f=c.call_async(SetBool.Request(data=enabled));end=time.monotonic()+3
 while not f.done() and time.monotonic()<end:pump(.03)
 assert f.done() and f.result().success==success,str(f.result() if f.done() else 'timeout')
def has(key,predicate):return any(k==key and predicate(v) for _,k,v in received)
def no_throttle():assert not has('accel',lambda v:v>0),received[-15:]
def ready():
 state.update(enabled=False,gear=2,override=False,deadman=True,speed_valid=True,speed=0.,brake=0.,brake_ack=True,gear_ack=True,request=True)
 pump(.2);received.clear();service(True);pump(.7)
 assert has('accel',lambda v:v>0),'No throttle after verified engagement'
try:
 assert c.wait_for_service(timeout_sec=10);pump(.7)
 assert not received,'Commands before explicit enable';checks.append('no commands before enable')
 state['gear']=3;pump(.1);service(True,False);assert not received;state['gear']=2;pump(.1)
 state['brake_ack']=False;state['brake']=0.;service(True);pump(.4)
 no_throttle();assert not has('shift',lambda v:v==3);checks.append('no shift without brake feedback')
 state['gear_ack']=False;state['brake_ack']=True;pump(.4)
 no_throttle();assert has('shift',lambda v:v==3);checks.append('no throttle without drive feedback')
 state['gear_ack']=True;pump(.5)
 assert has('accel',lambda v:v>0);checks.append('brake then drive then throttle')
 state['speed']=.15;pump(.1);service(False);pump(.12);received.clear();pump(.3)
 no_throttle();assert has('brake',lambda v:v>0);assert not has('shift',lambda v:v==2);assert not has('global',lambda v:v is False)
 checks.append('moving stop retains enabled braking, no Neutral')
 state['speed_valid']=False;state['speed']=0.;received.clear();pump(.3)
 assert not has('shift',lambda v:v==2);assert not has('global',lambda v:v is False);checks.append('invalid speed never confirms standstill')
 state['speed_valid']=True;state['gear_ack']=False;received.clear();pump(.5)
 assert has('shift',lambda v:v==2);assert not has('global',lambda v:v is False);checks.append('wait for Neutral feedback')
 state['gear_ack']=True;pump(.5);assert has('global',lambda v:v is False)
 received.clear();pump(.3);assert not received;checks.append('disable acknowledged, no automatic restart')
 ready();state['request']=False;received.clear();pump(1.2)
 no_throttle_after=[r for r in received if r[0]>received[0][0]+.6 and r[1]=='accel' and r[2]>0]
 assert not no_throttle_after;assert has('brake',lambda v:v>0);checks.append('stale request stops')
 ready();state['deadman']=False;received.clear();pump(.8)
 assert has('brake',lambda v:v>0);checks.append('deadman release stops')
 ready();state['override']=True;received.clear();pump(.3)
 assert has('global',lambda v:v is False)
 state['override']=False;received.clear();pump(.3);assert not received;checks.append('physical override releases ownership')
 ready();state['gear']=2;pump(.2);received.clear();pump(.5);no_throttle();checks.append('unexpected gear loss latches stop')
 ready();state['brake_ack']=False;state['speed']=.1
 received.clear();service(False);pump(.2);state['speed']=0.;pump(.3)
 state['brake_ack']=True;pump(.6)
 # Lack of brake confirmation must timeout without ever requesting Drive.
 state.update(enabled=False,gear=2,brake=0.,brake_ack=False);pump(.2)
 received.clear();service(True);pump(3.7)
 no_throttle();assert not has('shift',lambda v:v==3);checks.append('brake confirmation timeout, no Drive')
 state['brake_ack']=True;pump(.7)
 ready();state['speed_valid']=False;pump(.12);received.clear();pump(.3)
 assert has('brake',lambda v:v>0);no_throttle();checks.append('feedback loss stops while driving')
 state['speed_valid']=True;pump(.7)
 ready();state['enabled']=False;received.clear();pump(.3)
 assert not has('global',lambda v:v is True);checks.append('unexpected PACMod disable never re-enables itself')
 ready();os.killpg(p.pid,signal.SIGINT);pump(1.5)
 assert p.poll() is not None and state['gear']==2 and not state['enabled'];checks.append('Ctrl+C best-effort stop Neutral disable')
 report=dict(passed=True,checks=checks,environment='localhost isolated synthetic PACMod; no physical validation')
 (out/'e4_mock_check.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
finally:
 if p.poll() is None:
  os.killpg(p.pid,signal.SIGINT);pump(6.)
  if p.poll() is None:os.killpg(p.pid,signal.SIGKILL)
 n.destroy_node();rclpy.shutdown()
