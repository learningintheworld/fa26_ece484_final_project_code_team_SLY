"""One final command outlet. Starts disarmed; hardware defaults to observe-only."""
import math
import time
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from .control import CommandGate, front_to_steering_wheel


class Adapter(Node):
    def __init__(self):
        super().__init__('sly_platform_adapter')
        self.declare_parameter('platform','sim')
        self.declare_parameter('dry_run',True)
        self.declare_parameter('hardware_config','')
        self.platform=self.get_parameter('platform').value
        self.dry=self.get_parameter('dry_run').value
        if self.platform not in ('sim','e4'):raise ValueError('platform must be sim or e4')
        self.request=None;self.received=-math.inf;self.feedback_time=-math.inf
        self.speed=0.;self.override=False;self.report_times={};self.enabled_report=False;self.bus_fault=False
        self.brake=0.;self.gear=0;self.deadman=False;self.joy_time=-math.inf
        self.cfg={};self.pubs={};self.last_tick=None;self.ever_armed=False
        self.gate=CommandGate()
        self.status_pub=self.create_publisher(DiagnosticArray,'/sly/control/status',5)
        self.create_subscription(AckermannDriveStamped,'/sly/control/request',self.on_request,5)
        self.create_service(SetBool,'/sly/control/enable',self.on_enable)
        if self.platform=='sim':
            self.create_subscription(Odometry,'/odom',self.on_odom,10)
            if not self.dry:self.pubs['sim']=self.create_publisher(AckermannDrive,'/ackermann_cmd',5)
            self.command_topics=['/ackermann_cmd']
        else:
            from pacmod2_msgs.msg import VehicleSpeedRpt, GlobalRpt, SystemRptFloat, SystemRptInt
            from pacmod2_msgs.msg import GlobalCmd, SystemCmdFloat, SystemCmdInt, PositionWithSpeed
            self.types=(GlobalCmd,SystemCmdFloat,SystemCmdInt,PositionWithSpeed)
            path=self.get_parameter('hardware_config').value
            if path:self.cfg=yaml.safe_load(Path(path).read_text())
            self.gate.max_speed=float(self.cfg.get('max_speed',.2))
            self.gate.max_steering=float(self.cfg.get('max_front_steering',.25))
            if not self.dry and not self.hardware_ready():
                raise ValueError('Hardware profile unverified: remain dry_run until direction, speed and braking are tested')
            self.create_subscription(VehicleSpeedRpt,'/pacmod/vehicle_speed_rpt',self.on_speed,10)
            self.create_subscription(GlobalRpt,'/pacmod/global_rpt',self.on_global,10)
            self.create_subscription(SystemRptFloat,'/pacmod/steering_rpt',lambda m:self.report('steering',m),10)
            self.create_subscription(SystemRptFloat,'/pacmod/brake_rpt',lambda m:self.report('brake',m),10)
            self.create_subscription(SystemRptInt,'/pacmod/shift_rpt',lambda m:self.report('gear',m),10)
            self.create_subscription(Joy,'/joy',self.on_joy,10)
            self.command_topics=['/pacmod/global_cmd','/pacmod/accel_cmd','/pacmod/brake_cmd','/pacmod/steering_cmd','/pacmod/shift_cmd']
            if not self.dry:
                for key,typ in [('global',GlobalCmd),('accel',SystemCmdFloat),('brake',SystemCmdFloat),('steering',PositionWithSpeed),('shift',SystemCmdInt)]:
                    self.pubs[key]=self.create_publisher(typ,'/pacmod/'+key+'_cmd',5)
        # On hardware the watchdog does not depend on ROS clock progress.
        clock=Clock(clock_type=ClockType.STEADY_TIME) if self.platform=='e4' else self.get_clock()
        self.create_timer(.05,self.tick,clock=clock)

    def now(self):return self.get_clock().now().nanoseconds/1e9
    def age_clock(self):return time.monotonic() if self.platform=='e4' else self.now()
    def hardware_ready(self):
        c=self.cfg
        return bool(c.get('hardware_verified')) and all(isinstance(c.get(k),(int,float)) and math.isfinite(c[k]) and 0<c[k]<=1
            for k in ('stop_brake','max_throttle','speed_kp','max_speed','max_front_steering')) and c['max_speed']<=.5 and c['max_front_steering']<=math.radians(35)
    def on_request(self,m):
        self.request=(m.header.stamp.sec+m.header.stamp.nanosec/1e9,m.drive.speed,m.drive.steering_angle)
        self.received=self.age_clock()
    def on_odom(self,m):
        self.speed=float(m.twist.twist.linear.x);self.feedback_time=self.age_clock()
    def feedback_receipt(self,m):
        stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        age=self.now()-stamp
        return self.age_clock()-age if 0<=age<=.5 else -math.inf
    def on_speed(self,m):
        self.speed=float(m.vehicle_speed)
        self.feedback_time=self.feedback_receipt(m) if m.vehicle_speed_valid else -math.inf
    def on_global(self,m):
        self.override=bool(m.override_active);self.enabled_report=bool(m.enabled)
        self.bus_fault=any(getattr(m,k) for k in ('veh_can_timeout','str_can_timeout','brk_can_timeout','usr_can_timeout','user_can_read_errors'))
        self.report_times['global']=self.feedback_receipt(m)
    def report(self,key,m):
        self.report_times[key]=self.feedback_receipt(m) if math.isfinite(m.output) else -math.inf
        if key=='brake':self.brake=float(m.output)
        if key=='gear':self.gear=int(m.output)
    def on_joy(self,m):
        button=int(self.cfg.get('deadman_button',4))
        self.deadman=button<len(m.buttons) and bool(m.buttons[button]);self.joy_time=self.age_clock()
    def competing(self):
        own=0 if self.dry else 1
        return any(self.count_publishers(t)>own for t in self.command_topics)
    def flags(self):
        age=self.age_clock()
        blocked=self.override
        self.hardware_fault=''
        if self.platform=='e4':
            if self.bus_fault:self.hardware_fault='PACMOD_BUS_FAULT'
            elif any(age-self.report_times.get(k,-math.inf)>.5 for k in ('global','steering','brake','gear')):self.hardware_fault='STALE_PACMOD_FEEDBACK'
            elif age-self.joy_time>.5 or not self.deadman:self.hardware_fault='DEADMAN_RELEASED'
        return dict(override=blocked,competing=self.competing())
    def gate_input(self):
        # Check both source timestamp and independent receipt age.
        now=self.now();age=self.age_clock()
        request=self.request
        if age-self.received>.5 or self.hardware_fault:request=None
        feedback=now-(age-self.feedback_time)
        return now,request,feedback,self.speed
    def on_enable(self,req,res):
        if not req.data:
            self.gate.disarm();res.success=True;res.message='Stop latched';return res
        flags=self.flags()
        if self.dry:
            res.success=False;res.message='Observe-only adapter: no actuator publishers';return res
        ok,reason=self.gate.arm(*self.gate_input(),**flags)
        if ok and self.platform=='e4':
            GlobalCmd,_,SystemCmdInt,_=self.types
            self.pubs['global'].publish(GlobalCmd(enable=True,clear_override=False))
            self.pubs['shift'].publish(SystemCmdInt(command=3))
            self.armed_at=self.age_clock()
        self.ever_armed |= ok
        res.success=ok;res.message=self.hardware_fault or reason
        return res
    def tick(self):
        flags=self.flags()
        speed,steering=self.gate.step(*self.gate_input(),**flags)
        if self.platform=='e4' and self.gate.enabled:
            if self.age_clock()-self.armed_at>1. and (not self.enabled_report or self.gear!=3):
                self.gate.disarm('PACMOD_NOT_READY');speed=steering=0.
            elif not self.enabled_report or self.gear!=3:speed=steering=0.
        # Never publish competing commands, including zero commands, into a
        # joystick/other controller's outlet. Operator takes over in this case.
        if flags['competing']:self.ever_armed=False
        if not self.dry and not flags['competing']:
            if self.platform=='sim':
                self.pubs['sim'].publish(AckermannDrive(speed=float(speed),steering_angle=float(steering)))
            elif self.ever_armed:
                self.send_e4(speed,steering)
        status=DiagnosticStatus(name='sly/control',hardware_id=self.platform,
            level=DiagnosticStatus.OK if self.gate.enabled else DiagnosticStatus.WARN,
            message=self.hardware_fault or self.gate.reason,
            values=[KeyValue(key='enabled',value=str(self.gate.enabled)),KeyValue(key='dry_run',value=str(self.dry)),
                    KeyValue(key='speed',value=str(speed)),KeyValue(key='steering',value=str(steering))])
        msg=DiagnosticArray(status=[status]);msg.header.stamp=self.get_clock().now().to_msg();self.status_pub.publish(msg)
    def send_e4(self,speed,steering):
        GlobalCmd,FloatCmd,_,SteerCmd=self.types
        if self.override:
            self.pubs['global'].publish(GlobalCmd(enable=False,clear_override=False))
            self.ever_armed=False
            return
        if self.gate.enabled:
            msg=GlobalCmd(enable=True,clear_override=False);msg.header.stamp=self.get_clock().now().to_msg()
            self.pubs['global'].publish(msg)
        # No kick-start: positive speed error only, throttle/brake exclusive.
        throttle=min(self.cfg['max_throttle'],max(0.,self.cfg['speed_kp']*(speed-self.speed))) if speed>0 else 0.
        brake=self.cfg['stop_brake'] if speed<=0 or self.speed>speed+.1 else 0.
        if brake>0:throttle=0.
        self.pubs['accel'].publish(FloatCmd(command=float(throttle)))
        self.pubs['brake'].publish(FloatCmd(command=float(brake)))
        if speed>0:
            self.pubs['steering'].publish(SteerCmd(angular_position=front_to_steering_wheel(steering),angular_velocity_limit=1.))
        # Retain enabled braking until standstill; disengagement is not braking.
        if (not self.gate.enabled and math.isfinite(self.speed) and abs(self.speed)<.05
                and 0<=self.age_clock()-self.feedback_time<=.5):
            self.pubs['global'].publish(GlobalCmd(enable=False,clear_override=False))
            self.ever_armed=False


def main(args=None):
    rclpy.init(args=args)
    node=Adapter()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        # Normal stopping uses /sly/control/enable false while node remains
        # alive; Ctrl+C must not be used as the physical braking procedure.
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
