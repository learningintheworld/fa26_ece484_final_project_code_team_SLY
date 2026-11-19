"""Run in isolated ROS domain with pacmod2_msgs; no vehicle connection."""
import pytest
pytest.importorskip('pacmod2_msgs.msg')
import rclpy
from rclpy.parameter import Parameter
from std_srvs.srv import SetBool
from pathlib import Path
from sly_lane_following.adapter import Adapter


def test_unverified_standby_has_no_actuator_publishers_and_cannot_enable(monkeypatch):
    from rclpy.node import Node
    original=Node.__init__
    profile=Path(__file__).resolve().parents[1]/'config/e4_adapter.yaml'
    def init(self,*args,**kwargs):
        kwargs['parameter_overrides']=[Parameter('platform',value='e4'),Parameter('dry_run',value=False),Parameter('unverified_observe',value=True),Parameter('hardware_config',value=str(profile))]
        original(self,*args,**kwargs)
    monkeypatch.setattr(Node,'__init__',init)
    rclpy.init()
    node=None
    try:
        node=Adapter()
        assert node.dry and not node.pubs and not node.hardware_ready()
        for topic in node.command_topics: assert node.count_publishers(topic)==0
        response=node.on_enable(SetBool.Request(data=True),SetBool.Response())
        assert not response.success and 'Observe-only' in response.message
        node.cfg['hardware_verified']=True
        node.tick()
        assert node.dry and not node.pubs  # No transition on config changes.
    finally:
        if node is not None:node.destroy_node()
        rclpy.shutdown()


def test_observe_does_not_treat_missing_joystick_as_hardware_fault():
    from types import SimpleNamespace
    state=SimpleNamespace(override=False,hardware_fault='',platform='e4',bus_fault=False,
        report_times={k:10. for k in ('global','steering','brake','gear')},
        joy_time=float('-inf'),deadman=False,dry=True,age_clock=lambda:10.,competing=lambda:False)
    Adapter.flags(state)
    assert state.hardware_fault==''
    state.dry=False
    Adapter.flags(state)
    assert state.hardware_fault=='DEADMAN_RELEASED'
    state.dry=True;state.report_times['brake']=0.
    Adapter.flags(state)
    assert state.hardware_fault=='STALE_PACMOD_FEEDBACK'
