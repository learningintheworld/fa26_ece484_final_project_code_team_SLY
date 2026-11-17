"""Use the official simulator driver with a bounded, wall-time TF startup wait.

Upstream gem_control.py counts callbacks as retries, so queued /clock messages
can exhaust all attempts before wheel TF arrives. Zero track width then makes
all wheel speeds equal in turns. No vehicle/model/controller geometry is changed.
"""
import importlib.util
import time
from pathlib import Path
import numpy as np
import rclpy
from tf2_ros import TransformException
from ament_index_python.packages import get_package_prefix

def main(args=None):
    driver=Path(get_package_prefix('gem_gazebo'))/'lib/gem_gazebo/gem_control.py'
    spec=importlib.util.spec_from_file_location('sly_official_gem_control',driver)
    official=importlib.util.module_from_spec(spec);spec.loader.exec_module(official)
    class ReadyController(official.GEMController):
        def get_link_position(self,link):
            deadline=time.monotonic()+10.
            while rclpy.ok() and time.monotonic()<deadline:
                try:
                    t=self.tf_buffer.lookup_transform('base_link',link,rclpy.time.Time()).transform.translation
                    p=np.array([t.x,t.y,t.z])
                    if np.isfinite(p).all() and np.linalg.norm(p)>.1:return p
                except TransformException:
                    pass
                rclpy.spin_once(self,timeout_sec=.05)
            raise RuntimeError('Wheel TF unavailable: '+link+'; refusing zero-track fallback')
    rclpy.init(args=args);node=None
    try:
        node=ReadyController()
        if node.steer_joint_dist_div_2<.1:raise RuntimeError('Invalid wheel track')
        node.get_logger().info(f'Verified wheelbase={node.wheelbase:.4f} m track={2*node.steer_joint_dist_div_2:.4f} m')
        rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
