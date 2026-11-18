"""Read-only vehicle preflight; never publishes actuator commands."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/sly_lane_following'))


def main():
    print('Python:', sys.executable, sys.version.split()[0], flush=True)
    if sys.version_info[:2] != (3, 10):
        raise RuntimeError('Vehicle Humble requires its Python 3.10 environment')
    if sys.argv[1] == 'ros':
        import rclpy
        import cv_bridge
        import yaml
        from ackermann_msgs.msg import AckermannDriveStamped
        from sensor_msgs.msg import Image, CameraInfo
        from nav_msgs.msg import Path as RosPath
        from diagnostic_msgs.msg import DiagnosticArray
        from std_srvs.srv import SetBool
        from pacmod2_msgs.msg import VehicleSpeedRpt, GlobalRpt, SystemRptFloat, SystemRptInt
        from pacmod2_msgs.msg import GlobalCmd, SystemCmdFloat, SystemCmdInt, PositionWithSpeed
        for cls in (AckermannDriveStamped, Image, CameraInfo, RosPath, DiagnosticArray,
                    VehicleSpeedRpt, GlobalRpt, SystemRptFloat, SystemRptInt,
                    GlobalCmd, SystemCmdFloat, SystemCmdInt, PositionWithSpeed):
            cls.__class__.__import_type_support__()
            if cls.__class__._TYPE_SUPPORT is None:
                raise RuntimeError('Missing message type support: ' + cls.__name__)
        print('ROS_IMPORT_OK (feedback publishers still require on-vehicle verification)')
    elif sys.argv[1] in ('gpu', 'cpu'):
        import hashlib
        import numpy as np
        import torch
        from sly_lane_following.neural import NeuralMask, ENET_V3_SHA256
        checkpoint = ROOT / 'models/enet_v3.pth'
        if hashlib.sha256(checkpoint.read_bytes()).hexdigest() != ENET_V3_SHA256:
            raise RuntimeError('CHECKPOINT_HASH_MISMATCH: restore models/enet_v3.pth')
        print('Torch:', torch.__version__, 'Path:', torch.__file__, 'CUDA runtime:', torch.version.cuda, flush=True)
        device = 'cuda' if sys.argv[1] == 'gpu' else 'cpu'
        torch.set_num_threads(1)
        if device == 'cuda' and not torch.cuda.is_available():
            raise RuntimeError('CUDA_UNAVAILABLE')
        if device == 'cuda':
            print('GPU:', torch.cuda.get_device_name(0), flush=True)
        model = NeuralMask(checkpoint, device=device)
        output = model.probability(np.zeros((480, 768, 3), dtype=np.uint8))
        if device == 'cuda':
            torch.cuda.synchronize()
        if output.shape != (480, 768) or not np.isfinite(output).all():
            raise RuntimeError('ENET_INVALID_GPU_OUTPUT')
        print(sys.argv[1].upper() + '_FORWARD_OK (one inference only; not a latency or driving acceptance test)')
    else:
        raise ValueError('Expected ros, gpu or cpu')


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print('E4 preflight failed: ' + type(exc).__name__ + ': ' + str(exc), file=sys.stderr)
        sys.exit(1)
