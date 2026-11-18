"""Read-only vehicle preflight; never publishes actuator commands."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/sly_lane_following'))


MODEL_PROFILES = {
    'v3': ('models/enet_v3.pth', '5b6b3c12d4ba1609a6107f6241a3f42f6ce9a8b7112fcacb436be0960452a547'),
    'night': ('models/enet_night_expanded.pth', '425b89429f1ae098edca12c9f8c507e20dd643586d10310716dbd39ab367abf3'),
}


def selected_checkpoint(model='v3'):
    if model not in MODEL_PROFILES:
        raise ValueError('UNKNOWN_MODEL: choose v3 or night')
    relative, expected = MODEL_PROFILES[model]
    return ROOT / relative, expected


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
        from sly_lane_following.neural import NeuralMask
        model_name = sys.argv[2] if len(sys.argv) > 2 else 'v3'
        checkpoint, expected = selected_checkpoint(model_name)
        if hashlib.sha256(checkpoint.read_bytes()).hexdigest() != expected:
            raise RuntimeError('CHECKPOINT_HASH_MISMATCH: restore ' + str(checkpoint))
        print('Model:', model_name, 'Checkpoint:', checkpoint, 'SHA256:', expected, flush=True)
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
        raise ValueError('Expected ros, or gpu/cpu [v3|night]')


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print('E4 preflight failed: ' + type(exc).__name__ + ': ' + str(exc), file=sys.stderr)
        sys.exit(1)
