"""Read-only, bounded vehicle diagnostics. Never creates command publishers."""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
TOPICS = ('vehicle_speed', 'global', 'steering', 'brake', 'shift')


def run(argv, timeout=30):
    start = time.monotonic()
    try:
        p = subprocess.run(argv, capture_output=True, text=True, timeout=timeout)
        return dict(returncode=p.returncode, stdout=p.stdout, stderr=p.stderr,
                    seconds=round(time.monotonic()-start, 3))
    except subprocess.TimeoutExpired:
        return dict(returncode=None, stdout='', stderr='TIMEOUT', seconds=timeout)
    except OSError as error:
        return dict(returncode=None, stdout='', stderr=str(error), seconds=0)


def inference_classification(result):
    text = result['stdout'] + result['stderr']
    if result['returncode'] == 0:
        return 'FORWARD_OK'
    if 'CHECKPOINT' in text or 'UnpicklingError' in text:
        return 'WEIGHT_FAILURE'
    if 'cannot open shared object' in text or 'Could not load library' in text:
        return 'DYNAMIC_LIBRARY_MISSING'
    if 'CUDA_UNAVAILABLE' in text:
        return 'CUDA_UNAVAILABLE_CHECK_DRIVER_REPORT'
    if 'No module named' in text:
        return 'PYTHON_DEPENDENCY_MISSING'
    return 'FORWARD_FAILED_OR_TIMEOUT'


def feedback_classification(publishers, count, fresh, valid):
    if not publishers:
        return 'NO_PUBLISHER'
    if not count:
        return 'PUBLISHER_WITHOUT_RECEIVED_DATA'
    if not fresh:
        return 'STALE_OR_UNSTAMPED_DATA'
    if not valid:
        return 'INVALID_FEEDBACK'
    return 'FRESH_FEEDBACK_OBSERVED'


def feedback():
    import math
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from pacmod2_msgs.msg import VehicleSpeedRpt, GlobalRpt, SystemRptFloat, SystemRptInt
    rclpy.init()
    node = rclpy.create_node('sly_readonly_diagnostic_' + str(os.getpid()))
    stats = {k: dict(count=0, last=None, ages=[], valid=True) for k in TOPICS}
    def receive(key, msg):
        s = stats[key]
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        age = node.get_clock().now().nanoseconds / 1e9 - stamp
        s['count'] += 1
        s['last'] = time.monotonic()
        s['ages'].append(age)
        if key == 'vehicle_speed':
            s['valid'] = bool(msg.vehicle_speed_valid) and math.isfinite(msg.vehicle_speed)
        elif key == 'global':
            s['valid'] = not any(getattr(msg, k) for k in ('veh_can_timeout', 'str_can_timeout', 'brk_can_timeout', 'usr_can_timeout', 'user_can_read_errors'))
        else:
            s['valid'] = math.isfinite(msg.output)
    subscriptions = []
    for key, typ in zip(TOPICS, (VehicleSpeedRpt, GlobalRpt, SystemRptFloat, SystemRptFloat, SystemRptInt)):
        subscriptions.append(node.create_subscription(typ, '/pacmod/'+key+'_rpt', lambda m, k=key: receive(k, m), qos_profile_sensor_data))
    end = time.monotonic() + 6
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.1)
    report = {}
    for key, s in stats.items():
        topic = '/pacmod/'+key+'_rpt'
        pubs = node.count_publishers(topic)
        fresh = bool(s['ages']) and 0 <= s['ages'][-1] <= .5 and time.monotonic()-s['last'] <= .5
        report[topic] = dict(publishers=pubs, messages=s['count'], latest_source_age_s=s['ages'][-1] if s['ages'] else None,
                            result=feedback_classification(pubs, s['count'], fresh, s['valid']))
    node.destroy_node()
    rclpy.shutdown()
    print(json.dumps(report, indent=2))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--feedback-child', action='store_true')
    args = parser.parse_args()
    if args.feedback_child:
        feedback()
        return
    report = dict(created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                  python=sys.executable, python_version=sys.version.split()[0],
                  context={k: os.environ.get(k) for k in ('ROS_DISTRO', 'ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'SLY_DRIVER_WS')},
                  scope='Read-only; no control enable or actuator commands. Not hardware acceptance.')
    report['source_warnings'] = os.environ.get('SLY_DIAGNOSTIC_SOURCE_WARNINGS', '')
    checkpoint = ROOT / 'models/enet_v3.pth'
    report['weight_sha256'] = hashlib.sha256(checkpoint.read_bytes()).hexdigest() if checkpoint.exists() else 'MISSING'
    report['nvidia_driver'] = run(['nvidia-smi', '--query-gpu=name,driver_version', '--format=csv,noheader'], 10)
    report['torch'] = run([sys.executable, '-c', 'import torch; print(torch.__version__); print(torch.__file__); print(torch.version.cuda); print("cuda_available", torch.cuda.is_available())'])
    for device in ('cpu', 'gpu'):
        result = run([sys.executable, str(ROOT/'scripts/e4_check.py'), device], 90)
        result['result'] = inference_classification(result)
        report[device] = result
    report['pacmod_package'] = run(['ros2', 'pkg', 'prefix', 'pacmod2_msgs'])
    report['pacmod_interface'] = run(['ros2', 'interface', 'show', 'pacmod2_msgs/msg/VehicleSpeedRpt'])
    report['ros_import'] = run([sys.executable, str(ROOT/'scripts/e4_check.py'), 'ros'])
    if report['ros_import']['returncode'] == 0:
        report['feedback'] = run([sys.executable, str(Path(__file__).resolve()), '--feedback-child'], 12)
    else:
        report['feedback'] = {'result': 'ROS_IMPORT_FAILED; feedback cannot be evaluated in this interpreter'}
    out = ROOT/'results/e4_diagnostics'/datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    out.mkdir(parents=True, exist_ok=False)
    path = out/'report.json'
    path.write_text(json.dumps(report, indent=2, ensure_ascii=False))
    print(json.dumps(report, indent=2, ensure_ascii=False))
    print('\nReport:', path)
    print('Diagnostic collection finished; inspect individual results. This does not authorize driving.')


if __name__ == '__main__':
    main()
