"""Deployment guards; mock ROS commands never contact vehicle hardware."""
import os
import pickle
import subprocess
from pathlib import Path
import pytest
from sly_lane_following.neural import load_checkpoint

ROOT = Path(__file__).resolve().parents[3]


class LegacyTorch:
    def __init__(self, fail='Unsupported operand 71'):
        self.calls = []
        self.fail = fail

    def load(self, stream, **kwargs):
        self.calls.append((stream.read(), kwargs['weights_only']))
        if kwargs['weights_only']:
            raise pickle.UnpicklingError(self.fail)
        return {'legacy': True}


def test_known_checkpoint_only_legacy_fallback():
    torch = LegacyTorch()
    with pytest.warns(RuntimeWarning):
        assert load_checkpoint(torch, ROOT / 'models/enet_v3.pth') == {'legacy': True}
    assert [safe for _, safe in torch.calls] == [True, False]
    assert torch.calls[0][0] == torch.calls[1][0]


@pytest.mark.parametrize('contents', [b'unknown checkpoint', b'corrupted'])
def test_untrusted_checkpoint_never_uses_pickle(tmp_path, contents):
    p = tmp_path / 'model.pth'; p.write_bytes(contents)
    torch = LegacyTorch()
    with pytest.raises(pickle.UnpicklingError):
        load_checkpoint(torch, p)
    assert len(torch.calls) == 1 and torch.calls[0][1]


def test_other_loader_failure_never_retried():
    torch = LegacyTorch('unsupported unrelated object')
    with pytest.raises(pickle.UnpicklingError):
        load_checkpoint(torch, ROOT / 'models/enet_v3.pth')
    assert len(torch.calls) == 1


@pytest.fixture
def entry(tmp_path):
    root = tmp_path / 'alternate clone'; (root / 'scripts').mkdir(parents=True)
    (root / 'install').mkdir()
    (root / 'src/sly_lane_following/config').mkdir(parents=True)
    (root / 'src/sly_lane_following/config/real_enet.yaml').write_text('enet_device: cuda\nenet_checkpoint: models/enet_v3.pth\n')
    (root / 'src/sly_lane_following/config/real_enet_night.yaml').write_text('enet_device: cuda\nenet_checkpoint: models/enet_night_expanded.pth\n')
    log = tmp_path / 'calls'
    ros = tmp_path / 'humble.bash'
    ros.write_text('export ROS_DISTRO=humble\necho humble >> "$TEST_LOG"\n')
    driver = tmp_path / 'driver'; (driver / 'install').mkdir(parents=True)
    (driver / 'install/setup.bash').write_text('echo driver >> "$TEST_LOG"\n')
    (root / 'install/local_setup.bash').write_text('echo project >> "$TEST_LOG"\n')
    bindir = tmp_path / 'bin'; bindir.mkdir()
    commands = {
        'ros2': 'echo "ros2 $*" >> "$TEST_LOG"; exit "${ROS_FAIL:-0}"',
        'python-selected': '''echo "python $*" >> "$TEST_LOG"
for arg in "$@"; do
  if [[ "$arg" == config:=* ]]; then cat "${arg#config:=}" >> "$TEST_LOG"; fi
done
if [[ "$*" == *'e4_check.py ros'* ]]; then exit "${PY_ROS_FAIL:-0}"; fi
if [[ "$*" == *'e4_check.py gpu'* ]]; then exit "${GPU_FAIL:-0}"; fi
''',
        'colcon': 'echo "build $*" >> "$TEST_LOG"',
    }
    for name, body in commands.items():
        p = bindir / name; p.write_text('#!/bin/bash\n' + body + '\n'); p.chmod(0o755)
    script = (ROOT / 'scripts/e4.sh').read_text().replace('/opt/ros/humble/setup.bash', str(ros)).replace('/usr/bin/colcon', str(bindir / 'colcon'))
    (root / 'scripts/e4.sh').write_text(script)
    env = dict(os.environ, PATH=str(bindir)+':'+os.environ['PATH'], SLY_PYTHON=str(bindir/'python-selected'), SLY_DRIVER_WS=str(driver), TEST_LOG=str(log))
    def run(action, **extra):
        log.write_text('')
        proc = subprocess.run(['bash', str(root/'scripts/e4.sh'), action], cwd='/tmp', env=dict(env, **extra), text=True, capture_output=True)
        return proc, log.read_text()
    return run


def test_setup_order_and_no_old_project_overlay(entry):
    p, calls = entry('setup')
    assert p.returncode == 0, p.stderr
    assert calls.startswith('humble\ndriver\n') and '\nproject\n' not in calls
    assert calls.index('e4_check.py ros') < calls.index('e4_check.py gpu') < calls.index('build ')


@pytest.mark.parametrize('failure', [{'ROS_FAIL':'1'}, {'PY_ROS_FAIL':'1'}, {'GPU_FAIL':'1'}, {'GPU_FAIL':'134'}, {'SLY_DRIVER_WS':'/nonexistent_sly_driver'}])
def test_failed_setup_does_not_build(entry, failure):
    p, calls = entry('setup', **failure)
    assert p.returncode != 0 and 'build ' not in calls


def test_gpu_failure_prevents_perception(entry):
    p, calls = entry('vision', GPU_FAIL='134')
    assert p.returncode != 0 and '-m sly_lane_following.node' not in calls


def test_cpu_is_explicit_and_adapter_calls_main(entry):
    p, calls = entry('vision-cpu')
    assert p.returncode == 0 and 'e4_check.py gpu' not in calls
    assert '-m sly_lane_following.node' in calls
    p, calls = entry('observe')
    assert p.returncode == 0 and 'from sly_lane_following.adapter import main; main()' in calls
    assert 'dry_run:=true' in calls


def test_build_does_not_require_gpu(entry):
    p, calls = entry('build', GPU_FAIL='134')
    assert p.returncode == 0 and 'e4_check.py' not in calls and '\nproject\n' not in calls


def test_gpu_preflight_failure_modes(monkeypatch):
    import importlib.util
    import sys
    from types import SimpleNamespace
    import sly_lane_following.neural as neural
    spec = importlib.util.spec_from_file_location('e4_check', ROOT / 'scripts/e4_check.py')
    check = importlib.util.module_from_spec(spec); spec.loader.exec_module(check)
    monkeypatch.setattr(sys, 'version_info', (3, 10, 12))
    monkeypatch.setattr(sys, 'argv', ['e4_check.py', 'gpu'])
    cuda = SimpleNamespace(is_available=lambda: False, get_device_name=lambda _: 'mock')
    fake = SimpleNamespace(cuda=cuda, __version__='mock', __file__='mock/torch', set_num_threads=lambda _:None, version=SimpleNamespace(cuda='mock'))
    monkeypatch.setitem(sys.modules, 'torch', fake)
    with pytest.raises(RuntimeError, match='CUDA_UNAVAILABLE'):
        check.main()
    cuda.is_available = lambda: True
    def broken_model(*args, **kwargs):
        raise OSError('missing CUDA library')
    monkeypatch.setattr(neural, 'NeuralMask', broken_model)
    with pytest.raises(OSError, match='missing CUDA library'):
        check.main()


def test_diagnostics_survive_missing_driver_and_do_not_start_project(entry):
    p, calls = entry('diagnose', SLY_DRIVER_WS='/nonexistent_sly_driver')
    assert p.returncode == 0 and 'e4_diagnose.py' in calls
    assert 'build ' not in calls and '\nproject\n' not in calls


def test_real_old_torch_rejects_unverified_legacy_checkpoint(tmp_path):
    torch = pytest.importorskip('torch')
    if not torch.__version__.startswith('1.13.'):
        pytest.skip('Actual legacy-loader compatibility test requires Torch 1.13')
    p=tmp_path/'unknown.pth'
    torch.save({'metadata': 0.5}, p)
    with pytest.raises(pickle.UnpicklingError):
        load_checkpoint(torch,p)
    p.write_bytes(b'corrupted checkpoint')
    with pytest.raises(Exception):
        load_checkpoint(torch,p)


def test_night_checks_selected_weights_and_only_changes_vision_profile(entry):
    p, calls = entry('vision-night')
    assert p.returncode == 0 and 'e4_check.py gpu night' in calls
    assert 'enet_checkpoint: models/enet_night_expanded.pth' in calls
    assert 'enet_device: cuda' in calls and 'e4_pure_pursuit.yaml' in calls
    assert 'ENet model: night' in p.stdout
    assert 'lane_adapter' not in calls and 'service call' not in calls
    p, calls = entry('vision')
    assert p.returncode == 0 and 'e4_check.py gpu v3' in calls
    assert 'enet_checkpoint: models/enet_v3.pth' in calls


def test_night_failure_cannot_fall_back_or_start_vision(entry):
    p, calls = entry('vision-night', GPU_FAIL='134')
    assert p.returncode != 0 and '-m sly_lane_following.node' not in calls
    assert 'e4_check.py gpu v3' not in calls
    p, calls = entry('vision-night-cpu')
    assert p.returncode == 0 and 'e4_check.py gpu' not in calls
    assert 'enet_device: cpu' in calls and 'enet_night_expanded.pth' in calls


def test_night_diagnostic_selection_and_no_project_start(entry):
    p, calls = entry('diagnose-night')
    assert p.returncode == 0 and 'e4_diagnose.py --model night' in calls
    assert 'build ' not in calls and '-m sly_lane_following.node' not in calls
