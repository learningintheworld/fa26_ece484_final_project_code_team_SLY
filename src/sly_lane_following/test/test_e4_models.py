"""Published model pairs remain loadable independently of local training files."""
import hashlib
import json
from pathlib import Path
import shutil
import numpy as np
import pytest
import torch
import yaml
from sly_lane_following.neural import NeuralMask

ROOT = Path(__file__).resolve().parents[3]


def test_night_profile_changes_only_checkpoint():
    cfg = ROOT / 'src/sly_lane_following/config'
    base = yaml.safe_load((cfg/'real_enet.yaml').read_text())
    night = yaml.safe_load((cfg/'real_enet_night.yaml').read_text())
    assert base.pop('enet_checkpoint') == 'models/enet_v3.pth'
    assert night.pop('enet_checkpoint') == 'models/enet_night_expanded.pth'
    assert base == night


@pytest.mark.parametrize('name,parameters', [('enet_v3', 348996), ('enet_night_expanded', 429282)])
def test_published_model_forward(name, parameters):
    torch.set_num_threads(1)
    net = NeuralMask(ROOT/'models'/f'{name}.pth', device='cpu')
    assert sum(p.numel() for p in net.model.parameters()) == parameters
    assert net.inference_roi_top == 275/480
    result = net(np.zeros((480,768,3), np.uint8))
    assert result.shape == (480,768) and result.dtype == np.uint8
    assert not result[:275].any()


def test_night_requires_matching_sidecar(tmp_path):
    source = ROOT/'models/enet_night_expanded.pth'
    target = tmp_path/source.name
    shutil.copyfile(source, target)
    with pytest.raises(ValueError, match='METADATA_REQUIRED'):
        NeuralMask(target, 'cpu')
    meta = json.loads(source.with_suffix('.json').read_text())
    assert meta['sha256'] == hashlib.sha256(source.read_bytes()).hexdigest()
    meta['sha256'] = 'wrong'
    target.with_suffix('.json').write_text(json.dumps(meta))
    with pytest.raises(ValueError, match='HASH_MISMATCH'):
        NeuralMask(target, 'cpu')
