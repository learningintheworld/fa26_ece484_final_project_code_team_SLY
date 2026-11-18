"""Night candidates are isolated from v3 and cannot bypass label approval."""
import importlib.util
import json
from pathlib import Path
import cv2
import numpy as np
import pytest
import torch
from sly_lane_following.night_augmentation import sample_parameters, apply_night, make_training_view, sequence_parameters, headlight_field
from sly_lane_following.simple_enet import create_model, initialize_from_mp1, freeze_pretrained_batchnorm
from sly_lane_following.neural import NeuralMask


def training_module():
    path=Path(__file__).resolve().parents[3]/'scripts/train_night_lane_net.py'
    spec=importlib.util.spec_from_file_location('train_night',path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def image_label():
    im=np.full((96,160,3),100,np.uint8);label=np.zeros((96,160),np.uint8)
    cv2.line(im,(30,95),(70,30),(235,235,235),4);cv2.line(label,(30,95),(70,30),1,4)
    return im,label


def test_augmentation_is_deterministic_without_moving_labels():
    im,label=image_label();before=im.copy();lb=label.copy();p=sample_parameters(np.random.default_rng(1),'medium')
    a=apply_night(im,p,12);b=apply_night(im,p,12)
    assert np.array_equal(a,b) and np.array_equal(im,before) and np.array_equal(label,lb)
    assert a.dtype==np.uint8 and a.shape==im.shape and a.mean()<im.mean()
    assert a[label==1].mean()>a[label==0].mean()


def test_light_generator_never_receives_labels_and_rejects_bad_input():
    import inspect
    assert 'label' not in inspect.signature(apply_night).parameters
    with pytest.raises(ValueError):apply_night(np.zeros((2,2),np.uint8),{},0)
    with pytest.raises(ValueError):sample_parameters(np.random.default_rng(1),'unknown')


def test_temporal_light_is_continuous_at_knot():
    a=sequence_parameters(1.999999);b=sequence_parameters(2.000001)
    for k in a:
        if isinstance(a[k],float):assert abs(a[k]-b[k])<1e-5


def test_visible_training_view():
    im,label=image_label()
    for level in ('light','medium','strong'):
        result,meta=make_training_view(im,label,level,484)
        assert result.shape==im.shape and meta['visibility']['visible_fraction']>=.55


def test_expansion_preserves_starting_function_and_counts():
    torch.set_num_threads(2);torch.manual_seed(1)
    base=create_model().eval();expanded=create_model('simple_enet_night_v1').eval()
    initialize_from_mp1(expanded,base.state_dict())
    assert sum(p.numel() for p in base.parameters())==348996
    assert sum(p.numel() for p in expanded.parameters())==429282
    x=torch.randn(1,1,64,96)
    with torch.no_grad():assert torch.equal(base(x),expanded(x))
    expanded.train();freeze_pretrained_batchnorm(expanded)
    assert not expanded.initial_block.batch_norm.training
    assert expanded.extra64[0].block.ext_conv1[1].training
    assert any(n.startswith('extra') and p.requires_grad for n,p in expanded.named_parameters())


def test_pretrained_key_mismatch_not_silently_ignored():
    m=create_model('simple_enet_night_v1')
    with pytest.raises(ValueError,match='KEYS_MISMATCH'):initialize_from_mp1(m,{'wrong':torch.zeros(1)})
    with pytest.raises(ValueError,match='UNKNOWN_ARCHITECTURE'):create_model('unknown')


def test_new_weight_sidecar_hash_and_prediction(tmp_path):
    module=training_module();model=create_model('simple_enet_night_v1').eval()
    path=tmp_path/'candidate.pth';module.save_weights(model,path,'simple_enet_night_v1',275/480)
    loaded=NeuralMask(path,'cpu')
    for k,v in model.state_dict().items():assert torch.equal(v,loaded.model.state_dict()[k])
    meta=json.loads(path.with_suffix('.json').read_text());meta['sha256']='wrong';path.with_suffix('.json').write_text(json.dumps(meta))
    with pytest.raises(ValueError,match='HASH_MISMATCH'):NeuralMask(path,'cpu')


def test_training_requires_explicit_review_before_any_work(tmp_path):
    p=tmp_path/'manifest.json';p.write_text(json.dumps(dict(frozen=False,training_authorized=False,frames=[])))
    with pytest.raises(ValueError,match='REVIEW_REQUIRED'):training_module().validate_manifest(p)


def test_exact_epoch_mix(tmp_path):
    p=tmp_path/'manifest.json';p.write_text(json.dumps(dict(frames=[dict(split='train') for _ in range(350)])))
    ds=training_module().NightDataset(p,'train')
    from collections import Counter
    assert len(ds)==700 and Counter(ds.severity)==Counter(light=175,medium=140,strong=35)
    old=list(ds.severity);ds.set_epoch(1);assert ds.severity!=old


def test_two_headlight_lobes_with_dark_far_field():
    p=sample_parameters(np.random.default_rng(484),'medium')
    p.update(cx=.5,cy=.82,separation=.44,sx=.16,sy=.24,vertical_offset=0.,asymmetry=0.)
    field=headlight_field((480,768),p);row=field[round(.82*479)]
    l=int(np.argmax(row[:384]));r=384+int(np.argmax(row[384:]))
    assert .2 < l/767 < .4 and .6 < r/767 < .8
    assert row[384] < .85*min(row[l],row[r])
    assert field[:120].mean() < .08*field[320:].mean()
    assert np.all(np.isfinite(field)) and field.min() >= 0


def test_severity_reduces_ambient_without_losing_two_lamps():
    params=[sample_parameters(np.random.default_rng(484),s) for s in ('light','medium','strong')]
    assert params[0]['ambient']>params[1]['ambient']>params[2]['ambient']
    assert params[0]['gamma']<params[1]['gamma']<params[2]['gamma']
    for p in params:
        assert p['separation'] > 2*p['sx']


def test_tensor_candidate_requires_metadata(tmp_path):
    model=create_model().eval();path=tmp_path/'candidate.pth'
    torch.save(model.state_dict(),path)
    with pytest.raises(ValueError,match='METADATA_REQUIRED'):NeuralMask(path,'cpu')
