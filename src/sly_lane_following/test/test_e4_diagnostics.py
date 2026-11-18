"""Read-only diagnostic classification, process containment, and entry behavior."""
import importlib.util
from pathlib import Path
import sys
import pytest

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('e4_diagnose', ROOT/'scripts/e4_diagnose.py')
diag = importlib.util.module_from_spec(spec)
spec.loader.exec_module(diag)


@pytest.mark.parametrize('pubs,count,fresh,valid,expected', [
    (0,0,False,False,'NO_PUBLISHER'),
    (1,0,False,False,'PUBLISHER_WITHOUT_RECEIVED_DATA'),
    (1,2,False,True,'STALE_OR_UNSTAMPED_DATA'),
    (1,2,True,False,'INVALID_FEEDBACK'),
    (1,2,True,True,'FRESH_FEEDBACK_OBSERVED'),
])
def test_feedback_classification(pubs,count,fresh,valid,expected):
    assert diag.feedback_classification(pubs,count,fresh,valid)==expected


@pytest.mark.parametrize('stderr,expected', [
    ('CHECKPOINT_HASH_MISMATCH','WEIGHT_FAILURE'),
    ('ENET_METADATA_HASH_MISMATCH','WEIGHT_FAILURE'),
    ('ENET_METADATA_REQUIRED','WEIGHT_FAILURE'),
    ('libcublasLt.so.12: cannot open shared object file','DYNAMIC_LIBRARY_MISSING'),
    ('CUDA_UNAVAILABLE','CUDA_UNAVAILABLE_CHECK_DRIVER_REPORT'),
    ("No module named 'torch'",'PYTHON_DEPENDENCY_MISSING'),
    ('TIMEOUT','FORWARD_FAILED_OR_TIMEOUT'),
])
def test_inference_classification(stderr,expected):
    assert diag.inference_classification(dict(returncode=1,stdout='',stderr=stderr))==expected


def test_native_failure_is_contained():
    r=diag.run([sys.executable,'-c','import os; os._exit(134)'])
    assert r['returncode']==134


def test_bounded_child_and_missing_executable():
    assert diag.run([sys.executable,'-c','import time; time.sleep(5)'],.05)['stderr']=='TIMEOUT'
    assert diag.run(['/nonexistent/sly-program'])['returncode'] is None
