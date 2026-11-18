"""Optional MP1 SimpleENet mask backend; downstream lane selection is shared.

Input: undistorted BGR. Aspect-preserving 640x384 grayscale letterbox. Only
model probabilities are resized back; no lane extrapolation/post-fill occurs.
Torch is imported only when this optional backend is selected.
"""
from pathlib import Path
import hashlib
import io
import json
import pickle
import warnings
import cv2
import numpy as np


def letterbox(image, label=None):
    h, w = image.shape[:2]
    if h < 1 or w < 1:
        raise ValueError('EMPTY_IMAGE')
    scale = min(640 / w, 384 / h)
    nw, nh = int(round(w * scale)), int(round(h * scale))
    x, y = (640 - nw) // 2, (384 - nh) // 2
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    canvas = np.zeros((384, 640), np.float32)
    canvas[y:y+nh, x:x+nw] = cv2.resize(gray, (nw, nh), interpolation=cv2.INTER_AREA) / 255.
    target = None
    if label is not None:
        target = np.full((384, 640), 255, np.uint8)
        target[y:y+nh, x:x+nw] = cv2.resize(label, (nw, nh), interpolation=cv2.INTER_NEAREST)
    return canvas, target, (x, y, nw, nh, w, h)


def restore_probability(probability, mapping):
    x, y, nw, nh, w, h = mapping
    return cv2.resize(probability[y:y+nh, x:x+nw], (w, h), interpolation=cv2.INTER_LINEAR)


# Only this published, reviewed checkpoint may use the legacy pickle loader.
ENET_V3_SHA256 = '5b6b3c12d4ba1609a6107f6241a3f42f6ce9a8b7112fcacb436be0960452a547'


def load_checkpoint(torch, checkpoint):
    data = Path(checkpoint).expanduser().read_bytes()
    try:
        return torch.load(io.BytesIO(data), map_location='cpu', weights_only=True)
    except pickle.UnpicklingError as error:
        # PyTorch 1.13's restricted loader cannot read BINFLOAT metadata.
        if ('Unsupported operand 71' not in str(error)
                or hashlib.sha256(data).hexdigest() != ENET_V3_SHA256):
            raise
        warnings.warn('Using legacy loader for verified ENet v3 checkpoint only', RuntimeWarning)
        return torch.load(io.BytesIO(data), map_location='cpu', weights_only=False)


class NeuralMask:
    def __init__(self, checkpoint, device='cuda'):
        import torch
        from .simple_enet import create_model
        self.torch = torch
        self.device = torch.device(device)
        if self.device.type == 'cuda' and not torch.cuda.is_available():
            raise RuntimeError('ENET_CUDA_UNAVAILABLE: select cpu explicitly for offline use')
        state = load_checkpoint(torch, checkpoint)
        sidecar = Path(checkpoint).expanduser().with_suffix('.json')
        metadata = state
        architecture = 'simple_enet_v1'
        if not sidecar.exists() and 'state_dict' not in state:
            raise ValueError('ENET_METADATA_REQUIRED: keep the matching JSON beside tensor weights')
        if sidecar.exists():
            metadata = json.loads(sidecar.read_text())
            if metadata.get('schema') != 'sly_enet_tensor_checkpoint_v1':
                raise ValueError('ENET_METADATA_SCHEMA_MISMATCH')
            if metadata.get('sha256') != hashlib.sha256(Path(checkpoint).expanduser().read_bytes()).hexdigest():
                raise ValueError('ENET_METADATA_HASH_MISMATCH')
            if metadata.get('preprocessing') != 'gray_letterbox_640x384_v1':
                raise ValueError('ENET_PREPROCESSING_MISMATCH')
            architecture = metadata['architecture']
            if not all(isinstance(v, torch.Tensor) for v in state.values()):
                raise ValueError('ENET_EXPECTED_TENSOR_STATE')
        self.model = create_model(architecture).to(self.device)
        self.inference_roi_top = metadata.get('inference_roi_top', 0.) or 0.
        if not isinstance(self.inference_roi_top, (int, float)) or not 0 <= self.inference_roi_top < 1:
            raise ValueError('ENET_INVALID_INFERENCE_ROI')
        if 'state_dict' in state:
            if state.get('preprocessing') != 'gray_letterbox_640x384_v1':
                raise ValueError('ENET_PREPROCESSING_MISMATCH')
            state = state['state_dict']
        self.model.load_state_dict(state, strict=True)
        self.model.eval()

    def probability(self, image):
        tensor, _, mapping = letterbox(image)
        with self.torch.inference_mode():
            x = self.torch.from_numpy(tensor[None, None]).to(self.device)
            p = self.model(x).softmax(dim=1)[0, 1].cpu().numpy()
        if not np.isfinite(p).all():
            raise ValueError('ENET_NONFINITE_OUTPUT')
        return restore_probability(p, mapping)

    def __call__(self, image):
        mask = (self.probability(image) >= .5).astype(np.uint8) * 255
        # The checkpoint declares its supported lane-output region. A caller
        # cannot silently enable rows excluded during dataset validation.
        mask[:int(np.ceil(image.shape[0] * self.inference_roi_top))] = 0
        return mask
