"""Optional MP1 SimpleENet mask backend; downstream lane selection is shared.

Input: undistorted BGR. Aspect-preserving 640x384 grayscale letterbox. Only
model probabilities are resized back; no lane extrapolation/post-fill occurs.
Torch is imported only when this optional backend is selected.
"""
from pathlib import Path
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


class NeuralMask:
    def __init__(self, checkpoint, device='cuda'):
        import torch
        from .simple_enet import SimpleENet
        self.torch = torch
        self.device = torch.device(device)
        if self.device.type == 'cuda' and not torch.cuda.is_available():
            raise RuntimeError('ENET_CUDA_UNAVAILABLE: select cpu explicitly for offline use')
        self.model = SimpleENet().to(self.device)
        state = torch.load(Path(checkpoint).expanduser(), map_location='cpu', weights_only=True)
        self.inference_roi_top = state.get('inference_roi_top', 0.) or 0.
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
