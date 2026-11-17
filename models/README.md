# ENet v3 deployment checkpoint

`enet_v3.pth` is the unchanged, locally fine-tuned MP1 SimpleENet checkpoint used in the real-RGB and Highbay evaluations. Includes preprocessing/ROI metadata and weights; no recorded images.

- Source: `results/enet_real_v3/training/best.pth` (local training artifact).
- SHA-256: `5b6b3c12d4ba1609a6107f6241a3f42f6ce9a8b7112fcacb436be0960452a547`.
- Input: grayscale letterbox 640×384, derived from undistorted RGB; use the existing `neural.py` preprocessing.
- Start commands from the repository root, as the two ENet YAML configurations use `models/enet_v3.pth`.
- Requires compatible PyTorch; default inference uses CUDA. Simulation success is not physical-vehicle validation.
