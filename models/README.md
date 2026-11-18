# ENet v3 deployment checkpoint

`enet_v3.pth` is the unchanged, locally fine-tuned MP1 SimpleENet checkpoint used in the real-RGB and Highbay evaluations. Includes preprocessing/ROI metadata and weights; no recorded images.

- Source: `results/enet_real_v3/training/best.pth` (local training artifact).
- SHA-256: `5b6b3c12d4ba1609a6107f6241a3f42f6ce9a8b7112fcacb436be0960452a547`.
- Input: grayscale letterbox 640×384, derived from undistorted RGB; use the existing `neural.py` preprocessing.
- Start commands from the repository root, as the two ENet YAML configurations use `models/enet_v3.pth`.
- Requires compatible PyTorch; default inference uses CUDA. Simulation success is not physical-vehicle validation.


## Optional synthetic-night candidate

The default remains **v3**. `bash scripts/e4.sh vision-night` selects the candidate; stop the vehicle and the previous vision process before switching. No automatic model selection or driving enable occurs.

- Files: `enet_night_expanded.pth` **and** `enet_night_expanded.json`, stored as ordinary Git files. Keep the pair together; no Git LFS or environment download is required.
- SHA-256: `425b89429f1ae098edca12c9f8c507e20dd643586d10310716dbd39ab367abf3`.
- Architecture: `simple_enet_night_v1`, 429,282 parameters (+23%); same grayscale letterbox 640×384 and output ROI as v3.
- Source: local `results/night_v1/train_expanded/best.pth`; best epoch 30, MP1 epoch20 initialization. 500 assistant-reviewed originals, grouped 350/75/75 split from one dusk capture. Each training epoch uses 350 original and 350 online low-light views. Test images did not select weights; historical v3 training overlaps this new split.
- Augmentation: dark ambient light, two headlamp lobes, gamma, increased/decreased contrast, mild noise/blur, shadows and occasional glare. Inference takes normal camera RGB with no added darkening.
- Actual Torch 1.13.1 CPU loading/forward and laptop GPU inference passed. Vehicle A4000/driver compatibility and latency still require the existing preflight.

| Same-capture test | v3 | Night candidate |
|---|---:|---:|
| Original IoU | 88.60% | 89.48% |
| Original Recall | 97.90% | 95.10% |
| Synthetic light IoU | 79.11% | 89.81% |
| Synthetic medium IoU | 69.15% | 88.91% |

**Not approved as a default replacement:** original Recall drops 2.80 percentage points; medium-light background false positives rise slightly. The unchanged offline handoff preview also regresses around112.94s on original images (validity85.5%→73.0%), while synthetic dynamic-low-light validity improves78.6%→85.6%. These previews use hypothetical motion, not measured control feedback or vehicle trajectories. Labels are assistant-reviewed, not independent dense human ground truth. No real-night images were available.

The candidate is provided at the user's request for selectable vehicle observation. Preserve v3 as the fallback; do not equate synthetic results with nighttime driving readiness. `vision-night-cpu` is observation-only in purpose; `diagnose-night` checks the selected candidate, including its metadata. See [vehicle commands](../E4_QUICKSTART_ZH.md) and [AI handoff](../E4_AI_HANDOFF_ZH.md).
