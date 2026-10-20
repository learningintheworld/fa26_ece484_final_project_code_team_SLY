# Team SLY milestones

The immediate deliverable is the English pitch website and a five-minute
presentation. The code repository supplies an inspectable foundation and test
plan; the final-project system is not presented as completed.

## Proposed ownership — team confirmation pending

| Member | Primary responsibility | Pitch responsibility |
|---|---|---|
| xinweiy3 | Tracking, platform adapter, integration | Simulator baseline, controller approach, final content review |
| devonlu2 | Camera calibration, lane perception, local path | Website, system diagram, perception explanation |
| xianyis2 | Detection, ranging, stop supervisor, evaluation | Test plan, hardware coordination, risks and timeline |

All three participate in training and supervised vehicle tests. Rotate the
driving/takeover, software-operation and recording responsibilities.

## Sequence

| Window | Exit evidence |
|---|---|
| Pitch submission, Oct 19 | Website sources committed; true baseline video/chart; missing joystick demo disclosed |
| Oct 20–21 | Staff-approved training/vehicle appointment; supervised joystick demo if access permits |
| Oct 22 | Two timed rehearsals and offline website/video backup before the presentation |
| Oct 23–29 | Recorded actual sensor inventory, frame/QoS conventions, calibration and first rosbag |
| Oct 30–Nov 12 | Vision-derived local path tracking, then sign stopping and measured dwell |
| Nov 13–26 | Corridor obstacle stopping, failure injection and repeated integration tests |
| Nov 27–Dec 10 | Repeated two-lap trials, aggregate results and final website/video |
| Dec 11 | Final presentation, subject to the latest course notice |

Midpoint date remains subject to course announcement. Track individual work
and experiment evidence throughout; the pitch plan targets at least 36 total
person-hours after the pitch for the three-person team.

## Implementation queue

1. Verify local path/odometry frames in the simulator; add a real local-path
   producer. A prerecorded global path baseline must be labeled as such.
2. Port and evaluate MP1 segmentation on recorded target-scene images, then
   camera calibration, bird's-eye conversion and lane-center extraction.
3. Evaluate pretrained YOLO11n and range estimates before considering retraining.
   Depth requires alignment with RGB; LiDAR requires validated extrinsics.
4. Replace the synthetic stop-required event with measured range/stop geometry.
   Add stable sign identity, lap-aware rearming and validated clear-corridor logic.
5. Implement the independently reviewed hardware adapter after supervised
   setup; never directly remap simulator output into PACMod.
6. Execute the tests in `validation.md` and archive all attempts and failures.

Cone avoidance, parking, phone dispatch and gesture interaction are future work
after the core lane-follow/stop/resume/two-lap objective. Confirm with staff the
three-person team's obstacle evaluation boundary; do not silently remove general
obstacle stopping from scope.
