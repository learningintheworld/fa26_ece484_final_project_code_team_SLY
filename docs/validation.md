# Validation record and acceptance cases

## Verified in this starter

On 2026-10-19, the host command below passed **17 tests** with Python's standard
library unittest runner:

```bash
PYTHONPATH=src/sly_autonomy python3 -m unittest discover -s tests -v
```

The tests check straight tracking, known circular geometry, left/right symmetry,
steering saturation, curvature speed reduction, duplicate points, bad geometry
and parameter rejection. Safety tests check disabled startup, each missing or
stale stream, invalid/future data, measured-stop dwell, interrupted dwell,
same-sign suppression and rearming, obstacle priority, manual-release latching,
clock rewind and preservation of pending stops across disable.

This is logic verification, not evidence of trained perception, reliable vehicle
stopping, or closed-loop course completion.

## Verified ROS adapter smoke test

On 2026-10-19 America/Chicago, copied this repository into
`/tmp/sly_code_validation` inside the existing `ece484-ece484-1` container.
No coursework or simulator source was modified. `colcon build --symlink-install
--packages-select sly_autonomy` completed successfully (one package).

With the built setup sourced, ran:

```bash
ROS_DOMAIN_ID=84 python3 tests/ros_smoke.py
```

The isolated test domain received a synthetic clock, straight local path,
odometry and perception status. **All eight scenario groups passed**; the final
run observed 169 command messages over a clock ending at 9.5 s:

| Check | Observed result |
|---|---|
| Fresh inputs at startup | `DISABLED`, zero command |
| Default command isolation | No publisher on `/ackermann_cmd` |
| Explicit enable | `CRUISE`, positive speed ≤ 1 m/s, angle within ±0.45 rad |
| Perception stream removed | `FAULT_STOP`, zero command; fresh data alone did not restart |
| Manual takeover then release | `MANUAL` then `DISABLED`; explicit enable required |
| Explicit disable | Remained disabled despite fresh inputs |
| Path in `map` instead of local frame | Latched `FAULT_STOP` |
| Sign trigger, measured stop, obstacle hold | `SIGN_STOP` → `SIGN_DWELL`; obstacle prevented release after dwell; continuing same-sign signal did not retrigger |

The installed launch/config/entry point also started successfully with:

```bash
ROS_DOMAIN_ID=84 timeout --foreground --signal=INT 3s \
  ros2 launch sly_autonomy sim.launch.py
```

The controller reported disabled startup and finished cleanly after the
forwarded SIGINT. The timeout wrapper returns 124 as expected. This check found
and fixed an attempted publish after the ROS context was already shut down.
All test-started controller processes were terminated after validation.

**These were synthetic ROS message tests, not a Gazebo drive or a hardware
demonstration.** Odometry was supplied by the test driver; it did not measure a
moving vehicle. The perception producer was a test fixture, not a detector.

## Next integration checks

1. Connect verified GEM odometry and a real local-path producer in a simulator;
   verify frame origins, steering sign, command exclusivity and QoS.
2. Repeat message-level fault injection for each source using the actual producer
   rates, including delayed packets, invalid detections and simulation resets.
3. Measure actual stopping trajectories and tracking errors in Gazebo. A zero
   target speed must not be equated with measured instantaneous stopping.
4. Add camera-derived paths and measured range/occupancy before calling the system
   a visual closed loop. Run hardware tests only after the separate setup process.

## Project acceptance, still planned

| Scenario | Record |
|---|---|
| Straight, bends, shadows, one missing boundary | Lane departures, cross-track error, confidence, interventions |
| Sign approach/stop/restart and next lap | Detection latency, stopping location, dwell, duplicate events |
| Obstacle inside/outside corridor | Clearance, stop success, false stops, recovery after clearance |
| Data timeout or invalid perception/path | Detection-to-stop latency, latched fault, explicit recovery |
| Two complete laps | All attempts, success rate, elapsed time, lane departures, collisions, interventions |

Internal final target: repeat the full two-lap run at least three times and
report every attempt. Vehicle speed, stop offset and other physical thresholds
remain to be agreed with staff and measured on the assigned vehicle. Do not
claim the synthetic interface checks satisfy these criteria.
