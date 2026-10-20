# Simulation interfaces and control contract

`sly_sim_controller` runs at 20 Hz in ROS simulation time. Every restart begins
disabled. The pure Python modules expose `pure_pursuit(points, ControlConfig)`
and `SafetySupervisor(SafetyConfig)` independently of ROS.

## Topics and service

| Interface | Type | Meaning |
|---|---|---|
| `/sly/local_path` input | `nav_msgs/msg/Path` | Ordered local path in the rear-axle reference frame |
| `/sly/odom` input | `nav_msgs/msg/Odometry` | Measured planar speed from `twist.twist.linear.x/y` |
| `/sly/perception` input | `diagnostic_msgs/msg/DiagnosticArray` | Stamped validity, obstacle occupancy, stop-required event |
| `/sly/manual_takeover` input | `std_msgs/msg/Bool` | `true` immediately latches disabled; `false` only releases takeover |
| `/sly/sim/ackermann_cmd` output | `ackermann_msgs/msg/AckermannDrive` | Front-wheel angle in radians and speed in m/s; isolated by default |
| `/sly/state` output | `diagnostic_msgs/msg/DiagnosticArray` | State, enabled flag, target speed and stop reason |
| `/sly/enable` service | `std_srvs/srv/SetBool` | Explicit enable/disable; enable requires valid fresh inputs |

Path, odometry and perception subscriptions use ROS's sensor-data QoS
(best effort, volatile). The manual-event and state interfaces use reliable
volatile QoS. Check the actual producers' QoS before integration. State and
command outputs contain no durability guarantee after process termination.

## Coordinates, times and validity

- `Path.header.frame_id` must equal configurable `local_frame` (`base_link` by
  default). This name must represent **x forward, y left, at the rear axle**.
  Verify the GEM model's reference origin or transform the path first; the node
  does not implement TF. Pose headers may be empty or use the same frame.
- Supply at least two points with a nonzero segment, trimmed to x ≥ 0 and ordered
  in travel direction. The producer owns lane confidence, waypoint ordering,
  path horizon and progress; this controller does not select a closest point
  on an entire closed track. Non-finite, wholly behind, or unusably short paths
  are rejected. The 2 m lookahead is accumulated along the supplied path.
- Header timestamps must be acquisition times in the `/clock` domain, greater
  than zero. Zero, missing, more than 0.5 s old, or more than 0.05 s in the future
  triggers rejection. Receipt time is never substituted for source time.
- All three stamped streams are required even for a straight path. Perception
  must explicitly report valid; absence of an obstacle message is not “clear”.
- Stop control requests **zero target speed immediately**. The simulator's
  resulting deceleration depends on its dynamics. No physical stopping-distance
  guarantee follows from this command. Positive speed ramps up at 0.3 m/s²;
  stop requests bypass the acceleration ramp.

## Perception status format

The message header uses `local_frame` and the timestamp described above. Exactly
one `DiagnosticStatus` must have `name: sly/perception`, `level: 0` (`OK`), and
the following unique key/value pairs. Values are the literal strings `true` or
`false`. Missing, duplicated or malformed fields invalidate the observation.

```yaml
status:
  - name: sly/perception
    level: 0
    message: "upstream perception result"
    values:
      - {key: valid, value: 'true'}
      - {key: obstacle_present, value: 'false'}
      - {key: stop_sign_stop_required, value: 'false'}
```

`valid=true` must eventually mean that all required image/range inputs and the
upstream detector are valid. `obstacle_present` describes occupancy of the
planned driving corridor, including unclassified obstacles. The stop-sign bit
means **the stopping trigger has been reached**, not merely that a distant sign
was detected. Distance estimation and the calibrated trigger are not implemented
here. Never publish permanent all-clear data and label it working perception.
Synthetic status may be used only for an explicitly labeled adapter test.

## State transitions

| State | Motion | Recovery |
|---|---|---|
| `DISABLED` | Zero speed | Explicit successful enable |
| `MANUAL` | Zero speed | Release takeover, then explicit enable |
| `FAULT_STOP` | Zero speed | Repair inputs, then explicit enable |
| `OBSTACLE_STOP` | Zero speed | Fresh clear corridor; sign stop still applies |
| `SIGN_STOP` | Zero speed | Measured speed reaches standstill threshold |
| `SIGN_DWELL` | Zero speed | Continuous measured standstill for configured dwell |
| `CRUISE` | Low-speed Pure Pursuit | Any stop condition takes priority |

Fault/manual/disable conditions outrank movement. An obstacle holds the vehicle
even if a sign's dwell has finished. A sign event is latched if the detection
disappears before stopping. Dwell begins at measured speed ≤ 0.05 m/s, restarts
if motion resumes, and lasts 3 s by default. Manual takeover or a fault clears
elapsed dwell but preserves a pending stop. After serving a sign, a continuing
true signal cannot retrigger it; a fresh false signal lasting 2 s rearms it.
This time-based deduplication is a **starter heuristic**. Stable sign IDs and
lap-aware rearming are future perception/integration work.

The supervisor must be ticked periodically; a separate actuator-side watchdog
for process death is not implemented. ROS clock rewind latches a fault. A paused
simulation clock pauses the node and should only be used in simulation.
