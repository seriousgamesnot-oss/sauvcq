# SAUVC autonomy: handoff

## Architecture
BT (this package) -> one merged `sauvc_interfaces/ControlSetpoint` (position, depth, yaw, mode) -> PID (controls team).
Inputs: EKF `/odometry/filtered`, `/imu/data`, gate detections, optional Bar30 / floor clearance / battery.
Nodes never publish directly; they write to `SharedState`, and the executor publishes one setpoint per tick (`tickOnce`, 20 Hz).

## Mission phases
INITIAL_DIVE -> FORWARD_TRAVERSAL -> UTURN -> REVERSE_TRAVERSAL -> RETURN_TO_START -> ARRIVED_AT_START (HOLD_POSITION) -> COMPLETE.
Monitors run in parallel: depth watchdog, floor clearance (latched hold + climb), battery, sensor health (EKF lost > 2 s aborts to hold).

## Run
- Simulation: `ros2 launch sauvc_autonomy sauvc_sim.launch.py` or `~/BehaviourTrees_ws/run_sim_test.sh`
- Real: set values with `~/BehaviourTrees_ws/set_config.sh POOL MINCLR SAFEDEPTH [BAR30] [FLOOR] [BATTERY]`, then `ros2 launch sauvc_autonomy sauvc_bt_live.launch.py`
- The executor refuses to start until pool_depth_m, min_floor_clearance_m, floor_safe_depth_m are set.

## Assumptions to confirm (config/bt_config.yaml)
- `GateDetection.position` is in the vehicle body frame (x forward, y left); no TF yet; bbox-only detections unsupported.
- EKF z-up (depth = -z), yaw CCW positive.
- Start pose = EKF pose at mission start; arrival radius is horizontal only.
- Battery thresholds and critical action are unconfirmed; monitor is off until battery_topic is set.
- Floor: stale/invalid bottom-camera data counts as a violation (can stall the mission: safe side).

## Not filled on the blackboard
vn100_accel / vn100_gyro (no raw topic; `imu_accel`/`imu_gyro` come from /imu/data), target_pose, controller_status (UNKNOWN).

## Known limits
Monitors stop when the tree ends; bearing-based alignment (no gate orientation); old mock_* scripts are placeholders (use mock_vehicle_sim).
