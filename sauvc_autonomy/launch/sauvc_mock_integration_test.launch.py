"""
sauvc_mock_integration_test.launch.py
──────────────────────────────────────
ROS 2 launch file for cross-departmental mock integration testing.

Architecture:
  ┌─────────────────────────────────────────────────────────┐
  │  mock_perception_node   → /varuna/camera/front/compressed│
  │                           /varuna/gate_pose              │
  ├─────────────────────────────────────────────────────────┤
  │  mock_navigation_node   → /varuna/odom                   │
  │                           /sauvc/move_to_pose/_action    │
  ├─────────────────────────────────────────────────────────┤
  │  mock_controls_node     → /varuna/depth                  │
  │                           /varuna/imu                    │
  │  (subscribes: /varuna/cmd_vel, /varuna/heave_cmd)        │
  ├─────────────────────────────────────────────────────────┤
  │  sauvc_bt_executor      ← all of the above              │
  │  (the BT under test)                                     │
  └─────────────────────────────────────────────────────────┘

Usage:
  ros2 launch sauvc_autonomy sauvc_mock_integration_test.launch.py \\
      scenario:=forward_only    # or: full_run | uturn_only | depth_fault

Scenarios:
  full_run      : Complete qualification (forward + U-turn + reverse). Default.
  forward_only  : Gate visible, traversal succeeds, mission stops after 1 pass.
  uturn_only    : Mock starts AUV already past gate; tests U-turn subtree only.
  depth_fault   : Depth mock injects an out-of-bounds event after 10 s to test
                  EmergencySurface trigger.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    TimerAction,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory("sauvc_autonomy")
    bt_xml    = os.path.join(pkg_share, "bt_trees", "SAUVC_Qualification_Main.xml")

    # ── launch arguments ─────────────────────────────────────────────
    scenario_arg = DeclareLaunchArgument(
        "scenario",
        default_value="full_run",
        description="Mock scenario to run: full_run | forward_only | uturn_only | depth_fault",
        choices=["full_run", "forward_only", "uturn_only", "depth_fault"],
    )

    tick_rate_arg = DeclareLaunchArgument(
        "tick_rate_hz", default_value="20.0",
        description="BT tick rate in Hz"
    )

    op_depth_arg = DeclareLaunchArgument(
        "operational_depth", default_value="1.0",
        description="Target operational depth in metres"
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level", default_value="info",
        choices=["debug", "info", "warn", "error"],
    )

    # ── mock perception node ──────────────────────────────────────────
    # Publishes synthetic gate detections on a timer.
    # Parameters control the scenario behaviour.
    mock_perception = Node(
        package="sauvc_autonomy",
        executable="mock_perception_node",
        name="mock_perception",
        output="screen",
        parameters=[{
            "scenario":             LaunchConfiguration("scenario"),
            # Gate appears after 3 s; simulates AUV swimming toward gate
            "gate_visible_after_sec":     3.0,
            # Gate disappears for 2 s mid-approach to test YawSweepSearch recovery
            "gate_hidden_from_sec":      10.0,
            "gate_hidden_until_sec":     12.0,
            # Published gate pose (metres from AUV start, in AUV frame)
            "gate_initial_distance_m":    4.0,
            "gate_lateral_offset_m":      0.15,   # slight offset to exercise alignment
            "gate_yaw_offset_deg":        5.0,
            # Detection confidence (randomised Gaussian noise)
            "detection_confidence_mean":  0.82,
            "detection_confidence_std":   0.08,
            # Topics
            "output_camera_topic":       "/varuna/camera/front/compressed",
            "output_gate_pose_topic":    "/varuna/gate_pose",
        }],
        arguments=["--ros-args", "--log-level",
                   ["mock_perception:=", LaunchConfiguration("log_level")]],
    )

    # ── mock navigation node ──────────────────────────────────────────
    # Simulates odometry by integrating cmd_vel commands.
    # Also provides a fake MoveToWaypoint action server that completes
    # after a simulated travel time proportional to distance.
    mock_navigation = Node(
        package="sauvc_autonomy",
        executable="mock_navigation_node",
        name="mock_navigation",
        output="screen",
        parameters=[{
            "scenario":                LaunchConfiguration("scenario"),
            # Simulated vehicle kinematics
            "surge_max_m_s":           0.5,
            "yaw_rate_max_deg_s":     25.0,
            # Odometry publish rate
            "odom_rate_hz":           20.0,
            # Topics / actions
            "cmd_vel_topic":          "/varuna/cmd_vel",
            "odom_topic":             "/varuna/odom",
            "move_to_pose_action":    "/sauvc/move_to_pose",
            # Vehicle geometry (must match BT executor parameter)
            "vehicle_length_m":        0.65,
        }],
        arguments=["--ros-args", "--log-level",
                   ["mock_navigation:=", LaunchConfiguration("log_level")]],
    )

    # ── mock controls node ────────────────────────────────────────────
    # Simulates depth sensor + IMU and responds to heave commands.
    # In the "depth_fault" scenario, injects an out-of-bounds event.
    mock_controls = Node(
        package="sauvc_autonomy",
        executable="mock_controls_node",
        name="mock_controls",
        output="screen",
        parameters=[{
            "scenario":              LaunchConfiguration("scenario"),
            # Starting state
            "initial_depth_m":       0.0,      # starts at surface, dives on command
            "initial_yaw_deg":       0.0,
            # Simulated dynamics
            "heave_response_tau_s":  1.2,      # first-order lag for depth response
            "yaw_response_tau_s":    0.6,
            # Noise
            "depth_noise_std_m":     0.02,
            "imu_yaw_noise_std_deg": 0.5,
            # Depth fault injection (only in depth_fault scenario)
            "fault_inject_at_sec":  10.0,
            "fault_depth_m":         2.8,      # exceeds max_safe_depth → triggers abort
            "fault_duration_sec":    2.0,
            # Topics
            "heave_cmd_topic":       "/varuna/heave_cmd",
            "cmd_vel_topic":         "/varuna/cmd_vel",
            "depth_topic":           "/varuna/depth",
            "imu_topic":             "/varuna/imu",
            # Publish rates
            "depth_rate_hz":         20.0,
            "imu_rate_hz":           50.0,
        }],
        arguments=["--ros-args", "--log-level",
                   ["mock_controls:=", LaunchConfiguration("log_level")]],
    )

    # ── BT executor (the unit under test) ────────────────────────────
    bt_executor = Node(
        package="sauvc_autonomy",
        executable="sauvc_bt_executor",
        name="sauvc_bt_executor",
        output="screen",
        parameters=[{
            "bt_xml_path":        bt_xml,
            "tick_rate_hz":       LaunchConfiguration("tick_rate_hz"),
            "operational_depth":  LaunchConfiguration("operational_depth"),
            "min_safe_depth":     0.35,
            "max_safe_depth":     2.40,
            "vehicle_length_m":   0.65,
        }],
        arguments=["--ros-args", "--log-level",
                   ["sauvc_bt_executor:=", LaunchConfiguration("log_level")]],
    )

    # ── state transition validator ────────────────────────────────────
    # Lightweight node that subscribes to /varuna/mission_phase (a String
    # published by ScriptNode via the BT) and asserts the correct sequence:
    #   INIT → FORWARD_TRAVERSAL → UTURN → REVERSE_TRAVERSAL → COMPLETE
    # Prints a PASS/FAIL summary and exits when mission_phase == "COMPLETE".
    state_validator = Node(
        package="sauvc_autonomy",
        executable="mock_state_validator",
        name="state_validator",
        output="screen",
        parameters=[{
            "expected_phase_sequence": [
                "INIT",
                "FORWARD_TRAVERSAL",
                "UTURN",
                "REVERSE_TRAVERSAL",
                "COMPLETE",
            ],
            "timeout_sec": 120.0,
            "mission_phase_topic": "/varuna/mission_phase",
        }],
    )

    # ── optional: RQt / Groot2 live monitoring ────────────────────────
    # Uncomment to launch Groot2 and connect it to the BT publisher port.
    # groot2 = ExecuteProcess(
    #   cmd=["Groot2", "--mode", "monitor"],
    #   output="screen"
    # )

    # ── launch info ───────────────────────────────────────────────────
    launch_info = LogInfo(msg=[
        "\n",
        "╔══════════════════════════════════════════════════════╗\n",
        "║   SAUVC Mock Integration Test                        ║\n",
        "║   Scenario : ", LaunchConfiguration("scenario"),      " ║\n",
        "║   BT XML   : ", bt_xml,                               " ║\n",
        "╚══════════════════════════════════════════════════════╝\n",
    ])

    return LaunchDescription([
        # Arguments
        scenario_arg,
        tick_rate_arg,
        op_depth_arg,
        log_level_arg,
        # Info
        launch_info,
        # Mock nodes start first — give them 1 s before BT executor
        mock_perception,
        mock_navigation,
        mock_controls,
        # BT executor starts 1 second after mocks are up
        TimerAction(
            period=1.0,
            actions=[bt_executor],
        ),
        # Validator starts alongside BT
        TimerAction(
            period=1.1,
            actions=[state_validator],
        ),
    ])
