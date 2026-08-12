# seer_agv_driver

ROS 2 Python driver for the SEER/Robokit AMB-300 AGV TCP API in the unified
`${DOBOT_WS:-$HOME/dobot_ws}` robot workspace.

Only `seer_agv_node` owns the robot TCP ports.  The integrated PyQt operator
GUI uses ROS topics and services and must not construct a `SeerClient`.

The API numbers were verified against the complete protocol documents supplied
with the original project handoff. Those vendor documents are not required at
runtime and should only be redistributed with permission.

Implemented APIs (all listed calls are documentation-checked; live-test status is
recorded in the handoff package):

- Status port `19204`: `1000`, `1004`, `1005`, `1006`, `1007`, `1012`,
  `1020`, `1021`, `1022`, `1050`, `1060`, `1101`, `1300`, `1301`, `1500`
- Control port `19205`: `2000` stop, `2002` relocalize, `2003` confirm
  localization, `2004` cancel relocalization, `2010` open-loop motion,
  `2022` switch loaded map
- Navigation port `19206`: `3003` cancel navigation, `3051` station navigation
- Config port `19207`: `4011` download map

Deliberately not implemented yet:

- `3053` path preview and `3066` specified-path navigation
- arbitrary-coordinate/custom-path execution
- map upload/delete APIs

## Run

```bash
colcon build --symlink-install --packages-select seer_agv_msgs seer_agv_driver
source install/setup.bash
ros2 launch seer_agv_driver seer_agv.launch.py host:=192.168.192.5
```

One-click startup after reboot:

```bash
export DOBOT_WS="${HOME}/dobot_ws"
"${DOBOT_WS}/start_dobot_operator_gui.sh"
```

The script checks only the read-only status port `19204`, reuses an existing
`/seer_agv_node` when present, otherwise starts one with the safety-gated
`/seer_agv/cmd_vel` input enabled, and launches the integrated GUI. It does not
send non-zero motion by itself. On exit it requests verified `2000` stop before
terminating a driver that it started.

Published topics:

- `/seer_agv/pose` (`geometry_msgs/PoseStamped`)
- `/seer_agv/odom` (`nav_msgs/Odometry`, SEER map-localization pose)
- `/seer_agv/status` (`std_msgs/String`, JSON payload)
- `/seer_agv/map_markers` (`visualization_msgs/MarkerArray`)
- `/seer_agv/map_data` (`std_msgs/String`, transient-local compact JSON for the GUI)
- `/seer_agv/footprint_markers` (`visualization_msgs/MarkerArray`, live AGV footprint)

RViz:

```bash
rviz2 -d install/seer_agv_driver/share/seer_agv_driver/config/seer_agv.rviz
```

Start the driver before RViz so the node publishes
`seer_map -> seer_base_link`.  The dedicated names avoid collisions with CR5
links and any future wheel odometry.

The driver downloads the robot model with verified status API `1500 -> 11500` and parses the chassis rectangle footprint from the model. The AMB-300 footprint is drawn as `width=0.7 m`, `head=0.52 m`, `tail=0.48 m`. RViz shows the same rectangle at every station and at the current AGV pose.

The packaged RViz config leaves the old Pose/Odometry arrow displays disabled by default, because the Odometry display accumulates arrows over time. Use `/seer_agv/footprint_markers` for the live AGV body display.

The old standalone `seer_agv_panel` is intentionally not installed because it
opened a second TCP connection set. Use the `SEER AGV` tab in
`dobot_operator_gui`.

Services:

- `/seer_agv/stop` (`std_srvs/Trigger`) sends verified `2000`
- `/seer_agv/cancel_nav` (`std_srvs/Trigger`) sends verified `3003`
- `/seer_agv/download_map` (`std_srvs/Trigger`) downloads current map with verified `4011` and republishes markers
- `/seer_agv/load_map` (`seer_agv_msgs/LoadMap`) switches to an existing map with
  verified `2022`, then downloads it with `4011`
- `/seer_agv/relocalize` (`seer_agv_msgs/Relocalize`) starts relocation with
  verified `2002`; it never confirms the result automatically
- `/seer_agv/cancel_relocalization` (`std_srvs/Trigger`) sends verified `2004`
- `/seer_agv/navigate_to_station` (`seer_agv_msgs/NavigateToStation`) sends
  verified `3051` only after the common safety gate passes

Map switching and relocation require a fresh stationary status and explicit
`operator_confirmed=true`.  Switching a map invalidates localization.  After
relocation reaches `reloc_status=3`, the operator must inspect the displayed
pose and explicitly use `/seer_agv/confirm_localization` before motion is
enabled.

## Safe Teleop

Motion is off by default. Enable only after read-only status is healthy and the area is clear:

```bash
ros2 launch seer_agv_driver seer_agv.launch.py enable_cmd_vel:=true
```

The node clamps `/seer_agv/cmd_vel` to:

- `|vx| <= 0.1 m/s`
- `vy = 0` for the verified `rbk_diff` differential chassis
- `|w| <= 0.2 rad/s`
- single `2010` duration <= `motion_duration_ms`, default `300 ms`

The watchdog sends verified `2000` stop when `/seer_agv/cmd_vel` times out.
Motion is inhibited for unconfirmed localization, missing map, emergency stop,
charging, control ownership, blocked/slowed state, alarms, or stale status.

For the tested SRC/Robokit `v3.4.4.6`, `reloc_status=3` means relocation has
completed but an operator has not confirmed it. Only `reloc_status=1` is
accepted. The driver never sends API `2003` automatically.

The operator GUI exposes a guarded `Confirm localization` action backed by
`/seer_agv/confirm_localization`.  The request includes the pose snapshot shown
to the operator.  The driver sends a verified `2000` stop and only then sends
`2003` when status is fresh, relocation is exactly `3`, the map is loaded, the
AGV is stationary, and no other safety gate is active.  API `2003` is never
sent at startup or without an explicit operator confirmation.

Keyboard teleop publishes `/seer_agv/cmd_vel`; keep `seer_agv_node` running
with `enable_cmd_vel:=true` in another terminal:

```bash
ros2 run seer_agv_driver seer_keyboard_teleop
```

Use arrow keys to move, space to stop, and `q` to quit. Hold an arrow key for continuous motion; if no key repeat is received for `key_timeout` seconds, the teleop node publishes zero velocity.

Speeds can be overridden:

```bash
ros2 run seer_agv_driver seer_keyboard_teleop --ros-args \
  -p linear_speed:=0.03 \
  -p forward_speed:=0.03 \
  -p backward_speed:=0.03 \
  -p angular_speed:=0.10 \
  -p key_timeout:=0.15
```
