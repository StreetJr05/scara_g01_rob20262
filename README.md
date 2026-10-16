# RRP SCARA: point-to-point operation (Reporte 3)

ESP32 firmware (ESP-IDF + micro-ROS) plus ROS 2 Jazzy high-level software for an RRP SCARA. The prismatic axis is a binary solenoid, and the end effector is an electrovalve.

```
firmware/                 ESP-IDF project (C)
  main/scara_config.h     <- ALL pins, geometry, encoder, homing and PID defaults
  main/protocol.h         topics / commands / state layout (mirror: scara_control/protocol.py)
  main/control.c          state machine: homing, jog + deadman, point-to-point, P/valve sequence
  main/kinematics.c       FK / closed-form IK (2 solutions, limits, singularities)
  main/pid.c              discrete PID (anti-windup, filtered D, feed-forward, friction comp.)
  main/trajectory.c       synchronized quintic joint trajectory
  main/hal.c              MC33926 PWM (LEDC 20 kHz 10 bit), PCNT encoders x4, switches, outputs
  main/uros_node.c        micro-ROS node over USB serial with auto-reconnect
  components/micro_ros_espidf_component   (git submodule, branch jazzy)
ros2_ws/src/
  scara_description/      xacro (base, 2 arms, P), RViz, display launch
  scara_control/          bridge, teach pendant, goto CLI, characterization + analysis
docs/                     Notion-ready write-ups (IK, ground truth), in Spanish
```

## Hardware

| Function | Joint 1 | Joint 2 |
|---|---|---|
| MC33926 INV (direction) | GPIO 26 | GPIO 33 |
| MC33926 D2 (PWM 20 kHz, 10 bit) | GPIO 25 | GPIO 32 |
| Encoder A / B | GPIO 35 / 34 | GPIO 39 / 36 |
| Right / left limit switch (to GND, active LOW) | GPIO 27 / 13 | GPIO 19 / 18 |
| **Solenoid (P)**: 1 = energized = P **UP** | GPIO 16 | |
| **Electrovalve**: 1 = ON | GPIO 17 | |

GPIO 16 and 17 must drive a MOSFET or relay, never the coils directly; use a flyback diode on each coil. If your relay module is active-LOW, set `SOLENOID_ACTIVE_LEVEL` / `VALVE_ACTIVE_LEVEL` to 0.

P/valve logic: the solenoid is energized at boot so the P starts UP. Before any XY motion (homing or point-to-point) the P is raised. When a target has z "down", or when you press D on the pendant, the solenoid is released, the firmware waits `p_down_ms`, and then turns the **valve ON**. The valve turns OFF **only** on an explicit command (F key, or `goto --cmd valve_off`).

## 1. Build and flash the firmware

```bash
cd ~/scara_rrp && git submodule update --init      # first time only, on a fresh clone
idf                                                 # your alias: clean ESP-IDF shell (no ROS sourced!)
cd ~/scara_rrp/firmware
idf.py set-target esp32                             # first time only
idf.py build
idf.py -p /dev/ttyUSB0 flash                        # close the micro-ROS agent first (same port)
```

The first build also compiles micro-ROS, which takes several minutes. After changing `app-colcon.meta`, run `idf.py clean-microros`. UART0 belongs to micro-ROS, so `idf.py monitor` shows binary garbage. Use the `/scara/event` topic for logs instead.

## 2. Build the ROS 2 workspace

```bash
cd ~/scara_rrp/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 3. Run

```bash
# terminal 1: agent + bridge (sends PID params) + robot_state_publisher + RViz
ros2 launch scara_control scara.launch.py dev:=/dev/ttyUSB0
# terminal 2: keyboard teach pendant
ros2 run scara_control teach_pendant
# terminal 3: point-to-point targets in mm
ros2 run scara_control goto 185 40 65
ros2 run scara_control goto 185 40 25           # z low -> P down -> valve ON
ros2 run scara_control goto --cmd valve_off
ros2 run scara_control goto -i                  # interactive
ros2 run scara_control goto --file src/scara_control/config/demo_points.yaml
```

Press the ESP32's EN/reset button if it was already running before the agent started. It also reconnects by itself.

### Teach pendant keys (the window must have focus)

| Key | Action |
|---|---|
| **hold SPACE** | dead man's switch. Releasing it stops the motors at once (also on focus loss or pendant crash: the firmware times out after 250 ms) |
| Q / A | joint 1 + / − (with SPACE held) |
| W / S | joint 2 + / − |
| E / D | P up / P down (with SPACE held; down → valve ON) |
| R / F | valve ON / OFF |
| + / − | jog speed |
| H | HOME |
| X or Esc | STOP |
| C | reset fault |
| T | teach (store current pose); then *Go to selected* / *Save YAML* |

### Tuning the PID live

Edit `ros2_ws/src/scara_control/config/scara_params.yaml`, or change a value while running:

```bash
ros2 param set /scara_bridge j1_kp 5.0
ros2 param set /scara_bridge j2_ki 2.0
ros2 topic echo /scara/state      # see the ST_* layout in protocol.py (u1/u2 = PWM, q_ref vs q)
```

`rqt_plot /scara/state/data[2] /scara/state/data[4]` plots q1 against q1_ref, which is useful for tuning.

## 4. First power-up checklist (before trusting point-to-point)

1. **Measure and set** every `TODO(measure)` in `firmware/main/scara_config.h`: `Jx_COUNTS_PER_REV`, `L1_M`, `L2_M`, `TOOL_Z_*`, soft limits, which switch is at the + end (`Jx_RIGHT_IS_POSITIVE`), home switch angles, and home pose. Mirror L1, L2, the heights and the stroke in `scara.urdf.xacro`.
2. Lift the arm or keep a hand on the power switch. Hold SPACE and press Q. The pendant must show q1 **increasing**, and the arm must turn **counter-clockwise** seen from above.
   - Arm turns CW → flip `J1_MOTOR_INVERT`.
   - Angle decreases while turning CCW → flip `J1_ENC_INVERT`.
   - A mismatch between the two shows up as a "following error" FAULT, which is the intended safety behaviour.
3. Press each limit switch by hand and check the pendant's `Limits` row.
4. Press H (home). If the joint drives away from the switch, flip `Jx_HOME_TOWARD_POSITIVE` or `Jx_RIGHT_IS_POSITIVE`.
5. Tune the PID (start with Ki = Kd = 0, raise Kp until it oscillates, take about 60 % of that, then add Ki and Kd). Then raise `vmax`/`amax`.
6. Run `goto` on a few points and compare them against a ruler.

`goto --cmd force_homed` marks the current pose as the home pose without moving. It's only for bench tests with no limit switches.

## 5. Characterization (accuracy and precision)

```bash
ros2 run scara_control characterize --config src/scara_control/config/characterization.yaml --out run1.csv
ros2 run scara_control analyze run1.csv                 # ground truth (meas_x/y columns)
ros2 run scara_control analyze run1.csv --source enc    # encoder-only (internal repeatability)
```

This runs 5 points × 10 cycles in ISO 9283 style and reports AP (accuracy), mean error and RP (precision), as tables and PNG plots. See `docs/ground_truth.md` for the proposed ground-truth method.

## Topics

| Topic | Type | Direction |
|---|---|---|
| `/scara/cmd_pose` | geometry_msgs/Point [m] | PC → ESP32 |
| `/scara/jog` | geometry_msgs/Vector3 (x: J1, y: J2 in [−1, 1]) | PC → ESP32 |
| `/scara/deadman` | std_msgs/Bool (≥ 10 Hz heartbeat) | PC → ESP32 |
| `/scara/cmd` | std_msgs/Int32 (`CMD_*`) | PC → ESP32 |
| `/scara/params` | std_msgs/Float32MultiArray [id, value, …] | PC → ESP32 |
| `/scara/state` | std_msgs/Float32MultiArray (`ST_*`), 20 Hz | ESP32 → PC |
| `/scara/event` | std_msgs/String (IK flow, faults) | ESP32 → PC |
| `/joint_states`, `/scara/tool_pose`, `/scara/target_marker` | from `scara_bridge` | PC |
