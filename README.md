# RRP SCARA: point-to-point operation (Reporte 3)

ESP32 firmware (ESP-IDF 5.4 + micro-ROS) plus ROS 2 Jazzy high-level software for an RRP SCARA. Joints 1 and 2 are DC gearmotors with quadrature encoders; the prismatic axis (P) is a binary solenoid, and the end effector is an electrovalve.

```
firmware/                     ESP-IDF project (C)
  main/scara_config.h         <- ALL pins, geometry, encoders, coupling, limits, homing, PID defaults
  main/protocol.h             topics / commands / state layout (mirror: scara_control/protocol.py)
  main/control.c              state machine: homing, jog + deadman, point-to-point, P/valve sequence
  main/kinematics.c           FK / closed-form IK (2 solutions, limits, singularities)
  main/pid.c                  discrete PID (anti-windup, filtered D, feed-forward, friction comp.)
  main/trajectory.c           synchronized quintic joint trajectory
  main/hal.c                  MC33926 PWM (LEDC 20 kHz 10 bit), PCNT encoders x4, switches, outputs
  main/uros_node.c            micro-ROS node over USB serial with auto-reconnect
  sdkconfig.defaults          console off, UART0 pins for micro-ROS (see Troubleshooting)
  components/micro_ros_espidf_component   (git submodule, branch jazzy)
ros2_ws/src/
  scara_description/          xacro + CAD meshes (meshes/*.stl, mm), RViz, display launch
  scara_control/              bridge, teach pendant, goto CLI, tune, characterize + analyze
docs/                         Notion-ready write-ups (IK, ground truth), in Spanish
```

## The robot as built

All of these live in `firmware/main/scara_config.h`; the geometry and limits are mirrored in `ros2_ws/src/scara_description/urdf/scara.urdf.xacro`.

| Item | Value | Notes |
|---|---|---|
| Motors | Pololu 37D 50:1 with 64 CPR encoder (both joints) | |
| Transmission | 20 T → 100 T belt (5:1) | |
| Encoder resolution | 64 × 50 × 5 = **16000 counts / joint rev** (0.0225°) | x4 decoding |
| L1 / L2 | **112.11 mm / 90.00 mm** | axis to axis |
| Link heights | link 1 bottom 144.6 mm, link 2 bottom 171.6 mm | above the table |
| J1 / J2 coupling | **q2 = θ2 − 0.2 · q1** | J2 motor is on the base, see below |
| Joint limits (soft) | J1 −81° … +80°, J2 −64° … +67.5° | 5° inside the switches |
| Limit switches | J1 −86.2° / +84.9°, J2 −68.9° / +72.7° | left switch = + end on both joints |
| Home pose | q1 = 0°, q2 = 0° (arm stretched) | |
| Tool tip height P up / down | 65 / 25 mm | **placeholders**, still to be measured |

Angle convention (firmware, IK and URDF): seen from above, positive = counter-clockwise; **q1 = 0** is link 1 along the base +X axis; **q2 = 0** is link 2 in line with link 1.

**Workspace.** J2 only bends ±65°, so the tool reaches a ring **≈ 170–202 mm** from the J1 axis, over about ±80° around +X (a bit more to the sides depending on the elbow). Targets outside it are rejected with the reason (`outside workspace`, `joint limits`). Known-good points: `185 40`, `120 145`, `180 -60`, `190 0` (mm).

### J1 → J2 coupling

The J2 motor sits on the base and drives a pulley that turns freely on the J1 axis (20 T); a belt along link 1 turns link 2 (100 T). Turning link 1 alone therefore also bends the elbow by −0.2 × the J1 motion. The firmware handles it:

- the encoders give motor-side angles θ; the joint angles are `q1 = θ1`, `q2 = θ2 − J2_COUPLING · q1` (`J2_COUPLING = 20/100`),
- the PID runs on the motor angles, with references converted the other way (`θ2_ref = q2_ref + 0.2 · q1_ref`), so the J2 motor follows J1 automatically,
- re-referencing J1 at its home switch shifts J2's encoder so that q2 keeps its value.

Everything above the firmware (IK, limits, pendant, RViz, `/scara/state`) sees true joint angles. A robot with the J2 motor on link 1 would use `J2_COUPLING = 0`.

## Hardware

| Function | Joint 1 | Joint 2 |
|---|---|---|
| MC33926 INV (direction) | GPIO 26 | GPIO 33 |
| MC33926 D2 (PWM 20 kHz, 10 bit) | GPIO 25 | GPIO 32 |
| Encoder A / B | GPIO 35 / 34 | GPIO 39 / 36 |
| Right / left limit switch (to GND, active LOW) | GPIO 27 / 13 | GPIO 19 / 18 |
| **Solenoid (P)**: 1 = energized = P **UP** | GPIO 16 | |
| **Electrovalve**: 1 = ON | GPIO 17 | |

GPIO 16 and 17 must drive a MOSFET or relay, never the coils directly; use a flyback diode on each coil. If your relay module is active-LOW, set `SOLENOID_ACTIVE_LEVEL` / `VALVE_ACTIVE_LEVEL` to 0. Tie the grounds of the ESP32, the motor drivers and the coil supply together at one point, and keep motor wires away from the USB cable (switching noise can drop the USB link).

P/valve logic: the solenoid is energized at boot so the P starts UP. Before any XY motion (homing or point-to-point) the P is raised. When a target's z is below the middle of the P-up and P-down heights (45 mm with the current values), or when you press D on the pendant, the solenoid is released, the firmware waits `p_down_ms`, and then turns the **valve ON**. The valve turns OFF **only** on an explicit command (F key, or `goto --cmd valve_off`).

## 1. First-time setup

**ESP-IDF shell.** The firmware must be built in a shell **without ROS** (ROS changes `PYTHONPATH`/CMake paths and breaks the build). The `idf` alias in `~/.bashrc` opens one:

```bash
alias idf='env -i HOME="$HOME" USER="$USER" TERM="$TERM" PATH=/usr/local/bin:/usr/bin:/bin bash --rcfile ~/esp/idf_rc'
# ~/esp/idf_rc contains:
#   . "$HOME/esp/esp-idf/export.sh"
#   PS1="(idf) \w\$ "
```

Type `idf` on its own and wait for the `(idf)` prompt before typing anything else (lines pasted after it are lost). `exit` returns to the normal shell.

**Firmware:**

```bash
cd ~/scara_rrp && git submodule update --init      # fresh clone only
idf
cd ~/scara_rrp/firmware
idf.py set-target esp32                             # fresh clone only
idf.py build                                        # first build also compiles micro-ROS (several minutes)
idf.py -p /dev/ttyUSB0 flash
```

`sdkconfig` is generated from `sdkconfig.defaults` and is not in git. After changing `app-colcon.meta`, run `idf.py clean-microros`. UART0 belongs to micro-ROS, so `idf.py monitor` shows binary garbage; use the pendant log (`/scara/event`) instead.

**ROS 2 workspace:**

```bash
cd ~/scara_rrp/ros2_ws
colcon build --symlink-install
```

Find the ESP32 port with `ls /dev/ttyUSB* /dev/ttyACM*` (unplug/replug to see which one appears). Your user must be in the `dialout` group.

## 2. Daily use

Plug in the ESP32 with **motor power off**. Use separate terminals:

| Terminal | Runs | When |
|---|---|---|
| **A** firmware | `(idf)` shell: build + flash | only after changing `firmware/` |
| **B** ROS | agent + bridge + RViz | always, keep running |
| **C** pendant | teach pendant window | always, keep running |
| **D** commands | `goto`, `tune`, `ros2 param set`, … | as needed |

**Terminal B**

```bash
cd ~/scara_rrp/ros2_ws
source install/setup.bash
ros2 launch scara_control scara.launch.py dev:=/dev/ttyUSB0
```

Then press **EN** on the ESP32 and wait for `session established`. RViz shows the arm once `/scara/state` arrives (links lying at the origin = the ESP32 is not talking yet).

**Terminal C**

```bash
cd ~/scara_rrp/ros2_ws && source install/setup.bash
ros2 run scara_control teach_pendant
```

Check that it says **ESP32: online**, switch motor power on, click an empty area of the pendant window and press **H** to home.

**Terminal D**

```bash
cd ~/scara_rrp/ros2_ws && source install/setup.bash
ros2 run scara_control goto 185 40 65               # x y z in mm (base frame)
ros2 run scara_control goto 185 40 25               # z < 45 -> P down -> valve ON
ros2 run scara_control goto --cmd valve_off
ros2 run scara_control goto -i                      # interactive
ros2 run scara_control goto --file src/scara_control/config/demo_points.yaml
```

The pendant's *Point to point* box does the same: type x, y, z in mm and press GO or Enter.

**After changing firmware:** Ctrl+C in **B** (it holds the serial port) → `idf.py build` and `idf.py -p /dev/ttyUSB0 flash` in **A** → restart **B** (↑, Enter) and press EN. **After changing ROS code or config:** `colcon build --symlink-install` in B before launching.

**To finish:** motor power off, then Ctrl+C in C and B.

### Teach pendant keys (the window must have focus)

Click an empty grey area first: keys typed while the cursor is in the x/y/z boxes go into the box.

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

**After a FAULT** every key except C is ignored. Press **C**, then jog away from any pressed switch (the key toward a pressed switch is blocked on purpose).

## 3. Homing and calibration

### What HOME does

1. Raise the P.
2. **Joint 2 first**, then joint 1: drive closed-loop at 15°/s toward the home switch (the − end, `Jx_HOME_TOWARD_POSITIVE = 0`). When it closes, the encoder is set to `Jx_HOME_SWITCH_ANGLE`; then back off 5°.
3. Move both joints to the home pose (0°, 0°) and report `HOME done`.

Only one switch per joint is used for homing; the other end's switch is a safety stop. The encoders are incremental, so **before homing** the angles are only relative to the power-up position: soft limits are off and `goto` is rejected.

Homing safety checks (they fault instead of pushing into a hard stop): the joint hits its *other* switch (wrong direction / switch side), the joint moves less than 3° in 1.5 s (no power, stalled, encoder dead), or the switch is not found in 20 s.

After every reset the firmware logs `BOOT: reset = <reason>` in the pendant (power-on, PANIC, BROWNOUT, …) together with what the control loop was doing, which makes crashes and supply dips visible.

### Recalibrating the zero

If the home pose does not look like 0°/0° (link 1 along your 0° line, links in line):

1. Home with **H** (it must end with `HOME done`, not a reset).
2. With SPACE held, jog until link 1 is exactly on the 0° line and link 2 exactly in line with link 1.
3. Read q1, q2 on the pendant. New `Jx_HOME_SWITCH_ANGLE = old value − reading`. Reflash.

The readings are only valid on a properly homed robot: after a reset the encoders restart from wherever the arm stopped.

### Soft limits

Jog slowly to each switch after homing and read the angle when it clicks (the pendant's `Limits` row shows `#`). Set `Jx_MIN_RAD` / `Jx_MAX_RAD` about 5° inside, and the same values in the xacro (`j1_min` … `j2_max`). Jog limits only stop motion further out, so a joint beyond a limit can always be jogged back.

### Bring-up checks (new hardware or after rewiring)

1. Press each limit switch by hand and check the `Limits` row.
2. Hold SPACE and press Q: seen from above the arm must turn **counter-clockwise** and q1 must **increase** (same for W / q2).
   - It turns the wrong way **and** the angle still increases → flip both `Jx_MOTOR_INVERT` and `Jx_ENC_INVERT`.
   - Motor and encoder disagree → "following error" FAULT; flip only one of them.
3. Home, then check the zero and the limits as above.

`goto --cmd force_homed` marks the current pose as the home pose without moving. It's only for bench tests with no limit switches.

## 4. Tuning the controller

Gains are runtime parameters (`ros2_ws/src/scara_control/config/scara_params.yaml`), pushed to the ESP32 by the bridge. Change them live, no reflashing:

```bash
ros2 param set /scara_bridge j1_kp 3.0
ros2 param set /scara_bridge j1_kff 0.25
```

Live changes are lost when the launch restarts: copy the final values into `scara_params.yaml`.

Measure with the step-test tool (robot homed, keep clear of the arm):

```bash
ros2 run scara_control tune --pair j1 --out 5_kp8_j1    # shoulder swings 40 deg, elbow fixed
ros2 run scara_control tune --pair j2 --out 5_kp8_j2    # elbow moves ~37 deg
ros2 run scara_control tune --pair both                 # both joints
```

It prints per move and joint: **lag** behind the reference, **over**shoot, **settle** time into ±0.5°, **final** error, **sat%** (PWM at its limit) and **osc** (ringing), plus an estimate of the duty needed per rad/s as a starting value for `jX_kff`. Results go to `results/tuning/<out>/` at the project root, wherever you run it from: `response.png` plots reference vs. position, error and PWM, and `state.csv` has the raw data (kept locally, `*.csv` is not committed). `results/tuning/README.md` summarizes the runs that led to the current gains (locked anti-phase drive, `kff` 0.20/0.18, `kp` 6, `pos_tol` 0.2°).

Suggested order: velocity feed-forward `kff` (removes the lag) → `kp` → `kd` against ringing → a small `ki` for the final error. If the motion is too violent, lower `vmax` / `amax`.

## 5. Characterization (accuracy and precision)

```bash
ros2 run scara_control characterize --config src/scara_control/config/characterization.yaml --out run1.csv
ros2 run scara_control analyze run1.csv                 # ground truth (meas_x/y columns)
ros2 run scara_control analyze run1.csv --source enc    # encoder-only (internal repeatability)
```

This runs 5 points × 10 cycles in ISO 9283 style and reports AP (accuracy), mean error and RP (precision), as tables and PNG plots. See `docs/ground_truth.md` for the proposed ground-truth method. The encoders are on the motor side, so gearbox backlash and belt stretch only show up against the ground truth, not in `--source enc`.

## 6. Visualization

```bash
ros2 launch scara_description display.launch.py         # model only, joint sliders, no robot needed
```

The visuals are the CAD STLs in `scara_description/meshes/` (base, link1, link2; `link3.stl` for the quill is still to come, a cylinder stands in for it). Each mesh is exported in mm with its origin on its joint axis; the xacro rotates/offsets them. The VS Code URDF visualizer finds `package://scara_description` through `urdf-visualizer.packages` in `.vscode/settings.json`.

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| RViz links lying at the origin, pendant OFFLINE | ESP32 not connected to the agent: press EN with terminal B running |
| Flash: "port is busy" / "multiple access on port" | the agent holds the port: Ctrl+C in terminal B (or `pkill -f micro_ros_agent`) |
| Flash: "No such file /dev/ttyUSB0" | board disconnected; replug, check `ls /dev/ttyUSB*`. Kernel `error -71` = bad cable/connection |
| ESP32 boots but never connects | UART0 pins: `CONFIG_MICROROS_UART_TXD=1` / `RXD=3` must be set (they are, in `sdkconfig.defaults`). With the console off, ESP-IDF 5.4 leaves UART0 unrouted and `-1` sends nothing |
| `HOME` faults "joint not moving" | motor power off, wiring, or the joint is jammed |
| `HOME` faults "hit the + switch, expected the − one" | flip `Jx_RIGHT_IS_POSITIVE` (or the motor/encoder pair) |
| GO rejected "outside workspace" / "joint limits" | target outside the 170–202 mm ring or the joint limits |
| Pendant shows `BOOT: reset = BROWNOUT` | supply dip: separate/clean ESP32 supply, common ground, bulk capacitor on the motor supply |
| Keys do nothing | click the pendant window (not the x/y/z boxes); after a FAULT press C first |

## Still to do

- Measure the tool tip height with the P up and down (`TOOL_Z_UP_M` / `TOOL_Z_DOWN_M`, and `tool_z_up` / `tool_z_down` in the xacro).
- Add `meshes/link3.stl` (quill) to the model.

## Topics

| Topic | Type | Direction |
|---|---|---|
| `/scara/cmd_pose` | geometry_msgs/Point [m] | PC → ESP32 |
| `/scara/jog` | geometry_msgs/Vector3 (x: J1, y: J2 in [−1, 1]) | PC → ESP32 |
| `/scara/deadman` | std_msgs/Bool (≥ 10 Hz heartbeat) | PC → ESP32 |
| `/scara/cmd` | std_msgs/Int32 (`CMD_*`) | PC → ESP32 |
| `/scara/params` | std_msgs/Float32MultiArray [id, value, …] | PC → ESP32 |
| `/scara/state` | std_msgs/Float32MultiArray (`ST_*`), 20 Hz | ESP32 → PC |
| `/scara/event` | std_msgs/String (IK flow, faults, boot reason) | ESP32 → PC |
| `/joint_states`, `/scara/tool_pose`, `/scara/target_marker` | from `scara_bridge` | PC |

## Saving to GitHub

```bash
git add -A
git commit -m "describe what changed"
git push
```

`build/`, `install/`, `log/`, the generated `sdkconfig` and all `*.csv` files are ignored; the tuning plots in `results/` are committed. The micro-ROS component is a submodule (a pointer to its official repository), so a fresh clone needs `git submodule update --init`.
