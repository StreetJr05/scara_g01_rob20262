# PID tuning runs (2026-10-16)

Made with `ros2 run scara_control tune --pair j1|j2`. Each folder has `response.png` (reference vs. position, error, PWM) and `state.csv` (raw `/scara/state`, kept locally, not in git).

| Run | Drive | kp | kff (J1/J2) | pos_tol | J1 lag | J2 lag | overshoot | final error |
|---|---|---|---|---|---|---|---|---|
| 1 driveCoast | PWM on D2 (drive/coast) | 4 | 0 / 0 | 0.5° | 10–12° | 13° | — | 3–6°, stick-slip staircase |
| 2 antiphase | locked anti-phase | 4 | 0 / 0 | 0.5° | 3.7° | 4.0° | 0.4° | 0.33° |
| 3 + kff | locked anti-phase | 4 | 0.20 / 0.18 | 0.2° | 1.1° | 0.97° | ≤ 0.27° | ≤ 0.18° |
| 4 kp6 (**final**) | locked anti-phase | 6 | 0.20 / 0.18 | 0.2° | 0.48–0.82° | 0.67° | ≤ 0.14° | 0.06–0.18° |

Findings:
- Run 1: with the PWM on D2 the bridge tri-states in the off phase, the motor sees ≈ (2d − 1)·Vbat and the joints only break away at |duty| ≈ 0.5. Fixed in firmware by locked anti-phase (PWM on INV, D2 = enable), `MOTOR_DRIVE_ANTIPHASE` in `scara_config.h`.
- Run 2 → 3: the lag peaked at top speed, so velocity feed-forward was missing. Measured: duty ≈ 0.07 + 0.2 · |speed rad/s| on both joints.
- Run 4: the remaining lag is during acceleration/braking; kp 4 → 6 reduced it without ringing or saturation.
- J2 moves ≈ 0.5° while only J1 moves (J1 → J2 belt coupling), then settles within 0.2°.

Final values are in `ros2_ws/src/scara_control/config/scara_params.yaml`.
