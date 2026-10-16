/**
 * @file scara_config.h
 * @brief ALL hardware/geometry constants of the RRP SCARA live here.
 *
 * Values marked "TODO(measure)" are placeholders: measure your robot and
 * update them. Runtime-tunable values (PID gains, speeds, tolerances) have
 * their DEFAULTS here but can be changed live from ROS 2 through
 * ros2_ws/src/scara_control/config/scara_params.yaml (no reflashing needed).
 *
 * Units inside the firmware: meters, radians, seconds, duty in [-1, 1].
 */
#pragma once

#include <math.h>

#define DEG2RAD(d) ((d) * (float)M_PI / 180.0f)
#define RAD2DEG(r) ((r) * 180.0f / (float)M_PI)

/* ------------------------------------------------------------------------- */
/* Pinout (from the team's wiring table)                                      */
/* ------------------------------------------------------------------------- */
#define J1_PIN_DIR        26   /* MC33926 INV                               */
#define J1_PIN_PWM        25   /* MC33926 D2 (PWM)                          */
#define J1_PIN_ENC_A      35   /* input only, no pull-up. Pololu: yellow = A, white = B; */
#define J1_PIN_ENC_B      34   /* swapping them only reverses direction (Jx_ENC_INVERT) */
#define J1_PIN_LIM_RIGHT  27   /* NO switch to GND, INPUT_PULLUP, active LOW */
#define J1_PIN_LIM_LEFT   13

#define J2_PIN_DIR        33
#define J2_PIN_PWM        32
#define J2_PIN_ENC_A      39   /* VN */
#define J2_PIN_ENC_B      36   /* VP */
#define J2_PIN_LIM_RIGHT  19
#define J2_PIN_LIM_LEFT   18

#define PIN_SOLENOID      16   /* P axis. Energized (1) = P UP, released (0) = P DOWN */
#define PIN_VALVE         17   /* Electrovalve. 1 = ON                      */

/* Logic level that ENERGIZES each output. Many relay modules are active-LOW:
 * if yours is, set the corresponding value to 0. */
#define SOLENOID_ACTIVE_LEVEL 1
#define VALVE_ACTIVE_LEVEL    1

/* ------------------------------------------------------------------------- */
/* PWM (LEDC)                                                                */
/* ------------------------------------------------------------------------- */
#define PWM_FREQ_HZ       20000
/* 1: locked anti-phase (PWM on INV, D2 = enable): linear, no dead zone.
 * 0: drive/coast (PWM on D2): ~50 % duty dead zone with this driver. See hal.c */
#define MOTOR_DRIVE_ANTIPHASE 1
#define PWM_RES_BITS      10           /* 0..1023 */

/* ------------------------------------------------------------------------- */
/* Signs. Convention: looking at the robot from ABOVE, positive joint angle   */
/* = counter-clockwise. q1 = 0 along the base +X axis, q2 = 0 with link 2     */
/* aligned with link 1 (arm stretched).                                       */
/* Bring-up: jog with the pendant. If "+" makes the joint turn CW, flip       */
/* Jx_MOTOR_INVERT. If the reported angle goes the wrong way, flip            */
/* Jx_ENC_INVERT. (A wrong combination trips the "following error" fault.)    */
/* ------------------------------------------------------------------------- */
#define J1_MOTOR_INVERT   1
#define J1_ENC_INVERT     1
#define J2_MOTOR_INVERT   1
#define J2_ENC_INVERT     1

/* Which limit switch sits on the POSITIVE (CCW) end of each joint.
 * 1 = the RIGHT switch is the + end, 0 = the LEFT switch is the + end. */
#define J1_RIGHT_IS_POSITIVE 0   /* verified: left switch = + end */
#define J2_RIGHT_IS_POSITIVE 0   /* verified: left switch = + end */

/* ------------------------------------------------------------------------- */
/* Encoders: quadrature x4 counts per ONE revolution of the JOINT (output).  */
/* counts = encoder_PPR * 4 * gearbox_ratio * external (belt/pulley) ratio  */
/* ------------------------------------------------------------------------- */
/* Pololu 37D 50:1 (16 PPR -> 64 CPR at the motor) + 20T:100T belt (5:1) */
#define J1_COUNTS_PER_REV (16.0f * 4.0f * 50.0f * 5.0f)   /* = 16000 */
#define J2_COUNTS_PER_REV (16.0f * 4.0f * 50.0f * 5.0f)   /* = 16000 */
#define ENC_GLITCH_NS     1000         /* PCNT glitch filter */

/* J1 -> J2 coupling. J2's motor is on the base and drives a pulley that turns
 * freely on the J1 axis (20 T); a belt along link 1 drives link 2 (100 T).
 * Turning link 1 alone therefore also bends the elbow:
 *     q2 = th2 - J2_COUPLING * q1,   J2_COUPLING = 20 / 100
 * (th2 = J2 encoder angle, scaled by J2_COUNTS_PER_REV as if J1 stood still).
 * Set to 0 for a robot with the J2 motor on link 1. */
#define J2_COUPLING       (20.0f / 100.0f)

/* ------------------------------------------------------------------------- */
/* Geometry (must match ros2_ws/src/scara_description/urdf/scara.urdf.xacro) */
/* ------------------------------------------------------------------------- */
#define L1_M              0.11211f     /* joint1 -> joint2 axis distance (112.11 mm) */
#define L2_M              0.09000f     /* joint2 -> P (tool) axis distance (90.00 mm) */
#define TOOL_Z_UP_M       0.065f       /* tool tip height when P is UP    TODO(measure) */
#define TOOL_Z_DOWN_M     0.025f       /* tool tip height when P is DOWN  TODO(measure) */

/* Soft joint limits (rad), used by IK and jogging once the robot is homed.
 * Keep them a few degrees INSIDE the physical limit switches. */
#define J1_MIN_RAD        DEG2RAD(-81.0f)    /* - switch at -86.22 */
#define J1_MAX_RAD        DEG2RAD( 80.0f)    /* + switch at 84.94 */
#define J2_MIN_RAD        DEG2RAD(-64.0f)    /* - switch at -68.88 */
#define J2_MAX_RAD        DEG2RAD( 67.5f)    /* + switch at ~72.7 */

/* IK: minimum |sin(q2)| allowed. Below this the arm is (almost) fully
 * stretched or folded = singular configuration (Jacobian loses rank). */
#define IK_SINGULAR_EPS   0.02f

/* ------------------------------------------------------------------------- */
/* Homing                                                                    */
/* ------------------------------------------------------------------------- */
/* Each joint drives (closed loop, HOME_SPEED_RAD) toward its HOME switch, then the encoder is
 * set to HOME_SWITCH_ANGLE, the joint backs off and finally both joints go
 * to the HOME pose with a closed-loop trajectory. */
#define HOME_ORDER_J2_FIRST    1                    /* fold the forearm first */
#define J1_HOME_TOWARD_POSITIVE 0                   /* 0: home on the - end   */
#define J2_HOME_TOWARD_POSITIVE 0
#define J1_HOME_SWITCH_ANGLE   DEG2RAD(-86.22f)     /* 2026-10-15: true 0 was 81.22 deg past the backed-off switch (+5) */
#define J2_HOME_SWITCH_ANGLE   DEG2RAD(-68.88f)     /* 2026-10-15: -31.67 - 37.21 (q2 read with arm truly straight) */
#define J1_HOME_POSE_RAD       DEG2RAD(0.0f)        /* pose after homing */
#define J2_HOME_POSE_RAD       DEG2RAD(0.0f)        /* arm stretched: links at 180 deg */
#define HOME_SPEED_RAD         DEG2RAD(15.0f)       /* closed-loop joint speed while seeking (rad/s) */
#define HOME_BACKOFF_RAD       DEG2RAD(5.0f)
#define HOME_TIMEOUT_MS        20000
#define HOME_STALL_MS          1500                 /* seek must move at least ...  */
#define HOME_STALL_MIN_RAD     DEG2RAD(3.0f)        /* ... this much in that window */

/* ------------------------------------------------------------------------- */
/* Control loop                                                              */
/* ------------------------------------------------------------------------- */
#define CTRL_PERIOD_US         5000      /* Ts = 5 ms (200 Hz) */
#define LIMIT_DEBOUNCE_TICKS   4         /* 4 * 5 ms = 20 ms */
#define DEADMAN_TIMEOUT_MS     250       /* no deadman heartbeat -> stop */
#define JOG_TIMEOUT_MS         250
#define SETTLE_TIMEOUT_MS      3000
#define VEL_FILTER_HZ          30.0f     /* encoder velocity low-pass */
#define P_UP_ON_BOOT           1         /* energize solenoid at boot so P is up */
#define AUTO_VALVE_ON_DOWN     1         /* P reaches down -> valve ON */
#define STATE_PUB_PERIOD_MS    50        /* 20 Hz /scara/state */

/* ------------------------------------------------------------------------- */
/* DEFAULT runtime tunables (overridden by scara_params.yaml at runtime)     */
/* PID units: error in rad -> output duty [-1, 1]                            */
/* ------------------------------------------------------------------------- */
#define J1_KP_DEF        6.0f
#define J1_KI_DEF        1.5f
#define J1_KD_DEF        0.08f
#define J1_KFF_DEF       0.20f     /* velocity feed-forward [duty / (rad/s)], measured with `tune` */
#define J1_MIN_DUTY_DEF  0.08f     /* static-friction (deadzone) compensation */
#define J1_MAX_DUTY_DEF  0.80f
#define J1_VMAX_DEF      1.5f      /* rad/s, point-to-point */
#define J1_AMAX_DEF      3.0f      /* rad/s^2 */
#define J1_JOG_VMAX_DEF  0.6f      /* rad/s at full jog */

#define J2_KP_DEF        6.0f
#define J2_KI_DEF        1.5f
#define J2_KD_DEF        0.08f
#define J2_KFF_DEF       0.18f
#define J2_MIN_DUTY_DEF  0.08f
#define J2_MAX_DUTY_DEF  0.80f
#define J2_VMAX_DEF      2.0f
#define J2_AMAX_DEF      4.0f
#define J2_JOG_VMAX_DEF  0.8f

#define POS_TOL_DEF          DEG2RAD(0.2f)   /* "in position" band */
#define SETTLE_MS_DEF        200.0f          /* must stay in band this long */
#define P_UP_MS_DEF          300.0f          /* time for the solenoid to rise  */
#define P_DOWN_MS_DEF        300.0f          /* time for the P to reach bottom */
#define DERIV_CUTOFF_HZ_DEF  20.0f           /* PID derivative low-pass        */
#define FOLLOW_ERR_MAX_DEF   DEG2RAD(25.0f)  /* |ref - q| above this = FAULT   */
#define ELBOW_PREF_DEF       0.0f            /* 0 = least motion, +1 / -1 = sign of q2 */
