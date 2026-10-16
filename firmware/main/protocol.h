/**
 * @file protocol.h
 * @brief Contract between the firmware and ROS 2.
 *
 * MUST stay in sync with ros2_ws/src/scara_control/scara_control/protocol.py
 *
 * Only standard messages are used so libmicroros needs no custom interfaces.
 *
 *  Subscriptions (PC -> ESP32)
 *   /scara/cmd_pose  geometry_msgs/Point          target (x, y, z) in meters, base frame
 *   /scara/jog       geometry_msgs/Vector3        x = J1 speed, y = J2 speed, in [-1, 1]
 *   /scara/deadman   std_msgs/Bool                teach-pendant enable, >= 10 Hz heartbeat
 *   /scara/cmd       std_msgs/Int32               CMD_* below
 *   /scara/params    std_msgs/Float32MultiArray   pairs [id, value, id, value, ...]
 *
 *  Publishers (ESP32 -> PC)
 *   /scara/state     std_msgs/Float32MultiArray   ST_* layout below, 20 Hz
 *   /scara/event     std_msgs/String              human-readable events / IK flow
 */
#pragma once

#define TOPIC_CMD_POSE  "scara/cmd_pose"
#define TOPIC_JOG       "scara/jog"
#define TOPIC_DEADMAN   "scara/deadman"
#define TOPIC_CMD       "scara/cmd"
#define TOPIC_PARAMS    "scara/params"
#define TOPIC_STATE     "scara/state"
#define TOPIC_EVENT     "scara/event"

/* ---- /scara/cmd ---- */
typedef enum {
    CMD_NOP          = 0,
    CMD_HOME         = 1,
    CMD_STOP         = 2,   /* abort everything, motors off */
    CMD_P_UP         = 3,
    CMD_P_DOWN       = 4,   /* + valve ON if AUTO_VALVE_ON_DOWN */
    CMD_VALVE_ON     = 5,
    CMD_VALVE_OFF    = 6,
    CMD_RESET_FAULT  = 7,
    CMD_HOLD         = 8,   /* hold current position with the PID */
    CMD_FORCE_HOMED  = 9,   /* bench testing ONLY: current position := home pose */
} scara_cmd_t;

/* ---- operating modes ---- */
typedef enum {
    MODE_IDLE    = 0,   /* motors off */
    MODE_HOMING  = 1,
    MODE_HOLD    = 2,   /* PID holding a fixed position */
    MODE_JOG     = 3,   /* manual, deadman held */
    MODE_MOVING  = 4,   /* point-to-point sequence */
    MODE_FAULT   = 5,   /* motors off, needs CMD_RESET_FAULT */
} scara_mode_t;

/* ---- error codes (ST_ERROR) ---- */
typedef enum {
    ERR_NONE          = 0,
    ERR_UNREACHABLE   = 1,
    ERR_JOINT_LIMIT   = 2,
    ERR_NOT_HOMED     = 3,
    ERR_BUSY          = 4,
    ERR_LIMIT_SWITCH  = 5,
    ERR_HOME_TIMEOUT  = 6,
    ERR_FOLLOWING     = 7,
    ERR_SETTLE        = 8,
    ERR_SINGULAR      = 9,
} scara_err_t;

/* ---- /scara/state flag bits (ST_FLAGS) ---- */
#define FLAG_HOMED      (1u << 0)
#define FLAG_DEADMAN    (1u << 1)
#define FLAG_P_DOWN     (1u << 2)
#define FLAG_VALVE      (1u << 3)
#define FLAG_LIM1_R     (1u << 4)
#define FLAG_LIM1_L     (1u << 5)
#define FLAG_LIM2_R     (1u << 6)
#define FLAG_LIM2_L     (1u << 7)
#define FLAG_REACHED    (1u << 8)   /* last point-to-point move completed */

/* ---- /scara/state layout ---- */
enum {
    ST_MODE = 0, ST_FLAGS, ST_Q1, ST_Q2,        /* rad (encoders)               */
    ST_Q1_REF, ST_Q2_REF,                       /* rad, instantaneous reference */
    ST_Q1_GOAL, ST_Q2_GOAL,                     /* rad, IK solution             */
    ST_X, ST_Y, ST_Z,                           /* m, forward kinematics        */
    ST_U1, ST_U2,                               /* duty [-1, 1]                 */
    ST_DQ1, ST_DQ2,                             /* rad/s                        */
    ST_ERROR, ST_UPTIME,                        /* scara_err_t, s               */
    ST_MOVE_ID,                                 /* accepted cmd_pose counter    */
    ST_ELBOW,                                   /* +1 / -1 solution used        */
    ST_LEN
};

/* ---- /scara/params ids ---- */
enum {
    P_J1_KP = 0, P_J1_KI, P_J1_KD, P_J1_KFF, P_J1_MIN_DUTY, P_J1_MAX_DUTY,
    P_J1_VMAX, P_J1_AMAX, P_J1_JOG_VMAX,
    P_J2_KP = 10, P_J2_KI, P_J2_KD, P_J2_KFF, P_J2_MIN_DUTY, P_J2_MAX_DUTY,
    P_J2_VMAX, P_J2_AMAX, P_J2_JOG_VMAX,
    P_POS_TOL = 20, P_SETTLE_MS, P_P_UP_MS, P_P_DOWN_MS, P_DERIV_CUTOFF_HZ,
    P_FOLLOW_ERR_MAX, P_ELBOW_PREF,
    P_COUNT
};
#define P_JOINT_STRIDE 10
#define PARAMS_MSG_CAPACITY 64
