"""Mirror of firmware/main/protocol.h. Keep both files in sync."""

TOPIC_CMD_POSE = '/scara/cmd_pose'
TOPIC_JOG = '/scara/jog'
TOPIC_DEADMAN = '/scara/deadman'
TOPIC_CMD = '/scara/cmd'
TOPIC_PARAMS = '/scara/params'
TOPIC_STATE = '/scara/state'
TOPIC_EVENT = '/scara/event'

# /scara/cmd
CMD_NOP = 0
CMD_HOME = 1
CMD_STOP = 2
CMD_P_UP = 3
CMD_P_DOWN = 4
CMD_VALVE_ON = 5
CMD_VALVE_OFF = 6
CMD_RESET_FAULT = 7
CMD_HOLD = 8
CMD_FORCE_HOMED = 9

# modes
MODE_IDLE, MODE_HOMING, MODE_HOLD, MODE_JOG, MODE_MOVING, MODE_FAULT = range(6)
MODE_NAMES = ['IDLE', 'HOMING', 'HOLD', 'JOG', 'MOVING', 'FAULT']

# errors
(ERR_NONE, ERR_UNREACHABLE, ERR_JOINT_LIMIT, ERR_NOT_HOMED, ERR_BUSY, ERR_LIMIT_SWITCH,
 ERR_HOME_TIMEOUT, ERR_FOLLOWING, ERR_SETTLE, ERR_SINGULAR) = range(10)
ERR_NAMES = ['none', 'unreachable', 'joint limit', 'not homed', 'busy', 'limit switch',
             'home timeout', 'following error', 'settle timeout', 'singular']
REJECT_ERRORS = {ERR_UNREACHABLE, ERR_JOINT_LIMIT, ERR_NOT_HOMED, ERR_BUSY, ERR_SINGULAR}

# flags
FLAG_HOMED = 1 << 0
FLAG_DEADMAN = 1 << 1
FLAG_P_DOWN = 1 << 2
FLAG_VALVE = 1 << 3
FLAG_LIM1_R = 1 << 4
FLAG_LIM1_L = 1 << 5
FLAG_LIM2_R = 1 << 6
FLAG_LIM2_L = 1 << 7
FLAG_REACHED = 1 << 8

# /scara/state layout
(ST_MODE, ST_FLAGS, ST_Q1, ST_Q2, ST_Q1_REF, ST_Q2_REF, ST_Q1_GOAL, ST_Q2_GOAL,
 ST_X, ST_Y, ST_Z, ST_U1, ST_U2, ST_DQ1, ST_DQ2, ST_ERROR, ST_UPTIME, ST_MOVE_ID,
 ST_ELBOW, ST_LEN) = range(20)

# /scara/params ids  (name in scara_params.yaml -> id)
_JOINT_FIELDS = ['kp', 'ki', 'kd', 'kff', 'min_duty', 'max_duty', 'vmax', 'amax', 'jog_vmax']
PARAM_IDS = {}
for _j in range(2):
    for _i, _f in enumerate(_JOINT_FIELDS):
        PARAM_IDS[f'j{_j + 1}_{_f}'] = _j * 10 + _i
PARAM_IDS.update({
    'pos_tol': 20,          # rad
    'settle_ms': 21,
    'p_up_ms': 22,
    'p_down_ms': 23,
    'deriv_cutoff_hz': 24,
    'follow_err_max': 25,   # rad
    'elbow_pref': 26,       # 0 least motion, +1 / -1 sign of q2
})


def flags_of(state):
    return int(state[ST_FLAGS])
