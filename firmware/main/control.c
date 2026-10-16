/**
 * @file control.c
 * @brief Robot state machine + closed-loop joint control.
 *
 *  Point-to-point flow (MODE_MOVING):
 *    /scara/cmd_pose (x,y,z) -> ik_solve() -> q_goal -> [RAISE P if down]
 *    -> quintic trajectory q_ref(t) -> PID per joint -> PWM
 *    -> SETTLE (|q_goal - q| < pos_tol during settle_ms)
 *    -> [LOWER P if z is down -> valve ON] -> HOLD, FLAG_REACHED
 *
 *  Manual flow (MODE_JOG): deadman held + jog speed -> reference integrates
 *  the jog speed -> PID. Releasing the deadman (or losing its heartbeat for
 *  DEADMAN_TIMEOUT_MS) switches the motors OFF immediately (MODE_IDLE).
 */
#include "control.h"
#include "scara_config.h"
#include "hal.h"
#include "pid.h"
#include "kinematics.h"
#include "trajectory.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_attr.h"

/* Survive a software / panic / brownout reset (not a power-on): what the
 * control loop was doing last, reported as an event after the next boot. */
RTC_NOINIT_ATTR static uint32_t rtc_magic, rtc_mode, rtc_hphase, rtc_home_idx;
#define RTC_MAGIC 0x5CA2A001u

#define TS ((float)CTRL_PERIOD_US * 1e-6f)
#define EVENT_LEN 128

/* ======================================================================== */
/* Shared data (micro-ROS task <-> control task)                             */
/* ======================================================================== */

typedef struct {
    enum { IN_CMD, IN_GOTO } type;
    int cmd;
    float x, y, z;
} ctrl_input_t;

typedef struct {
    pid_gains_t pid[NUM_JOINTS];
    float vmax[NUM_JOINTS], amax[NUM_JOINTS], jog_vmax[NUM_JOINTS];
    float pos_tol, settle_ms, p_up_ms, p_down_ms, follow_err_max;
    int   elbow_pref;
} tunables_t;

typedef struct { char text[EVENT_LEN]; } event_t;

static portMUX_TYPE  s_lock = portMUX_INITIALIZER_UNLOCKED;
static QueueHandle_t s_in_q;
static QueueHandle_t s_ev_q;
static TaskHandle_t  s_task;

static tunables_t s_tun;                 /* protected by s_lock */
static float      s_jog[NUM_JOINTS];     /* protected by s_lock */
static int64_t    s_jog_ms;
static bool       s_deadman;
static int64_t    s_deadman_ms;
static bool       s_comm;
static float      s_state[ST_LEN];       /* protected by s_lock */

/* ======================================================================== */
/* Control-task private state                                               */
/* ======================================================================== */

typedef enum { SEQ_RAISE, SEQ_MOVE, SEQ_SETTLE, SEQ_LOWER } move_phase_t;
typedef enum { H_RAISE, H_RELEASE, H_SEEK, H_BACKOFF, H_POSE } home_phase_t;

static scara_mode_t mode = MODE_IDLE;
static scara_err_t  last_err = ERR_NONE;
static bool homed, p_down, valve, reached;
/* q: joint angles (kinematics, limits, references). th: motor-side angles from the
 * encoders. J2's motor sits on the base, so th2 = q2 + J2_COUPLING * q1. */
static float q[NUM_JOINTS], dq[NUM_JOINTS];
static float th[NUM_JOINTS], dth[NUM_JOINTS], th_prev[NUM_JOINTS];
static float q_ref[NUM_JOINTS], dq_ref[NUM_JOINTS], q_goal[NUM_JOINTS], u[NUM_JOINTS];
static float jog_v[NUM_JOINTS];
static pid_state_t pid[NUM_JOINTS];
static traj_t traj;
static int elbow = 1;
static int move_id;

static move_phase_t mphase;
static bool goal_p_down;
static int64_t phase_t0, settle_t0;
static int64_t valve_on_at = -1;         /* pending automatic valve ON */

static home_phase_t hphase;
static int home_idx;
#if HOME_ORDER_J2_FIRST
static const int home_order[NUM_JOINTS] = { 1, 0 };
#else
static const int home_order[NUM_JOINTS] = { 0, 1 };
#endif

static bool lim_r[NUM_JOINTS], lim_l[NUM_JOINTS];
static uint8_t lim_r_cnt[NUM_JOINTS], lim_l_cnt[NUM_JOINTS];

static tunables_t tun;                   /* local copy, refreshed every tick */

static const bool right_is_pos[NUM_JOINTS] = { J1_RIGHT_IS_POSITIVE, J2_RIGHT_IS_POSITIVE };
static const bool home_toward_pos[NUM_JOINTS] = { J1_HOME_TOWARD_POSITIVE, J2_HOME_TOWARD_POSITIVE };
static const float home_switch_angle[NUM_JOINTS] = { J1_HOME_SWITCH_ANGLE, J2_HOME_SWITCH_ANGLE };
static const float home_pose[NUM_JOINTS] = { J1_HOME_POSE_RAD, J2_HOME_POSE_RAD };
static const float qmin[NUM_JOINTS] = { J1_MIN_RAD, J2_MIN_RAD };
static const float qmax[NUM_JOINTS] = { J1_MAX_RAD, J2_MAX_RAD };

static inline int64_t now_ms(void) { return esp_timer_get_time() / 1000; }

static void event(const char *fmt, ...)
{
    event_t ev;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(ev.text, sizeof(ev.text), fmt, ap);
    va_end(ap);
    xQueueSend(s_ev_q, &ev, 0);   /* drop if full, never block the loop */
}

static bool lim_pos(int j) { return right_is_pos[j] ? lim_r[j] : lim_l[j]; }
static bool lim_neg(int j) { return right_is_pos[j] ? lim_l[j] : lim_r[j]; }

/* ======================================================================== */
/* Helpers                                                                   */
/* ======================================================================== */

static void read_sensors(void)
{
    const float a = expf(-2.0f * (float)M_PI * VEL_FILTER_HZ * TS);
    for (int j = 0; j < NUM_JOINTS; j++) {
        th[j] = hal_encoder_get_rad(j);
        dth[j] = a * dth[j] + (1.0f - a) * (th[j] - th_prev[j]) / TS;
        th_prev[j] = th[j];

        bool r = hal_limit_right_raw(j), l = hal_limit_left_raw(j);
        /* simple integrating debounce */
        if (r != lim_r[j]) { if (++lim_r_cnt[j] >= LIMIT_DEBOUNCE_TICKS) { lim_r[j] = r; lim_r_cnt[j] = 0; } }
        else lim_r_cnt[j] = 0;
        if (l != lim_l[j]) { if (++lim_l_cnt[j] >= LIMIT_DEBOUNCE_TICKS) { lim_l[j] = l; lim_l_cnt[j] = 0; } }
        else lim_l_cnt[j] = 0;
    }
    q[0] = th[0];
    q[1] = th[1] - J2_COUPLING * th[0];
    dq[0] = dth[0];
    dq[1] = dth[1] - J2_COUPLING * dth[0];
}

/** Re-read the encoders after a re-reference, without a velocity spike. */
static void sensors_reset(void)
{
    for (int j = 0; j < NUM_JOINTS; j++) {
        th[j] = th_prev[j] = hal_encoder_get_rad(j);
        dth[j] = 0.0f;
    }
    q[0] = th[0];
    q[1] = th[1] - J2_COUPLING * th[0];
    dq[0] = dq[1] = 0.0f;
}

/** Declare that joint j is now at angle `val` (rad). Re-referencing J1 also
 * shifts J2's encoder so that q2 (= th2 - c * q1) keeps its value. */
static void set_joint_angle(int j, float val)
{
    const float th0 = hal_encoder_get_rad(0), th1 = hal_encoder_get_rad(1);
    if (j == 0) {
        const float q2_now = th1 - J2_COUPLING * th0;
        hal_encoder_set_rad(0, val);
        hal_encoder_set_rad(1, q2_now + J2_COUPLING * val);
    } else {
        hal_encoder_set_rad(1, val + J2_COUPLING * th0);
    }
    sensors_reset();
}

static void motors_off(void)
{
    for (int j = 0; j < NUM_JOINTS; j++) u[j] = 0.0f;
    hal_motors_off();
}

static void hold_here(void)
{
    for (int j = 0; j < NUM_JOINTS; j++) {
        q_ref[j] = q[j];
        dq_ref[j] = 0.0f;
        jog_v[j] = 0.0f;
        pid_reset(&pid[j]);
    }
}

static void set_mode(scara_mode_t m)
{
    if (m == mode) return;
    if (m == MODE_IDLE || m == MODE_FAULT) {
        traj.active = false;
        motors_off();
    }
    if (m == MODE_HOLD || m == MODE_JOG || m == MODE_IDLE) {
        hold_here();
    }
    mode = m;
}

static void fault(scara_err_t err, const char *fmt, ...)
{
    char msg[EVENT_LEN - 8];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    last_err = err;
    set_mode(MODE_FAULT);
    event("FAULT %s", msg);
}

static void solenoid(bool up)
{
    hal_solenoid_set(up);
    p_down = !up;
}

static void set_valve(bool on)
{
    hal_valve_set(on);
    valve = on;
    valve_on_at = -1;
}

/** Lower P and, if configured, schedule valve ON once it reached the bottom. */
static void lower_p(void)
{
    solenoid(false);
#if AUTO_VALVE_ON_DOWN
    valve_on_at = now_ms() + (int64_t)tun.p_down_ms;
#endif
}

static void raise_p(void)
{
    valve_on_at = -1;
    solenoid(true);
}

/** Closed-loop: PID on every joint towards q_ref/dq_ref. */
static void run_pid(void)
{
    /* the PID acts on the motor-side angles: joint references -> motor references */
    const float th_ref[NUM_JOINTS]  = { q_ref[0],  q_ref[1]  + J2_COUPLING * q_ref[0] };
    const float dth_ref[NUM_JOINTS] = { dq_ref[0], dq_ref[1] + J2_COUPLING * dq_ref[0] };
    for (int j = 0; j < NUM_JOINTS; j++) {
        u[j] = pid_update(&tun.pid[j], &pid[j], th_ref[j], dth_ref[j], th[j], dth[j], TS);
        /* never push into a pressed limit switch */
        if ((u[j] > 0.0f && lim_pos(j)) || (u[j] < 0.0f && lim_neg(j))) {
            u[j] = 0.0f;
        }
        hal_motor_set(j, u[j]);
    }
}

static bool following_error_ok(void)
{
    for (int j = 0; j < NUM_JOINTS; j++) {
        if (fabsf(q_ref[j] - q[j]) > tun.follow_err_max) {
            fault(ERR_FOLLOWING, "following error J%d: ref=%.1f q=%.1f deg (check motor/encoder sign, gains)",
                  j + 1, RAD2DEG(q_ref[j]), RAD2DEG(q[j]));
            return false;
        }
    }
    return true;
}

/** Bit mask of the pressed limit switches (J1 R/L, J2 R/L). */
static uint32_t limit_mask(void)
{
    uint32_t m = 0;
    for (int j = 0; j < NUM_JOINTS; j++) m |= (lim_r[j] ? 1u : 0u) << (2 * j) | (lim_l[j] ? 2u : 0u) << (2 * j);
    return m;
}

/* ======================================================================== */
/* Commands                                                                  */
/* ======================================================================== */

static void start_goto(float x, float y, float z)
{
    move_id++;
    reached = false;

    if (!homed) { last_err = ERR_NOT_HOMED; event("REJECT #%d: robot not homed (send HOME first)", move_id); return; }
    if (mode != MODE_IDLE && mode != MODE_HOLD) { last_err = ERR_BUSY; event("REJECT #%d: busy (mode %d)", move_id, mode); return; }

    ik_solution_t sol;
    scara_err_t err = ik_solve(x, y, z, q[0], q[1], tun.elbow_pref, &sol);
    if (err != ERR_NONE) {
        last_err = err;
        const char *why = err == ERR_UNREACHABLE ? "outside workspace"
                        : err == ERR_SINGULAR ? "singular (arm stretched/folded)"
                        : "joint limits";
        event("REJECT #%d: (%.1f, %.1f, %.1f) mm -> %s (r=%.1f mm)", move_id,
              x * 1e3f, y * 1e3f, z * 1e3f, why, sqrtf(x * x + y * y) * 1e3f);
        return;
    }

    last_err = ERR_NONE;
    q_goal[0] = sol.q1;
    q_goal[1] = sol.q2;
    elbow = sol.elbow;
    goal_p_down = sol.p_down;
    event("GOTO #%d (%.1f, %.1f, %.1f) mm -> IK q1=%.2f q2=%.2f deg, elbow %c%s, P %s",
          move_id, x * 1e3f, y * 1e3f, z * 1e3f, RAD2DEG(sol.q1), RAD2DEG(sol.q2),
          sol.elbow > 0 ? '+' : '-', sol.alt_valid ? " (2 sols)" : " (1 sol)",
          sol.p_down ? "DOWN" : "UP");

    if (mode == MODE_IDLE) hold_here();
    mode = MODE_MOVING;
    phase_t0 = now_ms();
    if (p_down) {
        raise_p();
        mphase = SEQ_RAISE;
    } else {
        mphase = SEQ_MOVE;
        traj_plan(&traj, q_ref, q_goal, tun.vmax, tun.amax);
    }
}

static void start_home(void)
{
    if (mode == MODE_MOVING || mode == MODE_HOMING || mode == MODE_FAULT) {
        event("REJECT HOME: mode %d", mode);
        return;
    }
    homed = false;
    reached = false;
    set_mode(MODE_IDLE);
    mode = MODE_HOMING;
    raise_p();
    hold_here();
    hphase = H_RAISE;
    home_idx = 0;
    phase_t0 = now_ms();
    event("HOME started");
}

static void handle_cmd(int cmd)
{
    switch (cmd) {
    case CMD_HOME: start_home(); break;
    case CMD_STOP:
        if (mode != MODE_FAULT) set_mode(MODE_IDLE);
        event("STOP: motors off");
        break;
    case CMD_P_UP:
    case CMD_P_DOWN:
    case CMD_VALVE_ON:
    case CMD_VALVE_OFF:
        if (mode == MODE_MOVING || mode == MODE_HOMING) { event("REJECT IO cmd %d: busy", cmd); break; }
        if (cmd == CMD_P_UP) raise_p();
        else if (cmd == CMD_P_DOWN) lower_p();
        else set_valve(cmd == CMD_VALVE_ON);
        event("IO: P %s, valve %s%s", p_down ? "DOWN" : "UP", valve ? "ON" : "OFF",
              valve_on_at >= 0 ? " (ON when P is down)" : "");
        break;
    case CMD_RESET_FAULT:
        if (mode == MODE_FAULT) { mode = MODE_IDLE; hold_here(); }
        last_err = ERR_NONE;
        event("fault reset");
        break;
    case CMD_HOLD:
        if (mode == MODE_IDLE) { set_mode(MODE_HOLD); event("HOLD"); }
        break;
    case CMD_FORCE_HOMED:
        if (mode == MODE_IDLE || mode == MODE_HOLD) {
            set_joint_angle(0, home_pose[0]);
            set_joint_angle(1, home_pose[1]);
            hold_here();
            homed = true;
            event("WARNING: FORCE_HOMED, current pose := home pose (bench test only)");
        }
        break;
    default: break;
    }
}

/* ======================================================================== */
/* Mode handlers                                                             */
/* ======================================================================== */

/* Closed-loop constant-speed motion of joint j for homing (same idea as jog):
 * the PID supplies whatever duty the joint needs, up to max_duty. The other
 * joint holds its position. */
static void home_drive(int j, float v)
{
    q_ref[j] += v * TS;
    dq_ref[j] = v;
    const float lead = 0.5f * tun.follow_err_max;   /* reference never runs away from a blocked joint */
    if (q_ref[j] > q[j] + lead) q_ref[j] = q[j] + lead;
    if (q_ref[j] < q[j] - lead) q_ref[j] = q[j] - lead;
    for (int k = 0; k < NUM_JOINTS; k++) if (k != j) dq_ref[k] = 0.0f;
    run_pid();
}

static void do_homing(void)
{
    /* Last phase: both joints referenced, move to the home pose. Handled first
     * because home_idx == NUM_JOINTS here, so home_order[home_idx] is invalid. */
    if (hphase == H_POSE) {
        traj_step(&traj, TS, q_ref, dq_ref);
        run_pid();
        if (!traj.active) {
            homed = true;
            last_err = ERR_NONE;
            set_mode(MODE_HOLD);
            q_ref[0] = home_pose[0];
            q_ref[1] = home_pose[1];
            event("HOME done: q=(%.1f, %.1f) deg", RAD2DEG(q[0]), RAD2DEG(q[1]));
        }
        return;
    }

    const int j = home_order[home_idx];
    const bool to_pos = home_toward_pos[j];
    const bool at_switch = to_pos ? lim_pos(j) : lim_neg(j);
    const float dir = to_pos ? 1.0f : -1.0f;
    const int64_t t = now_ms();

    if (t - phase_t0 > HOME_TIMEOUT_MS) {
        fault(ERR_HOME_TIMEOUT, "homing J%d: switch not found (wiring / J%d_RIGHT_IS_POSITIVE?)", j + 1, j + 1);
        return;
    }

    switch (hphase) {
    case H_RAISE:            /* let the P go up before any XY motion */
        if (t - phase_t0 >= (int64_t)tun.p_up_ms) { hphase = at_switch ? H_RELEASE : H_SEEK; phase_t0 = t; }
        break;
    case H_RELEASE:          /* already on the switch: move away first */
        home_drive(j, -dir * HOME_SPEED_RAD);
        if (!at_switch) { hphase = H_SEEK; phase_t0 = t; }
        break;
    case H_SEEK: {
        /* Seek is open loop, so catch the two ways it can push into a hard stop:
         * the OTHER switch of this joint closes (wrong direction / switch side), or
         * the joint does not move (no motor power, stalled, encoder not counting). */
        static int64_t seek_t0 = -1, chk_t;
        static float chk_q;
        static bool other_was_open;
        const bool other = to_pos ? lim_neg(j) : lim_pos(j);
        if (seek_t0 != phase_t0) {
            seek_t0 = phase_t0;
            chk_t = t;
            chk_q = q[j];
            other_was_open = !other;
        }
        if (other && other_was_open) {
            fault(ERR_HOME_TIMEOUT, "homing J%d: hit the %s switch, expected the %s one "
                  "(check J%d_MOTOR_INVERT / J%d_RIGHT_IS_POSITIVE / J%d_HOME_TOWARD_POSITIVE)",
                  j + 1, to_pos ? "-" : "+", to_pos ? "+" : "-", j + 1, j + 1, j + 1);
            return;
        }
        if (!other) other_was_open = true;
        if (t - chk_t >= HOME_STALL_MS) {
            if (fabsf(q[j] - chk_q) < HOME_STALL_MIN_RAD) {
                fault(ERR_HOME_TIMEOUT, "homing J%d: joint not moving (%.1f deg in %d ms): "
                      "motor power, wiring, end stop or encoder?",
                      j + 1, RAD2DEG(fabsf(q[j] - chk_q)), HOME_STALL_MS);
                return;
            }
            chk_t = t;
            chk_q = q[j];
        }

        home_drive(j, dir * HOME_SPEED_RAD);
        if (at_switch) {
            motors_off();
            set_joint_angle(j, home_switch_angle[j]);
            hold_here();   /* references follow the new encoder zero */
            hphase = H_BACKOFF;
            phase_t0 = t;
        }
        break;
    }
    case H_BACKOFF:
        home_drive(j, -dir * HOME_SPEED_RAD);
        if (!at_switch && fabsf(q[j] - home_switch_angle[j]) >= HOME_BACKOFF_RAD) {
            motors_off();
            hold_here();
            event("HOME J%d ok", j + 1);
            if (++home_idx < NUM_JOINTS) {
                hphase = H_SEEK;
                phase_t0 = t;
                const int nj = home_order[home_idx];
                if (home_toward_pos[nj] ? lim_pos(nj) : lim_neg(nj)) hphase = H_RELEASE;
            } else {
                hold_here();
                traj_plan(&traj, q_ref, home_pose, tun.vmax, tun.amax);
                for (int k = 0; k < NUM_JOINTS; k++) q_goal[k] = home_pose[k];
                hphase = H_POSE;
            }
        }
        break;
    case H_POSE:   /* handled above */
        break;
    }
}

static void do_moving(void)
{
    const int64_t t = now_ms();

    switch (mphase) {
    case SEQ_RAISE:
        run_pid();   /* hold XY while the P goes up */
        if (t - phase_t0 >= (int64_t)tun.p_up_ms) {
            mphase = SEQ_MOVE;
            traj_plan(&traj, q_ref, q_goal, tun.vmax, tun.amax);
        }
        break;
    case SEQ_MOVE:
        traj_step(&traj, TS, q_ref, dq_ref);
        run_pid();
        if (!traj.active) {
            mphase = SEQ_SETTLE;
            phase_t0 = t;
            settle_t0 = -1;
        }
        break;
    case SEQ_SETTLE: {
        run_pid();
        bool in_band = true;
        for (int j = 0; j < NUM_JOINTS; j++) in_band &= fabsf(q_goal[j] - q[j]) < tun.pos_tol;
        if (!in_band) settle_t0 = -1;
        else if (settle_t0 < 0) settle_t0 = t;

        const bool settled = settle_t0 >= 0 && (t - settle_t0) >= (int64_t)tun.settle_ms;
        const bool timeout = (t - phase_t0) >= SETTLE_TIMEOUT_MS;
        if (settled || timeout) {
            if (timeout && !settled) {
                last_err = ERR_SETTLE;
                event("WARN #%d: settle timeout, e=(%.2f, %.2f) deg", move_id,
                      RAD2DEG(q_goal[0] - q[0]), RAD2DEG(q_goal[1] - q[1]));
            }
            if (goal_p_down) lower_p();
            mphase = SEQ_LOWER;
            phase_t0 = t;
        }
        break;
    }
    case SEQ_LOWER:
        run_pid();
        if (!goal_p_down || t - phase_t0 >= (int64_t)tun.p_down_ms) {
            float x, y;
            fk_xy(q[0], q[1], &x, &y);
            float xg, yg;
            fk_xy(q_goal[0], q_goal[1], &xg, &yg);
            reached = true;
            mode = MODE_HOLD;
            event("REACHED #%d q=(%.2f, %.2f) deg, enc xy=(%.1f, %.1f) mm, |e|=%.2f mm, P %s, valve %s",
                  move_id, RAD2DEG(q[0]), RAD2DEG(q[1]), x * 1e3f, y * 1e3f,
                  hypotf(x - xg, y - yg) * 1e3f, p_down ? "DOWN" : "UP",
                  (valve || valve_on_at >= 0) ? "ON" : "OFF");
        }
        break;
    }
}

static void do_jog(bool deadman_ok, const float jog[NUM_JOINTS])
{
    if (!deadman_ok) {
        set_mode(MODE_IDLE);
        event("DEADMAN released: motors stopped");
        return;
    }
    for (int j = 0; j < NUM_JOINTS; j++) {
        float v_cmd = jog[j] * tun.jog_vmax[j];
        if ((v_cmd > 0.0f && lim_pos(j)) || (v_cmd < 0.0f && lim_neg(j))) v_cmd = 0.0f;
        /* acceleration-limited speed ramp */
        const float dv = tun.amax[j] * TS;
        if (v_cmd > jog_v[j] + dv) jog_v[j] += dv;
        else if (v_cmd < jog_v[j] - dv) jog_v[j] -= dv;
        else jog_v[j] = v_cmd;

        const float prev = q_ref[j];
        q_ref[j] += jog_v[j] * TS;
        dq_ref[j] = jog_v[j];

        if (homed) {   /* soft limits only make sense with a valid zero. They block motion
                        * further OUT only, so a joint beyond a limit can be jogged back. */
            if (jog_v[j] > 0.0f && q_ref[j] > qmax[j]) { q_ref[j] = fmaxf(prev, qmax[j]); dq_ref[j] = jog_v[j] = 0.0f; }
            if (jog_v[j] < 0.0f && q_ref[j] < qmin[j]) { q_ref[j] = fminf(prev, qmin[j]); dq_ref[j] = jog_v[j] = 0.0f; }
        }
        /* do not let the reference run away from a blocked joint */
        const float lead = 0.5f * tun.follow_err_max;
        if (q_ref[j] > q[j] + lead) q_ref[j] = q[j] + lead;
        if (q_ref[j] < q[j] - lead) q_ref[j] = q[j] - lead;
    }
    run_pid();
}

/* ======================================================================== */
/* Main loop                                                                 */
/* ======================================================================== */

static void publish_state(void)
{
    float x, y;
    fk_xy(q[0], q[1], &x, &y);
    uint32_t flags = (homed ? FLAG_HOMED : 0) | (p_down ? FLAG_P_DOWN : 0) | (valve ? FLAG_VALVE : 0) |
                     (lim_r[0] ? FLAG_LIM1_R : 0) | (lim_l[0] ? FLAG_LIM1_L : 0) |
                     (lim_r[1] ? FLAG_LIM2_R : 0) | (lim_l[1] ? FLAG_LIM2_L : 0) |
                     (reached ? FLAG_REACHED : 0);
    float st[ST_LEN] = {
        [ST_MODE] = mode, [ST_FLAGS] = 0,
        [ST_Q1] = q[0], [ST_Q2] = q[1], [ST_Q1_REF] = q_ref[0], [ST_Q2_REF] = q_ref[1],
        [ST_Q1_GOAL] = q_goal[0], [ST_Q2_GOAL] = q_goal[1],
        [ST_X] = x, [ST_Y] = y, [ST_Z] = p_down ? TOOL_Z_DOWN_M : TOOL_Z_UP_M,
        [ST_U1] = u[0], [ST_U2] = u[1], [ST_DQ1] = dq[0], [ST_DQ2] = dq[1],
        [ST_ERROR] = last_err, [ST_UPTIME] = (float)(esp_timer_get_time() * 1e-6),
        [ST_MOVE_ID] = move_id, [ST_ELBOW] = elbow,
    };
    taskENTER_CRITICAL(&s_lock);
    if (s_deadman && (now_ms() - s_deadman_ms) < DEADMAN_TIMEOUT_MS) flags |= FLAG_DEADMAN;
    st[ST_FLAGS] = (float)flags;
    memcpy(s_state, st, sizeof(st));
    taskEXIT_CRITICAL(&s_lock);
}

static void control_step(void)
{
    rtc_magic = RTC_MAGIC;
    rtc_mode = mode;
    rtc_hphase = hphase;
    rtc_home_idx = home_idx;

    /* snapshot of the shared inputs */
    float jog[NUM_JOINTS];
    bool deadman_ok, jog_fresh;
    const int64_t t = now_ms();
    taskENTER_CRITICAL(&s_lock);
    tun = s_tun;
    memcpy(jog, s_jog, sizeof(jog));
    deadman_ok = s_comm && s_deadman && (t - s_deadman_ms) < DEADMAN_TIMEOUT_MS;
    jog_fresh = (t - s_jog_ms) < JOG_TIMEOUT_MS;
    taskEXIT_CRITICAL(&s_lock);
    if (!jog_fresh) jog[0] = jog[1] = 0.0f;
    for (int j = 0; j < NUM_JOINTS; j++) tun.pid[j].dead_band = tun.pos_tol;

    read_sensors();

    ctrl_input_t in;
    while (xQueueReceive(s_in_q, &in, 0) == pdTRUE) {
        if (in.type == IN_CMD) handle_cmd(in.cmd);
        else start_goto(in.x, in.y, in.z);
    }

    /* delayed automatic valve ON (P has reached the bottom) */
    if (valve_on_at >= 0 && t >= valve_on_at) {
        set_valve(true);
        event("valve ON (P down)");
    }

    /* enter JOG from IDLE/HOLD when the deadman is held and a jog key pressed */
    const bool jog_req = fabsf(jog[0]) > 0.05f || fabsf(jog[1]) > 0.05f;
    if ((mode == MODE_IDLE || mode == MODE_HOLD) && deadman_ok && jog_req) {
        set_mode(MODE_JOG);
        reached = false;
    }

    /* a switch that closes while moving / holding autonomously = collision. A switch
     * that was already pressed is not a fault (run_pid never pushes into it), so the
     * robot can always be driven off a switch after a fault reset. */
    static uint32_t lim_prev;
    const uint32_t lim_now = limit_mask();
    if ((mode == MODE_MOVING || mode == MODE_HOLD) && (lim_now & ~lim_prev)) {
        fault(ERR_LIMIT_SWITCH, "limit switch hit in mode %d", mode);
    }
    lim_prev = lim_now;

    switch (mode) {
    case MODE_IDLE:
    case MODE_FAULT:
        motors_off();
        break;
    case MODE_HOMING:
        do_homing();
        break;
    case MODE_HOLD:
        for (int j = 0; j < NUM_JOINTS; j++) dq_ref[j] = 0.0f;
        if (following_error_ok()) run_pid();
        break;
    case MODE_JOG:
        do_jog(deadman_ok, jog);
        if (mode == MODE_JOG) following_error_ok();
        break;
    case MODE_MOVING:
        if (following_error_ok()) do_moving();
        break;
    }

    publish_state();
}

/* esp_timer callback (runs in the esp_timer task): wakes the control task
 * with a precise period, independent of the FreeRTOS tick rate. */
static void tick_cb(void *arg)
{
    xTaskNotifyGive(s_task);
}

static void control_task(void *arg)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        control_step();
    }
}

/* ======================================================================== */
/* Public API                                                                */
/* ======================================================================== */

static void tunables_default(tunables_t *t)
{
    const pid_gains_t g1 = { J1_KP_DEF, J1_KI_DEF, J1_KD_DEF, J1_KFF_DEF, J1_MIN_DUTY_DEF, J1_MAX_DUTY_DEF, DERIV_CUTOFF_HZ_DEF, POS_TOL_DEF };
    const pid_gains_t g2 = { J2_KP_DEF, J2_KI_DEF, J2_KD_DEF, J2_KFF_DEF, J2_MIN_DUTY_DEF, J2_MAX_DUTY_DEF, DERIV_CUTOFF_HZ_DEF, POS_TOL_DEF };
    t->pid[0] = g1;
    t->pid[1] = g2;
    t->vmax[0] = J1_VMAX_DEF;  t->amax[0] = J1_AMAX_DEF;  t->jog_vmax[0] = J1_JOG_VMAX_DEF;
    t->vmax[1] = J2_VMAX_DEF;  t->amax[1] = J2_AMAX_DEF;  t->jog_vmax[1] = J2_JOG_VMAX_DEF;
    t->pos_tol = POS_TOL_DEF;
    t->settle_ms = SETTLE_MS_DEF;
    t->p_up_ms = P_UP_MS_DEF;
    t->p_down_ms = P_DOWN_MS_DEF;
    t->follow_err_max = FOLLOW_ERR_MAX_DEF;
    t->elbow_pref = (int)ELBOW_PREF_DEF;
}

void control_start(void)
{
    s_in_q = xQueueCreate(8, sizeof(ctrl_input_t));
    s_ev_q = xQueueCreate(12, sizeof(event_t));
    tunables_default(&s_tun);

    static const char *const rst_names[] = {
        [ESP_RST_UNKNOWN] = "unknown", [ESP_RST_POWERON] = "power-on", [ESP_RST_EXT] = "external pin",
        [ESP_RST_SW] = "software", [ESP_RST_PANIC] = "PANIC (crash)", [ESP_RST_INT_WDT] = "interrupt watchdog",
        [ESP_RST_TASK_WDT] = "task watchdog", [ESP_RST_WDT] = "watchdog", [ESP_RST_DEEPSLEEP] = "deep sleep",
        [ESP_RST_BROWNOUT] = "BROWNOUT (supply dip)", [ESP_RST_SDIO] = "SDIO",
    };
    const esp_reset_reason_t rr = esp_reset_reason();
    const char *rn = (rr < sizeof(rst_names) / sizeof(rst_names[0]) && rst_names[rr]) ? rst_names[rr] : "other";
    if (rtc_magic == RTC_MAGIC && rr != ESP_RST_POWERON)
        event("BOOT: reset = %s; before it: mode %lu, home phase %lu, joint idx %lu",
              rn, (unsigned long)rtc_mode, (unsigned long)rtc_hphase, (unsigned long)rtc_home_idx);
    else
        event("BOOT: reset = %s", rn);
    rtc_magic = 0;
    tun = s_tun;

    read_sensors();
    sensors_reset();
    hold_here();
#if P_UP_ON_BOOT
    raise_p();
#endif
    set_valve(false);

    xTaskCreatePinnedToCore(control_task, "control", 6144, NULL, configMAX_PRIORITIES - 2, &s_task, 1);

    const esp_timer_create_args_t args = { .callback = tick_cb, .name = "ctrl_tick",
                                           .dispatch_method = ESP_TIMER_TASK };
    esp_timer_handle_t timer;
    ESP_ERROR_CHECK(esp_timer_create(&args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, CTRL_PERIOD_US));
}

void control_post_cmd(int cmd)
{
    ctrl_input_t in = { .type = IN_CMD, .cmd = cmd };
    xQueueSend(s_in_q, &in, 0);
}

void control_post_goto(float x, float y, float z)
{
    ctrl_input_t in = { .type = IN_GOTO, .x = x, .y = y, .z = z };
    xQueueSend(s_in_q, &in, 0);
}

void control_set_jog(float j1, float j2)
{
    j1 = fmaxf(-1.0f, fminf(1.0f, j1));
    j2 = fmaxf(-1.0f, fminf(1.0f, j2));
    taskENTER_CRITICAL(&s_lock);
    s_jog[0] = j1;
    s_jog[1] = j2;
    s_jog_ms = now_ms();
    taskEXIT_CRITICAL(&s_lock);
}

void control_set_deadman(bool pressed)
{
    taskENTER_CRITICAL(&s_lock);
    s_deadman = pressed;
    s_deadman_ms = now_ms();
    taskEXIT_CRITICAL(&s_lock);
}

void control_set_comm(bool connected)
{
    taskENTER_CRITICAL(&s_lock);
    s_comm = connected;
    taskEXIT_CRITICAL(&s_lock);
}

void control_set_param(int id, float v)
{
    taskENTER_CRITICAL(&s_lock);
    if (id < P_POS_TOL) {
        const int j = id / P_JOINT_STRIDE;
        pid_gains_t *g = &s_tun.pid[j];
        switch (id % P_JOINT_STRIDE) {
        case P_J1_KP:       g->kp = v; break;
        case P_J1_KI:       g->ki = v; break;
        case P_J1_KD:       g->kd = v; break;
        case P_J1_KFF:      g->kff = v; break;
        case P_J1_MIN_DUTY: g->min_duty = fmaxf(0.0f, fminf(v, 0.5f)); break;
        case P_J1_MAX_DUTY: g->max_duty = fmaxf(0.0f, fminf(v, 1.0f)); break;
        case P_J1_VMAX:     s_tun.vmax[j] = fmaxf(v, 1e-3f); break;
        case P_J1_AMAX:     s_tun.amax[j] = fmaxf(v, 1e-3f); break;
        case P_J1_JOG_VMAX: s_tun.jog_vmax[j] = fmaxf(v, 0.0f); break;
        default: break;
        }
    } else {
        switch (id) {
        case P_POS_TOL:         s_tun.pos_tol = fmaxf(v, 1e-4f); break;
        case P_SETTLE_MS:       s_tun.settle_ms = fmaxf(v, 0.0f); break;
        case P_P_UP_MS:         s_tun.p_up_ms = fmaxf(v, 0.0f); break;
        case P_P_DOWN_MS:       s_tun.p_down_ms = fmaxf(v, 0.0f); break;
        case P_DERIV_CUTOFF_HZ: s_tun.pid[0].deriv_cutoff_hz = s_tun.pid[1].deriv_cutoff_hz = v; break;
        case P_FOLLOW_ERR_MAX:  s_tun.follow_err_max = fmaxf(v, 0.01f); break;
        case P_ELBOW_PREF:      s_tun.elbow_pref = (v > 0.5f) ? 1 : (v < -0.5f ? -1 : 0); break;
        default: break;
        }
    }
    taskEXIT_CRITICAL(&s_lock);
}

void control_get_state(float out[ST_LEN])
{
    taskENTER_CRITICAL(&s_lock);
    memcpy(out, s_state, sizeof(s_state));
    taskEXIT_CRITICAL(&s_lock);
}

bool control_pop_event(char *buf, size_t len)
{
    event_t ev;
    if (xQueueReceive(s_ev_q, &ev, 0) != pdTRUE) return false;
    strncpy(buf, ev.text, len - 1);
    buf[len - 1] = '\0';
    return true;
}
