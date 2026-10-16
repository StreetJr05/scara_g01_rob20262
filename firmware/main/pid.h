/**
 * @file pid.h
 * @brief Discrete PID position controller with:
 *  - backward-Euler integral with conditional-integration anti-windup,
 *  - derivative on (ref_rate - measured_rate), first-order low-pass filtered
 *    (no derivative kick when a new target arrives),
 *  - velocity feed-forward,
 *  - static-friction compensation (min_duty) outside the dead band,
 *  - output saturation.
 *
 *   e[k] = r[k] - y[k]
 *   I[k] = I[k-1] + Ki * Ts * e[k]           (frozen while saturated & pushing further)
 *   D[k] = a D[k-1] + (1 - a) Kd (dr[k] - dy[k]),  a = tau / (tau + Ts), tau = 1 / (2 pi fc)
 *   u[k] = sat( Kp e[k] + I[k] + D[k] + Kff dr[k] )
 */
#pragma once

#include <stdbool.h>

typedef struct {
    float kp, ki, kd, kff;
    float min_duty, max_duty;
    float deriv_cutoff_hz;
    float dead_band;       /* |e| below this and no ref motion -> u = 0 */
} pid_gains_t;

typedef struct {
    float integ;
    float d_filt;
} pid_state_t;

void  pid_reset(pid_state_t *s);

/**
 * @param ref       reference position [rad]
 * @param ref_rate  reference velocity [rad/s] (trajectory / jog)
 * @param y         measured position [rad]
 * @param y_rate    measured velocity [rad/s]
 * @param ts        sample time [s]
 * @return duty [-max_duty, max_duty]
 */
float pid_update(const pid_gains_t *g, pid_state_t *s,
                 float ref, float ref_rate, float y, float y_rate, float ts);
