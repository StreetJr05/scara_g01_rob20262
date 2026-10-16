#include "pid.h"

#include <math.h>

void pid_reset(pid_state_t *s)
{
    s->integ = 0.0f;
    s->d_filt = 0.0f;
}

float pid_update(const pid_gains_t *g, pid_state_t *s,
                 float ref, float ref_rate, float y, float y_rate, float ts)
{
    const float e = ref - y;

    /* Filtered derivative of the error (dr - dy) */
    const float tau = 1.0f / (2.0f * (float)M_PI * fmaxf(g->deriv_cutoff_hz, 0.1f));
    const float a = tau / (tau + ts);
    s->d_filt = a * s->d_filt + (1.0f - a) * g->kd * (ref_rate - y_rate);

    /* In position and reference not moving: release the motor and keep the
     * integrator from winding up against static friction (avoids buzzing). */
    if (fabsf(e) < g->dead_band && fabsf(ref_rate) < 1e-4f) {
        return 0.0f;
    }

    float u_unsat = g->kp * e + s->integ + s->d_filt + g->kff * ref_rate;

    /* Static friction compensation */
    if (u_unsat > 0.0f) {
        u_unsat += g->min_duty;
    } else if (u_unsat < 0.0f) {
        u_unsat -= g->min_duty;
    }

    float u = u_unsat;
    if (u > g->max_duty) u = g->max_duty;
    if (u < -g->max_duty) u = -g->max_duty;

    /* Conditional integration: integrate unless saturated in the same
     * direction as the error. */
    const bool saturated = (u != u_unsat);
    if (!saturated || (u_unsat * e) < 0.0f) {
        s->integ += g->ki * ts * e;
        /* hard clamp so the integral alone can never exceed the max duty */
        if (s->integ > g->max_duty) s->integ = g->max_duty;
        if (s->integ < -g->max_duty) s->integ = -g->max_duty;
    }

    return u;
}
