#include "trajectory.h"

#include <math.h>

#define TRAJ_T_MIN 0.2f

void traj_plan(traj_t *tr, const float q0[NUM_JOINTS], const float qf[NUM_JOINTS],
               const float vmax[NUM_JOINTS], const float amax[NUM_JOINTS])
{
    float T = TRAJ_T_MIN;
    for (int j = 0; j < NUM_JOINTS; j++) {
        tr->q0[j] = q0[j];
        tr->dq[j] = qf[j] - q0[j];
        const float d = fabsf(tr->dq[j]);
        const float t_v = 1.875f * d / fmaxf(vmax[j], 1e-3f);
        const float t_a = sqrtf(5.7735f * d / fmaxf(amax[j], 1e-3f));
        T = fmaxf(T, fmaxf(t_v, t_a));
    }
    tr->T = T;
    tr->t = 0.0f;
    tr->active = true;
}

bool traj_step(traj_t *tr, float ts, float q_ref[NUM_JOINTS], float dq_ref[NUM_JOINTS])
{
    if (!tr->active) {
        return false;
    }
    tr->t += ts;
    float tau = tr->t / tr->T;
    if (tau >= 1.0f) {
        tau = 1.0f;
        tr->active = false;
    }
    const float tau2 = tau * tau, tau3 = tau2 * tau;
    const float s = tau3 * (10.0f - 15.0f * tau + 6.0f * tau2);
    const float ds = tau2 * (30.0f - 60.0f * tau + 30.0f * tau2) / tr->T;
    for (int j = 0; j < NUM_JOINTS; j++) {
        q_ref[j] = tr->q0[j] + tr->dq[j] * s;
        dq_ref[j] = tr->active ? tr->dq[j] * ds : 0.0f;
    }
    return tr->active;
}
