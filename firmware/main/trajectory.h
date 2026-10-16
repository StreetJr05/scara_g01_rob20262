/**
 * @file trajectory.h
 * @brief Synchronized quintic (minimum-jerk) joint-space trajectory.
 *
 *  q(t)  = q0 + dq * s(tau),  tau = t / T
 *  s     = 10 tau^3 - 15 tau^4 + 6 tau^5     (zero vel. and accel. at both ends)
 *  max |dq/dt|  = 1.875  |dq| / T
 *  max |d2q/dt2| = 5.7735 |dq| / T^2
 *  T = max over joints of the time that respects vmax and amax, so both
 *  joints start and finish together (straight-ish, predictable motion).
 */
#pragma once

#include <stdbool.h>
#include "hal.h"

typedef struct {
    float q0[NUM_JOINTS];
    float dq[NUM_JOINTS];
    float T;         /* s */
    float t;         /* s, elapsed */
    bool  active;
} traj_t;

void traj_plan(traj_t *tr, const float q0[NUM_JOINTS], const float qf[NUM_JOINTS],
               const float vmax[NUM_JOINTS], const float amax[NUM_JOINTS]);

/** Advance by ts and write the reference. Returns true while still running. */
bool traj_step(traj_t *tr, float ts, float q_ref[NUM_JOINTS], float dq_ref[NUM_JOINTS]);
