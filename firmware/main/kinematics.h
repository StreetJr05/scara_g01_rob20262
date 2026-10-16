/**
 * @file kinematics.h
 * @brief Forward / inverse kinematics of the RRP SCARA.
 *
 *  x = L1 cos(q1) + L2 cos(q1 + q2)
 *  y = L1 sin(q1) + L2 sin(q1 + q2)
 *  z = TOOL_Z_UP  (P up)  |  TOOL_Z_DOWN (P down)       (binary prismatic)
 *
 * See docs/cinematica_inversa.md for the derivation.
 */
#pragma once

#include <stdbool.h>
#include "protocol.h"

typedef struct {
    float q1, q2;      /* rad */
    bool  p_down;      /* prismatic joint: true = down */
    int   elbow;       /* +1: q2 >= 0, -1: q2 < 0 */
    bool  alt_valid;   /* the other elbow solution was also feasible */
} ik_solution_t;

void fk_xy(float q1, float q2, float *x, float *y);

/**
 * @param x, y, z     requested tool position [m]
 * @param q1_now, q2_now  current joint angles (used to choose the solution)
 * @param elbow_pref  0 = least joint motion, +1 / -1 = force the sign of q2
 * @param out         solution (valid only if ERR_NONE)
 * @return ERR_NONE, ERR_UNREACHABLE, ERR_JOINT_LIMIT or ERR_SINGULAR
 */
scara_err_t ik_solve(float x, float y, float z,
                     float q1_now, float q2_now, int elbow_pref,
                     ik_solution_t *out);

bool joint_in_limits(int joint, float q);
