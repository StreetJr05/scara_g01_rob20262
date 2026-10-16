/**
 * @file kinematics.c
 * @brief Closed-form (geometric) IK for the planar 2R part + binary P axis.
 *
 * 1. Law of cosines:   c2 = (x^2 + y^2 - L1^2 - L2^2) / (2 L1 L2)
 *    |c2| > 1        -> point outside the annulus |L1-L2| <= r <= L1+L2
 * 2. Two solutions:    q2 = +acos(c2)  (elbow +)   or   q2 = -acos(c2) (elbow -)
 * 3. Shoulder:         q1 = atan2(y, x) - atan2(L2 s2, L1 + L2 c2)
 * 4. Each candidate is wrapped into the joint range and checked against the
 *    soft limits. Selection policy: forced elbow if requested, otherwise the
 *    feasible one with the smallest max(|dq1|, |dq2|) (the motion time of a
 *    synchronized trajectory is dominated by the largest joint displacement).
 * 5. Singularities: det(J) = L1 L2 sin(q2) = 0  -> q2 = 0 (arm stretched,
 *    workspace outer boundary) or q2 = pi (folded, inner boundary). Points
 *    with |sin q2| < IK_SINGULAR_EPS are rejected: near them small Cartesian
 *    errors require huge joint motions. r = 0 is the shoulder singularity
 *    (q1 undefined), only reachable when L1 == L2.
 * 6. z selects the P state: z below the midpoint between TOOL_Z_UP and
 *    TOOL_Z_DOWN -> P down.
 */
#include "kinematics.h"
#include "scara_config.h"

#include <math.h>

static const float s_qmin[2] = { J1_MIN_RAD, J2_MIN_RAD };
static const float s_qmax[2] = { J1_MAX_RAD, J2_MAX_RAD };

void fk_xy(float q1, float q2, float *x, float *y)
{
    *x = L1_M * cosf(q1) + L2_M * cosf(q1 + q2);
    *y = L1_M * sinf(q1) + L2_M * sinf(q1 + q2);
}

bool joint_in_limits(int joint, float q)
{
    return q >= s_qmin[joint] && q <= s_qmax[joint];
}

/* Choose q + 2k*pi inside the limits, closest to q_now. Returns false if none. */
static bool wrap_into_limits(int joint, float q, float q_now, float *out)
{
    const float two_pi = 2.0f * (float)M_PI;
    bool found = false;
    float best = 0.0f;
    for (int k = -1; k <= 1; k++) {
        float c = q + k * two_pi;
        if (joint_in_limits(joint, c) && (!found || fabsf(c - q_now) < fabsf(best - q_now))) {
            best = c;
            found = true;
        }
    }
    *out = best;
    return found;
}

scara_err_t ik_solve(float x, float y, float z,
                     float q1_now, float q2_now, int elbow_pref,
                     ik_solution_t *out)
{
    const float r2 = x * x + y * y;
    const float r = sqrtf(r2);

    if (r > L1_M + L2_M || r < fabsf(L1_M - L2_M)) {
        return ERR_UNREACHABLE;
    }

    float c2 = (r2 - L1_M * L1_M - L2_M * L2_M) / (2.0f * L1_M * L2_M);
    if (c2 > 1.0f) c2 = 1.0f;     /* numerical noise on the boundary */
    if (c2 < -1.0f) c2 = -1.0f;
    const float s2_abs = sqrtf(1.0f - c2 * c2);

    if (s2_abs < IK_SINGULAR_EPS) {
        return ERR_SINGULAR;
    }

    struct { float q1, q2; bool ok; } cand[2];
    const int elbow_sign[2] = { +1, -1 };
    for (int i = 0; i < 2; i++) {
        float s2 = elbow_sign[i] * s2_abs;
        float q2 = atan2f(s2, c2);
        float q1 = atan2f(y, x) - atan2f(L2_M * s2, L1_M + L2_M * c2);
        cand[i].ok = wrap_into_limits(0, q1, q1_now, &cand[i].q1) &&
                     wrap_into_limits(1, q2, q2_now, &cand[i].q2);
    }

    int pick = -1;
    if (elbow_pref > 0 && cand[0].ok) {
        pick = 0;
    } else if (elbow_pref < 0 && cand[1].ok) {
        pick = 1;
    } else if (cand[0].ok && cand[1].ok) {
        float d0 = fmaxf(fabsf(cand[0].q1 - q1_now), fabsf(cand[0].q2 - q2_now));
        float d1 = fmaxf(fabsf(cand[1].q1 - q1_now), fabsf(cand[1].q2 - q2_now));
        pick = (d0 <= d1) ? 0 : 1;
    } else if (cand[0].ok) {
        pick = 0;
    } else if (cand[1].ok) {
        pick = 1;
    }

    if (pick < 0) {
        return ERR_JOINT_LIMIT;
    }

    out->q1 = cand[pick].q1;
    out->q2 = cand[pick].q2;
    out->elbow = elbow_sign[pick];
    out->alt_valid = cand[1 - pick].ok;
    out->p_down = z < 0.5f * (TOOL_Z_UP_M + TOOL_Z_DOWN_M);
    return ERR_NONE;
}
