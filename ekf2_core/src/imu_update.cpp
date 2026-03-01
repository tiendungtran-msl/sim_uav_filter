/**
 * @file imu_update.cpp
 * @brief ESKF IMU predict step.
 *
 * Implements:
 *   1. Nominal state propagation
 *   2. Error-state Jacobian F construction
 *   3. Covariance propagation  P = F P F^T + Q
 */
#include "ekf.h"
#include <string.h>

namespace ekf2 {

// ---------------------------------------------------------------------------
static inline float skew_ij(const float v[3], int i, int j) {
    //  [  0  -v2  v1 ]
    //  [ v2   0  -v0 ]
    //  [-v1  v0   0  ]
    const float s[3][3] = {
        { 0.f,    -v[2],  v[1] },
        { v[2],   0.f,   -v[0] },
        {-v[1],   v[0],   0.f  }
    };
    return s[i][j];
}

// ---------------------------------------------------------------------------
void Ekf::imuUpdate(const ImuSample& imu) {
    const float dt = par_.dt;

    // ── Corrected measurements ────────────────────────────────────────────
    float om[3] = {
        imu.gyr[0] - nom_.bg[0],
        imu.gyr[1] - nom_.bg[1],
        imu.gyr[2] - nom_.bg[2],
    };
    float am[3] = {
        imu.acc[0] - nom_.ba[0],
        imu.acc[1] - nom_.ba[1],
        imu.acc[2] - nom_.ba[2],
    };

    // ── Rotation matrix R (body → NED) ────────────────────────────────────
    Mat3 R;
    nom_.q.toR(R);

    // ── Specific force in NED ─────────────────────────────────────────────
    float f_ned[3];
    nom_.q.rotate(am, f_ned);

    // Store estimated NED acceleration (R·am + g) for adaptive GPS velocity R
    a_ned_[0] = f_ned[0];
    a_ned_[1] = f_ned[1];
    a_ned_[2] = f_ned[2] + par_.gravity;

    // ── Nominal state propagation ─────────────────────────────────────────
    // p ← p + v·dt
    nom_.p[0] += nom_.v[0] * dt;
    nom_.p[1] += nom_.v[1] * dt;
    nom_.p[2] += nom_.v[2] * dt;

    // v ← v + (R·am + g)·dt
    nom_.v[0] += (f_ned[0])                   * dt;
    nom_.v[1] += (f_ned[1])                   * dt;
    nom_.v[2] += (f_ned[2] + par_.gravity)    * dt;

    // q ← q ⊗ exp(om·dt)
    float phi[3] = {om[0]*dt, om[1]*dt, om[2]*dt};
    Quat dq = rotvec_to_quat(phi);
    nom_.q = nom_.q * dq;
    nom_.q.normalise();

    // bias unchanged (random walk propagated in Q)

    nom_.t += dt;

    // ── F matrix (15×15, identity + perturbations) ────────────────────────
    // Row blocks: 0-2=δp, 3-5=δv, 6-8=δθ, 9-11=δbg, 12-14=δba
    Mat15 F;
    F.identity();

    // δp += δv · dt
    for (int i = 0; i < 3; i++) F(i, 3+i) = dt;

    // δv += -R·[am×]·δθ · dt
    //      -R·δba · dt
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            // -R * skew(am)
            float val = 0.f;
            for (int k = 0; k < 3; k++)
                val += R(i,k) * skew_ij(am, k, j);
            F(3+i, 6+j) = -val * dt;

            // -R (for ba)
            F(3+i, 12+j) = -R(i,j) * dt;
        }

    // δθ += -[om×]·δθ · dt  →  F(6..8, 6..8) = I - skew(om)*dt
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            F(6+i, 6+j) = (i==j ? 1.f : 0.f) - skew_ij(om, i, j) * dt;

    // δθ += -δbg · dt
    for (int i = 0; i < 3; i++) F(6+i, 9+i) = -dt;

    // ── Covariance propagation P = F·P·F^T + Q ────────────────────────────
    propagateCovariance(F);
    symmetrizeP();

    // ── Save to ring buffer ───────────────────────────────────────────────
    saveBuffer();
}

}  // namespace ekf2
