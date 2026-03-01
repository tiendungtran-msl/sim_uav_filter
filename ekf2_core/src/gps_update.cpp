/**
 * @file gps_update.cpp
 * @brief ESKF GPS (position + velocity) measurement update with delay compensation.
 *
 * Algorithm  (mirrors Python eskf.py):
 *   1. Position innovation: GPS_pos(t_stamp) – buffered_pos(t_stamp)
 *   2. Velocity innovation: GPS_vel(t_stamp) – CURRENT velocity
 *      (buffer not used for velocity because it omits prior GPS corrections)
 *   3. Adaptive R_gps_v: inflated by (|a_manouvre|·delay)² to absorb timing mismatch
 *   4. Innovation gating: reject if NIS > 140
 *   5. Joseph-form covariance update for numerical stability
 *   6. Apply dx to current nominal state
 */
#include "ekf.h"
#include <string.h>

namespace ekf2 {

// ---------------------------------------------------------------------------
// Simple 6×6 matrix inverse via Gauss-Jordan elimination
// ---------------------------------------------------------------------------
static bool invert6(const Mat<6,6>& A, Mat<6,6>& Ainv) {
    float a[6][12];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) a[i][j] = A(i,j);
        for (int j = 0; j < 6; j++) a[i][6+j] = (i==j) ? 1.f : 0.f;
    }
    for (int col = 0; col < 6; col++) {
        int piv = col;
        for (int r = col+1; r < 6; r++)
            if (fabsf(a[r][col]) > fabsf(a[piv][col])) piv = r;
        if (piv != col)
            for (int j = 0; j < 12; j++) { float t=a[col][j]; a[col][j]=a[piv][j]; a[piv][j]=t; }
        float d = a[col][col];
        if (fabsf(d) < 1e-12f) return false;
        for (int j = 0; j < 12; j++) a[col][j] /= d;
        for (int r = 0; r < 6; r++) {
            if (r == col) continue;
            float f = a[r][col];
            for (int j = 0; j < 12; j++) a[r][j] -= f * a[col][j];
        }
    }
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            Ainv(i,j) = a[i][6+j];
    return true;
}

// ---------------------------------------------------------------------------
void Ekf::gpsUpdate(const GpsSample& gps) {
    if (!gps.valid) return;

    // ── 1. Buffered position at t_stamp ────────────────────────────────────
    int bidx = findBuffered(gps.t_stamp);
    const float* p_ref = (bidx >= 0) ? buf_[bidx].nom.p : nom_.p;

    // ── 2. H matrix (6×15): pos then vel ──────────────────────────────────
    Mat<6,15> H;
    H.zero();
    for (int i = 0; i < 3; i++) {
        H(i,   i)   = 1.f;   // δp
        H(3+i, 3+i) = 1.f;   // δv
    }

    // ── 3. Innovation ─────────────────────────────────────────────────────
    // Position: compare GPS to buffered (delay-compensated)
    // Velocity: compare GPS to CURRENT nominal (no replay available)
    float innov[6];
    for (int i = 0; i < 3; i++) {
        innov[i]   = gps.pos[i] - p_ref[i];
        innov[3+i] = gps.vel[i] - nom_.v[i];
    }

    // ── 4. Adaptive R for velocity ────────────────────────────────────────
    // During manoeuvres, delay causes systematic velocity bias ≈ |a_man|·Δt
    float sp2 = par_.sigma_gps_pos * par_.sigma_gps_pos;
    float sv2 = par_.sigma_gps_vel * par_.sigma_gps_vel;
    float R_diag[6];
    for (int i = 0; i < 3; i++) R_diag[i] = sp2;
    for (int i = 0; i < 3; i++) {
        float a_man = a_ned_[i] - ((i==2) ? par_.gravity : 0.f);
        float bias  = a_man * par_.gps_delay;
        R_diag[3+i] = sv2 + bias * bias;
    }

    // ── 5. Innovation covariance S = H P H^T + R ──────────────────────────
    Mat<15,6> PHt = P_ * H.T();
    Mat<6,6>  S   = H * PHt;
    for (int i = 0; i < 6; i++) S(i,i) += R_diag[i];

    // ── 6. Innovation gating: reject if NIS > 140 ─────────────────────────
    Mat<6,6> Sinv;
    if (!invert6(S, Sinv)) return;

    float nis = 0.f;
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            nis += innov[i] * Sinv(i,j) * innov[j];
    if (nis > 140.f) return;

    // ── 7. Kalman gain K = P H^T S^{-1} ──────────────────────────────────
    Mat<15,6> K = PHt * Sinv;

    // ── 8. Error state correction dx = K · innov ──────────────────────────
    float dx[STATE_N] = {};
    for (int i = 0; i < STATE_N; i++)
        for (int j = 0; j < 6; j++)
            dx[i] += K(i,j) * innov[j];

    // Check for NaN
    bool ok = true;
    for (int i = 0; i < STATE_N; i++)
        if (dx[i] != dx[i]) { ok = false; break; }
    if (!ok) return;

    // ── 9. Joseph-form covariance update ──────────────────────────────────
    // P = (I-KH) P (I-KH)^T + K R K^T
    Mat15 KH;
    KH.zero();
    for (int i = 0; i < STATE_N; i++)
        for (int j = 0; j < STATE_N; j++)
            for (int k = 0; k < 6; k++)
                KH(i,j) += K(i,k) * H(k,j);

    Mat15 IKH;
    IKH.identity();
    for (int i = 0; i < STATE_N; i++)
        for (int j = 0; j < STATE_N; j++)
            IKH(i,j) -= KH(i,j);

    // tmp = IKH * P_ * IKH^T
    Mat15 tmp = IKH * P_;
    Mat15 IKHt; // transpose of IKH
    for (int i = 0; i < STATE_N; i++)
        for (int j = 0; j < STATE_N; j++)
            IKHt(i,j) = IKH(j,i);
    Mat15 P_joseph = tmp * IKHt;

    // + K * R * K^T
    // R is diagonal → K * R * K^T = sum_j (K[:,j] * R[j] * K[:,j]^T)
    for (int i = 0; i < STATE_N; i++)
        for (int j = 0; j < STATE_N; j++)
            for (int m = 0; m < 6; m++)
                P_joseph(i,j) += K(i,m) * R_diag[m] * K(j,m);

    P_ = P_joseph;

    // ── 10. Apply dx and symmetrise ───────────────────────────────────────
    injectError(dx);
    symmetrizeP();
}

}  // namespace ekf2
