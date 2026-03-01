/**
 * @file covariance.cpp
 * @brief ESKF covariance propagation  P = F P F^T + Q
 *
 * Optimized for 15×15 fixed-size matrices on embedded targets.
 * Uses temporary stack array — no heap allocation.
 */
#include "ekf.h"

namespace ekf2 {

void Ekf::propagateCovariance(const Mat15& F) {
    // Tmp = F * P  (15×15)
    Mat15 tmp;
    for (int i = 0; i < STATE_N; i++)
        for (int j = 0; j < STATE_N; j++) {
            float s = 0.f;
            for (int k = 0; k < STATE_N; k++)
                s += F(i,k) * P_(k,j);
            tmp(i,j) = s;
        }

    // P = Tmp * F^T + Q  (F^T means swap i,j indices of F)
    for (int i = 0; i < STATE_N; i++)
        for (int j = 0; j < STATE_N; j++) {
            float s = 0.f;
            for (int k = 0; k < STATE_N; k++)
                s += tmp(i,k) * F(j,k);   // F^T(k,j) = F(j,k)
            P_(i,j) = s + Q_(i,j);
        }
}

}  // namespace ekf2

// ---------------------------------------------------------------------------
// Barometer update  (separate TU, included here for brevity)
// ---------------------------------------------------------------------------
// In a production system this would live in baro_update.cpp
#include "ekf.h"

namespace ekf2 {

void Ekf::baroUpdate(const BaroSample& baro) {
    if (!baro.valid) return;

    // H (1×15): z_meas = -p_ned_z  →  d/d(δp_z) = -1
    float H[STATE_N] = {};
    H[2] = -1.f;

    float z_pred  = -nom_.p[2];
    float innov   = baro.alt - z_pred;

    // Inflated R: sensor noise + unmodeled baro bias
    float R_baro  = par_.sigma_baro * par_.sigma_baro
                  + par_.sigma_baro_bias * par_.sigma_baro_bias;

    // S = H P H^T + R  (scalar)
    float S = R_baro;
    for (int i = 0; i < STATE_N; i++)
        for (int j = 0; j < STATE_N; j++)
            S += H[i] * P_(i,j) * H[j];

    if (S < 1e-8f) return;

    // K = P H^T / S   (15×1)
    float K[STATE_N] = {};
    for (int i = 0; i < STATE_N; i++) {
        float v = 0.f;
        for (int j = 0; j < STATE_N; j++) v += P_(i,j) * H[j];
        K[i] = v / S;
    }

    // dx = K * innov
    float dx[STATE_N] = {};
    for (int i = 0; i < STATE_N; i++) dx[i] = K[i] * innov;

    // NaN check
    bool ok = true;
    for (int i = 0; i < STATE_N; i++)
        if (dx[i] != dx[i]) { ok = false; break; }
    if (!ok) return;

    // Joseph form: P = (I - K H) P (I - K H)^T + K R K^T
    // For scalar measurement: IKH(i,j) = I(i,j) - K[i]*H[j]
    Mat15 P_new;
    P_new.zero();
    for (int i = 0; i < STATE_N; i++)
        for (int j = 0; j < STATE_N; j++) {
            float s = 0.f;
            for (int a = 0; a < STATE_N; a++)
                for (int b = 0; b < STATE_N; b++) {
                    float ikh_ia = ((i==a) ? 1.f : 0.f) - K[i]*H[a];
                    float ikh_jb = ((j==b) ? 1.f : 0.f) - K[j]*H[b];
                    s += ikh_ia * P_(a,b) * ikh_jb;
                }
            P_new(i,j) = s + K[i] * R_baro * K[j];
        }
    P_ = P_new;

    injectError(dx);
    symmetrizeP();
}

}  // namespace ekf2
