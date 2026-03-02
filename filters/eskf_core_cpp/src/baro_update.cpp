/**
 * @file baro_update.cpp
 * @brief Barometer measurement update cho ESKF15 — altitude (1D).
 *
 * Observation model:
 *   h(x) = -p_D   (altitude = -down, vì NED: D dương hướng xuống)
 *   z = baro_alt   (dương hướng lên)
 *
 *   y = z - h(x) = baro_alt - (-p_D) = baro_alt + p_D
 *
 * Jacobian H (1×15):
 *   H = [0 0 -1  0 0 0  0 0 0  0 0 0  0 0 0]
 *       (chỉ ∂h/∂p_D = ∂(-p_D)/∂(δp_D) = -1)
 *
 * R = σ²_baro  (scalar)
 *
 * Baro không có delay đáng kể → update trực tiếp.
 */

#include "eskf/eskf.hpp"

namespace eskf {

InnovationRecord ESKF15::update_baro(const BaroSample& baro) {
    InnovationRecord rec;
    rec.clear();
    rec.sensor_id = 2;  // BARO
    rec.t = baro.t;
    rec.dim = 1;

    if (!baro.valid) {
        rec.accepted = false;
        return rec;
    }

    // ================================================================
    // Predicted measurement: h(x) = -p_D
    // ================================================================
    double h_pred = -state_.p[2];  // altitude = -down

    // ================================================================
    // Innovation: y = z - h(x)
    // ================================================================
    double y_val = baro.alt - h_pred;

    // ================================================================
    // H matrix (1×15)
    // ∂(-p_D)/∂(δp) = [0, 0, -1]
    // ================================================================
    Mat<1, STATE_DIM> H = Mat<1, STATE_DIM>::zero();
    H(0, 2) = -1.0;  // ∂h/∂(δp_D) = -1

    // ================================================================
    // R matrix (1×1)
    // ================================================================
    Mat<1,1> R;
    R(0,0) = BARO_NOISE * BARO_NOISE;

    // ================================================================
    // S = H * P * H^T + R  (1×1)
    // ================================================================
    Mat<1,1> S = H * P_ * H.T() + R;

    // ================================================================
    // Gating
    // ================================================================
    Mat<1,1> S_inv;
    if (!invert1(S, S_inv)) {
        rec.accepted = false;
        return rec;
    }

    double nis = y_val * y_val * S_inv(0,0);

    rec.nis = nis;
    rec.innovation[0] = y_val;

    if (nis > BARO_GATE_CHI2) {
        rec.accepted = false;
        return rec;
    }

    // ================================================================
    // Kalman gain: K = P * H^T * S^{-1}  (15×1)
    // ================================================================
    Mat<STATE_DIM, 1> K = P_ * H.T() * S_inv;

    // ================================================================
    // δx = K * y
    // ================================================================
    Mat<STATE_DIM, 1> dx = K * y_val;
    inject_error_state(dx);

    // ================================================================
    // Joseph form: P = (I-KH) * P * (I-KH)^T + K * R * K^T
    // ================================================================

    // KH (15×15) = K(15×1) * H(1×15)
    PMatrix KH = PMatrix::zero();
    for (int i = 0; i < STATE_DIM; ++i)
        for (int j = 0; j < STATE_DIM; ++j)
            KH(i,j) = K(i,0) * H(0,j);

    PMatrix I_KH = PMatrix::identity() - KH;

    // KRKt = K * R(0,0) * K^T
    PMatrix KRKt = PMatrix::zero();
    double kr = R(0,0);
    for (int i = 0; i < STATE_DIM; ++i)
        for (int j = 0; j < STATE_DIM; ++j)
            KRKt(i,j) = K(i,0) * kr * K(j,0);

    P_ = I_KH * P_ * I_KH.T() + KRKt;
    regularize_P();

    rec.accepted = true;
    return rec;
}

}  // namespace eskf
