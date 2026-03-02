/**
 * @file mag_update.cpp
 * @brief Magnetometer measurement update cho ESKF15 — full 3D vector.
 *
 * Observation model:
 *   Từ trường thật trong body frame = R(q)^T * mag_earth_ned
 *   → h(x) = R^T * m_ned
 *
 *   z = mag_meas_body  (3D, µT)
 *   y = z - h(x)       (innovation)
 *
 * Jacobian H (3×15):
 *   ∂h/∂(δθ) = ∂(R^T * m)/∂(δθ)
 *
 *   Khi δθ nhỏ: R_perturbed ≈ R * (I + [δθ×])
 *   → R_perturbed^T ≈ (I - [δθ×]) * R^T
 *   → h(δθ) ≈ R^T*m - [δθ×]*R^T*m = R^T*m + [R^T*m]× * δθ
 *   → ∂h/∂(δθ) = [R^T * m_ned]× = skew(R^T * m_ned)
 *
 *   H = [0₃  0₃  skew(h_pred)  0₃  0₃]   (3×15)
 *
 * Vì sao dùng full 3D thay vì chỉ heading?
 * - BMM150 xuất 3D vector → tận dụng hết thông tin
 * - Heading-only bỏ phí thông tin inclination (roll/pitch aiding)
 * - Dùng gating để loại outlier khi từ trường bị nhiễu
 *
 * Lưu ý: Không có delay compensation cho MAG (delay ≈ 0 hoặc rất nhỏ).
 */

#include "eskf/eskf.hpp"

namespace eskf {

InnovationRecord ESKF15::update_mag(const MagSample& mag) {
    InnovationRecord rec;
    rec.clear();
    rec.sensor_id = 1;  // MAG
    rec.t = mag.t;
    rec.dim = 3;

    if (!mag.valid) {
        rec.accepted = false;
        return rec;
    }

    // ================================================================
    // Predicted measurement: h(x) = R^T * mag_earth_ned
    // ================================================================
    Mat3 R = rot_from_quat(state_.q);
    Mat3 Rt = R.T();  // NED → body
    Vec3 h_pred = Rt * mag_earth_ned;  // từ trường predicted trong body

    // ================================================================
    // Innovation: y = z - h(x)
    // ================================================================
    Vec3 y;
    y[0] = mag.mag_body[0] - h_pred[0];
    y[1] = mag.mag_body[1] - h_pred[1];
    y[2] = mag.mag_body[2] - h_pred[2];

    // ================================================================
    // Jacobian H (3×15)
    // H = [0₃  0₃  skew(h_pred)  0₃  0₃]
    // ================================================================
    Mat<3, STATE_DIM> H = Mat<3, STATE_DIM>::zero();
    Mat3 h_skew = skew(h_pred);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            H(i, 6+j) = h_skew(i, j);

    // ================================================================
    // R matrix (3×3) — measurement noise
    // ================================================================
    Mat3 Rm = Mat3::zero();
    double r_mag = MAG_NOISE * MAG_NOISE;
    Rm(0,0) = r_mag;  Rm(1,1) = r_mag;  Rm(2,2) = r_mag;

    // ================================================================
    // Innovation covariance: S = H * P * H^T + R
    // ================================================================
    Mat3 S = H * P_ * H.T() + Rm;

    // ================================================================
    // Gating: NIS = y^T * S^{-1} * y
    // ================================================================
    Mat3 S_inv;
    if (!invert3(S, S_inv)) {
        rec.accepted = false;
        mag_reject_++;
        return rec;
    }

    Mat<1,1> nis_mat = y.T() * S_inv * y;
    double nis = nis_mat[0];

    rec.nis = nis;
    for (int i = 0; i < 3; ++i) rec.innovation[i] = y[i];

    if (nis > MAG_GATE_CHI2) {
        rec.accepted = false;
        mag_reject_++;
        return rec;
    }

    // ================================================================
    // Kalman gain: K = P * H^T * S^{-1}   (15×3)
    // ================================================================
    Mat<STATE_DIM, 3> K = P_ * H.T() * S_inv;

    // ================================================================
    // Error-state update: δx = K * y
    // ================================================================
    Mat<STATE_DIM, 1> dx_full = Mat<STATE_DIM, 1>::zero();
    // K(15×3) * y(3×1) → dx(15×1)
    // Manual multiply vì K * y chỉ cho Mat<15,1>
    Mat<3,1> y_col;
    y_col[0] = y[0]; y_col[1] = y[1]; y_col[2] = y[2];
    dx_full = K * y_col;

    inject_error_state(dx_full);

    // ================================================================
    // Covariance update — Joseph form
    // P = (I-KH) * P * (I-KH)^T + K * R * K^T
    // ================================================================
    // Cần tính KH (15×15) = K(15×3) * H(3×15)
    PMatrix KH = PMatrix::zero();
    for (int i = 0; i < STATE_DIM; ++i)
        for (int j = 0; j < STATE_DIM; ++j) {
            double sum = 0;
            for (int k = 0; k < 3; ++k)
                sum += K(i, k) * H(k, j);
            KH(i, j) = sum;
        }

    PMatrix I_KH = PMatrix::identity() - KH;

    // KRKt = K * R * K^T (15×15)
    // K(15×3) * R(3×3) = KR(15×3), KR * K^T(3×15) = (15×15)
    Mat<STATE_DIM, 3> KR = K * Rm;
    PMatrix KRKt = PMatrix::zero();
    for (int i = 0; i < STATE_DIM; ++i)
        for (int j = 0; j < STATE_DIM; ++j) {
            double sum = 0;
            for (int k = 0; k < 3; ++k)
                sum += KR(i, k) * K(j, k);  // K^T(k,j) = K(j,k)
            KRKt(i, j) = sum;
        }

    P_ = I_KH * P_ * I_KH.T() + KRKt;
    regularize_P();

    rec.accepted = true;
    mag_accept_++;
    return rec;
}

}  // namespace eskf
