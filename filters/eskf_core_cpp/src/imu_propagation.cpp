/**
 * @file imu_propagation.cpp
 * @brief IMU propagation — predict step cho ESKF15.
 *
 * Đây là bước chạy ở tần số cao nhất (400Hz cho ICM42688P).
 *
 * Mô hình ESKF15 error-state dynamics:
 *
 *   δp_dot = δv
 *   δv_dot = -R * [a_corrected×] * δθ  -  R * δba  +  noise_accel
 *   δθ_dot = -[ω_corrected×] * δθ  -  δbg  +  noise_gyro
 *   δbg_dot = noise_bg_rw    (random walk)
 *   δba_dot = noise_ba_rw    (random walk)
 *
 * Trong đó:
 *   a_corrected = accel_meas - ba  (specific force đã trừ bias)
 *   ω_corrected = gyro_meas - bg   (angular rate đã trừ bias)
 *   R = rot_from_quat(q)           (rotation body→NED hiện tại)
 *   [v×] = skew-symmetric matrix
 *
 * F matrix (15×15) — Jacobian rời rạc hoá (Euler):
 *   F = I + Fc * dt
 *
 * Q matrix (15×15) — process noise:
 *   Q = G * Qc * G^T * dt
 *   với Qc = diag(σ²_gyro, σ²_accel, σ²_bg_rw, σ²_ba_rw)
 *
 * Propagation:
 *   P = F * P * F^T + Q
 */

#include "eskf/eskf.hpp"

namespace eskf {

/**
 * Propagate state và covariance cho 1 bước IMU.
 *
 * ĐÂY là hàm "nóng" nhất — chạy 400 lần/giây.
 * Cần tối ưu: tránh tạo object tạm không cần thiết.
 */
void ESKF15::propagate_one_step(const ImuSample& imu) {
    double dt = imu.dt;
    if (dt <= 0 || dt > 0.1) dt = IMU_DT;  // Safety guard

    // ================================================================
    // 1) Trừ bias từ IMU measurement
    // ================================================================
    Vec3 omega;  // ω = gyro - bg
    omega[0] = imu.gyro[0] - state_.bg[0];
    omega[1] = imu.gyro[1] - state_.bg[1];
    omega[2] = imu.gyro[2] - state_.bg[2];

    Vec3 accel;  // a = accel_meas - ba
    accel[0] = imu.accel[0] - state_.ba[0];
    accel[1] = imu.accel[1] - state_.ba[1];
    accel[2] = imu.accel[2] - state_.ba[2];

    // ================================================================
    // 2) Ma trận xoay hiện tại R: body → NED
    // ================================================================
    Mat3 R = rot_from_quat(state_.q);

    // ================================================================
    // 3) Cập nhật nominal state
    // ================================================================

    // Specific force trong NED: f_ned = R * a_body
    Vec3 f_ned = R * accel;

    // Gravity NED: [0, 0, g] (g dương, down = +Z trong NED)
    Vec3 g_ned = Vec3::zero();
    g_ned[2] = GRAVITY;

    // v_new = v + (f_ned + g_ned) * dt
    //   (Newton's law: accel_NED = specific_force + gravity)
    state_.v[0] += (f_ned[0] + g_ned[0]) * dt;
    state_.v[1] += (f_ned[1] + g_ned[1]) * dt;
    state_.v[2] += (f_ned[2] + g_ned[2]) * dt;

    // p_new = p + v * dt  (dùng v mới — semi-implicit Euler)
    state_.p[0] += state_.v[0] * dt;
    state_.p[1] += state_.v[1] * dt;
    state_.p[2] += state_.v[2] * dt;

    // q_new = q ⊗ δq(ω * dt)
    Vec3 rotvec;
    rotvec[0] = omega[0] * dt;
    rotvec[1] = omega[1] * dt;
    rotvec[2] = omega[2] * dt;
    Vec4 dq = quat_from_rotvec(rotvec);
    state_.q = quat_normalize(quat_mult(state_.q, dq));

    // Bias: giữ nguyên (random walk ở Q, không đổi trong predict)
    // bg, ba không đổi

    // Timestamp
    state_.t = imu.t;

    // ================================================================
    // 4) Xây dựng F matrix (15×15)
    // ================================================================
    //
    // Error-state dynamics (xem mô tả ở đầu file):
    //
    //  F = I + Fc * dt, trong đó Fc (continuous-time):
    //
    //  Fc = | 0   I   0         0    0   |  (δp row)
    //       | 0   0   -R[a×]    0   -R   |  (δv row)
    //       | 0   0   -[ω×]    -I    0   |  (δθ row)
    //       | 0   0    0         0    0   |  (δbg row)
    //       | 0   0    0         0    0   |  (δba row)
    //
    //  Index: p=0:3, v=3:6, θ=6:9, bg=9:12, ba=12:15

    FMatrix F = FMatrix::identity();

    // F(p, v) = I * dt  → block (0,3)
    F(0, 3) = dt;  F(1, 4) = dt;  F(2, 5) = dt;

    // F(v, θ) = -R * [a×] * dt  → block (3,6)
    Mat3 Ra_skew = R * skew(accel);  // R * [a×]
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            F(3+i, 6+j) = -Ra_skew(i,j) * dt;

    // F(v, ba) = -R * dt  → block (3,12)
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            F(3+i, 12+j) = -R(i,j) * dt;

    // F(θ, θ) = I - [ω×] * dt  → block (6,6)
    Mat3 omega_skew = skew(omega);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            F(6+i, 6+j) = ((i==j) ? 1.0 : 0.0) - omega_skew(i,j) * dt;

    // F(θ, bg) = -I * dt  → block (6,9)
    F(6, 9) = -dt;  F(7, 10) = -dt;  F(8, 11) = -dt;

    // ================================================================
    // 5) Xây dựng Q matrix (process noise)
    // ================================================================
    //
    // Noise spectral densities → discrete Q:
    //   σ²_gyro  = (GYRO_NOISE_PSD)² * (1/dt)    → rad²/s² per sample
    //   σ²_accel = (ACCEL_NOISE_PSD)² * (1/dt)
    //   σ²_bg_rw = (GYRO_BIAS_RW)² * dt
    //   σ²_ba_rw = (ACCEL_BIAS_RW)² * dt
    //
    // Q đơn giản hoá (diagonal approximation):
    //   Q(v,v)   = R * σ²_a * I * R^T * dt²  (noise accel qua R)
    //   Q(θ,θ)   = σ²_g * I * dt²              (noise gyro)
    //   Q(bg,bg) = σ²_bg_rw_d * I
    //   Q(ba,ba) = σ²_ba_rw_d * I

    // Variance per IMU sample (discrete)
    double var_gyro  = GYRO_NOISE_PSD * GYRO_NOISE_PSD / dt;   // (rad/s)²
    double var_accel = ACCEL_NOISE_PSD * ACCEL_NOISE_PSD / dt;  // (m/s²)²
    double var_bg_rw = GYRO_BIAS_RW * GYRO_BIAS_RW * dt;        // (rad/s)²
    double var_ba_rw = ACCEL_BIAS_RW * ACCEL_BIAS_RW * dt;      // (m/s²)²

    PMatrix Q = PMatrix::zero();

    // Q(v,v) = R * diag(var_accel) * R^T * dt²
    //        = var_accel * (R * R^T) * dt²
    //        = var_accel * I * dt²  (vì R là orthogonal)
    // Tuy nhiên, chính xác hơn: Q_vv = R * Qa * R^T * dt² (nếu Qa không isotropic)
    // Ở đây Qa isotropic nên đơn giản:
    double q_vel = var_accel * dt * dt;
    Q(3,3) = q_vel;  Q(4,4) = q_vel;  Q(5,5) = q_vel;

    // Q(θ,θ) = var_gyro * I * dt²
    double q_att = var_gyro * dt * dt;
    Q(6,6) = q_att;  Q(7,7) = q_att;  Q(8,8) = q_att;

    // Q(bg,bg) = var_bg_rw * I
    Q(9,9) = var_bg_rw;  Q(10,10) = var_bg_rw;  Q(11,11) = var_bg_rw;

    // Q(ba,ba) = var_ba_rw * I
    Q(12,12) = var_ba_rw;  Q(13,13) = var_ba_rw;  Q(14,14) = var_ba_rw;

    // ================================================================
    // 6) Propagate covariance: P = F * P * F^T + Q
    // ================================================================
    P_ = F * P_ * F.T() + Q;

    // ================================================================
    // 7) Regularize P
    // ================================================================
    regularize_P();
}

}  // namespace eskf
