/**
 * @file gps_update.cpp
 * @brief GPS measurement update cho ESKF15 — position + velocity (6D).
 *
 * Observation model GPS:
 *   z_pos = pos_gps  (3, NED)
 *   z_vel = vel_gps  (3, NED)
 *
 *   h(x) = [p; v]
 *
 *   H = [I₃  0₃  0₃  0₃  0₃]   (pos: δz_pos/δ(δp) = I)
 *       [0₃  I₃  0₃  0₃  0₃]   (vel: δz_vel/δ(δv) = I)
 *
 * H có kích thước 6×15.
 *
 * Innovation:
 *   y = z - h(x) = [gps_pos - p; gps_vel - v]
 *
 * Innovation covariance:
 *   S = H * P * H^T + R   (6×6)
 *
 * Gating (chi-square test):
 *   NIS = y^T * S^{-1} * y < χ²(α, 6)
 *
 * Kalman gain:
 *   K = P * H^T * S^{-1}  (15×6)
 *
 * Error-state update:
 *   δx = K * y
 *
 * Covariance update — Joseph form (ổn định số hơn standard form):
 *   P = (I - K*H) * P * (I - K*H)^T + K * R * K^T
 *
 * Tại sao Joseph form?
 * - Standard form P = (I-KH)P có thể mất positive-definiteness do rounding
 * - Joseph form đảm bảo P luôn symmetric positive semi-definite
 * - Quantitatif giống nhau nhưng robust hơn cho fixed-point / float
 *
 * Delay compensation:
 * - GPS có delay ~200ms (gps.t_stamp < gps.t_receive)
 * - Nếu có delay > 1 bước IMU:
 *   1) Rollback state/P về snapshot gần t_stamp
 *   2) Apply GPS update tại đó
 *   3) Re-propagate forward bằng IMU buffer
 */

#include "eskf/eskf.hpp"

namespace eskf {

InnovationRecord ESKF15::update_gps(const GpsSample& gps) {
    InnovationRecord rec;
    rec.clear();
    rec.sensor_id = 0;  // GPS
    rec.t = gps.t_receive;
    rec.dim = 6;

    if (!gps.valid) {
        rec.accepted = false;
        return rec;
    }

    // ================================================================
    // Delay compensation: kiểm tra xem GPS có delay không
    // ================================================================
    double t_now = state_.t;
    double t_meas = gps.t_stamp;
    double delay = t_now - t_meas;

    // Nếu delay > 1 bước IMU → rollback
    bool use_rollback = (delay > IMU_DT * 1.5);

    NominalState saved_state;
    PMatrix saved_P;
    double saved_t = t_now;

    if (use_rollback) {
        // Tìm snapshot gần nhất trước t_meas
        int snap_idx = snap_buf_.find_latest_before(t_meas);
        if (snap_idx < 0) {
            // Không tìm được snapshot → không thể rollback
            // Fallback: update trực tiếp (less accurate nhưng vẫn OK)
            use_rollback = false;
        } else {
            // Lưu state hiện tại
            saved_state.copy_from(state_);
            saved_P = P_;

            // Rollback về snapshot
            const StateSnapshot& snap = snap_buf_.at(snap_idx);
            state_.copy_from(snap.state);
            P_ = snap.P;

            // Re-propagate từ snapshot → t_meas bằng IMU history
            // (chỉ propagate đến t_meas, không đến t_now)
            static ImuSample replay_imu[IMU_BUF_SIZE];
            int n_imu = imu_buf_.get_after(state_.t, replay_imu, IMU_BUF_SIZE);
            for (int i = 0; i < n_imu; ++i) {
                if (replay_imu[i].t > t_meas + 1e-9) break;
                propagate_one_step(replay_imu[i]);
            }
        }
    }

    // ================================================================
    // Xây dựng observation
    // ================================================================

    // Innovation y = z - h(x)
    // y(0:3) = gps_pos - p
    // y(3:6) = gps_vel - v
    Mat<6,1> y = Mat<6,1>::zero();
    y[0] = gps.pos[0] - state_.p[0];
    y[1] = gps.pos[1] - state_.p[1];
    y[2] = gps.pos[2] - state_.p[2];
    y[3] = gps.vel[0] - state_.v[0];
    y[4] = gps.vel[1] - state_.v[1];
    y[5] = gps.vel[2] - state_.v[2];

    // H matrix (6×15)
    // H = [I₃ 0 0 0 0; 0 I₃ 0 0 0]
    Mat<6, STATE_DIM> H = Mat<6, STATE_DIM>::zero();
    H(0,0) = 1; H(1,1) = 1; H(2,2) = 1;   // pos
    H(3,3) = 1; H(4,4) = 1; H(5,5) = 1;   // vel

    // R matrix (6×6) — measurement noise covariance
    Mat<6,6> R = Mat<6,6>::zero();
    double rp = GPS_POS_NOISE * GPS_POS_NOISE;
    double rv = GPS_VEL_NOISE * GPS_VEL_NOISE;
    R(0,0) = rp; R(1,1) = rp; R(2,2) = rp;
    R(3,3) = rv; R(4,4) = rv; R(5,5) = rv;

    // ================================================================
    // Innovation covariance S = H * P * H^T + R
    // ================================================================
    Mat<6,6> S = H * P_ * H.T() + R;

    // ================================================================
    // Gating: NIS = y^T * S^{-1} * y
    // ================================================================
    Mat<6,6> S_inv;
    if (!invert6(S, S_inv)) {
        // S singular → reject
        rec.accepted = false;
        if (use_rollback) {
            // Khôi phục state
            state_.copy_from(saved_state);
            P_ = saved_P;
        }
        gps_reject_++;
        return rec;
    }

    Mat<1,1> nis_mat = y.T() * S_inv * y;
    double nis = nis_mat[0];

    rec.nis = nis;
    for (int i = 0; i < 6; ++i) rec.innovation[i] = y[i];

    if (nis > GPS_GATE_CHI2) {
        // Failed gating → reject measurement
        rec.accepted = false;
        if (use_rollback) {
            state_.copy_from(saved_state);
            P_ = saved_P;
        }
        gps_reject_++;
        return rec;
    }

    // ================================================================
    // Kalman gain: K = P * H^T * S^{-1}   (15×6)
    // ================================================================
    Mat<STATE_DIM, 6> K = P_ * H.T() * S_inv;

    // ================================================================
    // Error-state update: δx = K * y
    // ================================================================
    Mat<STATE_DIM, 1> dx = K * y;
    inject_error_state(dx);

    // ================================================================
    // Covariance update — Joseph form
    // P = (I-KH) * P * (I-KH)^T + K * R * K^T
    // ================================================================
    PMatrix I_KH = PMatrix::identity() - K * H;
    P_ = I_KH * P_ * I_KH.T() + K * R * K.T();

    regularize_P();

    // ================================================================
    // Nếu đã rollback → re-propagate forward đến t_now
    // ================================================================
    if (use_rollback) {
        repropagate_to(saved_t);
    }

    rec.accepted = true;
    gps_accept_++;
    return rec;
}

}  // namespace eskf
