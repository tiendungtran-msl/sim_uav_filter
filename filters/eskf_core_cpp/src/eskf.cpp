/**
 * @file eskf.cpp
 * @brief Implementation chính của ESKF15 — init, predict, inject, regularize.
 *
 * File này chứa:
 * - Constructor + init()
 * - predict() — wrapper gọi propagate_one_step + snapshot
 * - inject_error_state() — cập nhật nominal từ error-state
 * - regularize_P() — symmetrize + clamp
 * - save_snapshot() — lưu state/P vào ring buffer
 * - repropagate_to() — replay IMU history sau rollback
 *
 * Các measurement update (GPS, MAG, Baro) nằm ở file riêng.
 */

#include "eskf/eskf.hpp"

namespace eskf {

// =====================================================================
// Constructor
// =====================================================================

ESKF15::ESKF15() {
    // Từ trường chuẩn NED mặc định (µT)
    mag_earth_ned = Vec3::zero();
    mag_earth_ned[0] = MAG_EARTH_N;
    mag_earth_ned[1] = MAG_EARTH_E;
    mag_earth_ned[2] = MAG_EARTH_D;

    imu_count_ = 0;
    step_since_snapshot_ = 0;
    gps_accept_ = gps_reject_ = 0;
    mag_accept_ = mag_reject_ = 0;
}

// =====================================================================
// Init — reset toàn bộ filter
// =====================================================================

void ESKF15::init(double t0) {
    // Reset nominal state
    state_.reset();
    state_.t = t0;

    // Initial covariance P0 = diag(...)
    P_ = PMatrix::zero();
    // δp: position uncertainty
    P_(0,0) = P0_POS;  P_(1,1) = P0_POS;  P_(2,2) = P0_POS;
    // δv: velocity uncertainty
    P_(3,3) = P0_VEL;  P_(4,4) = P0_VEL;  P_(5,5) = P0_VEL;
    // δθ: attitude uncertainty
    P_(6,6) = P0_ATT;  P_(7,7) = P0_ATT;  P_(8,8) = P0_ATT;
    // δbg: gyro bias uncertainty
    P_(9,9) = P0_GBIAS;  P_(10,10) = P0_GBIAS;  P_(11,11) = P0_GBIAS;
    // δba: accel bias uncertainty
    P_(12,12) = P0_ABIAS;  P_(13,13) = P0_ABIAS;  P_(14,14) = P0_ABIAS;

    // Reset counters
    imu_count_ = 0;
    step_since_snapshot_ = 0;
    gps_accept_ = gps_reject_ = 0;
    mag_accept_ = mag_reject_ = 0;

    // Reset buffers
    imu_buf_.reset();
    snap_buf_.reset();

    // Lưu snapshot ban đầu
    save_snapshot();
}

// =====================================================================
// Predict — IMU propagation
// =====================================================================

void ESKF15::predict(const ImuSample& imu) {
    // 1) Propagate state + covariance
    propagate_one_step(imu);

    // 2) Lưu IMU sample vào buffer (cho delay compensation replay)
    imu_buf_.push(imu);

    // 3) Lưu snapshot định kỳ
    imu_count_++;
    step_since_snapshot_++;
    if (step_since_snapshot_ >= SNAPSHOT_INTERVAL) {
        save_snapshot();
        step_since_snapshot_ = 0;
    }
}

// =====================================================================
// Inject error-state vào nominal
// =====================================================================

/**
 * Sau measurement update, error-state δx được tính từ Kalman gain:
 *   δx = K * innovation
 *
 * Cần inject δx vào nominal state:
 *   p += δp
 *   v += δv
 *   q = q ⊗ quat_from_rotvec(δθ)   ← QUAN TRỌNG: multiplicative update cho quaternion
 *   bg += δbg
 *   ba += δba
 *
 * Sau inject, error-state reset về 0 (implicit trong ESKF).
 */
void ESKF15::inject_error_state(const Mat<STATE_DIM, 1>& dx) {
    // δp → p
    state_.p[0] += dx[0];
    state_.p[1] += dx[1];
    state_.p[2] += dx[2];

    // δv → v
    state_.v[0] += dx[3];
    state_.v[1] += dx[4];
    state_.v[2] += dx[5];

    // δθ → q (multiplicative: q_new = q ⊗ δq)
    Vec3 dtheta;
    dtheta[0] = dx[6]; dtheta[1] = dx[7]; dtheta[2] = dx[8];
    Vec4 dq = quat_from_rotvec(dtheta);
    state_.q = quat_normalize(quat_mult(state_.q, dq));

    // δbg → bg
    state_.bg[0] += dx[9];
    state_.bg[1] += dx[10];
    state_.bg[2] += dx[11];

    // δba → ba
    state_.ba[0] += dx[12];
    state_.ba[1] += dx[13];
    state_.ba[2] += dx[14];
}

// =====================================================================
// Regularize P — symmetrize + clamp
// =====================================================================

/**
 * Duy trì tính ổn định số của ma trận covariance:
 * 1) P = 0.5 * (P + P^T) — loại bỏ asymmetry tích luỹ
 * 2) Clamp diagonal: P_MIN_DIAG ≤ P(i,i) ≤ P_MAX theo state
 *
 * Nếu không làm điều này, sau nhiều bước propagate,
 * P có thể trở nên không đối xứng → NaN → filter diverge.
 */
void ESKF15::regularize_P() {
    P_.symmetrize();

    // Clamp theo nhóm state
    for (int i = 0; i < 3; ++i)  P_(i,i) = (P_(i,i) < P_MIN_DIAG) ? P_MIN_DIAG : ((P_(i,i) > P_MAX_POS) ? P_MAX_POS : P_(i,i));
    for (int i = 3; i < 6; ++i)  P_(i,i) = (P_(i,i) < P_MIN_DIAG) ? P_MIN_DIAG : ((P_(i,i) > P_MAX_VEL) ? P_MAX_VEL : P_(i,i));
    for (int i = 6; i < 9; ++i)  P_(i,i) = (P_(i,i) < P_MIN_DIAG) ? P_MIN_DIAG : ((P_(i,i) > P_MAX_ATT) ? P_MAX_ATT : P_(i,i));
    for (int i = 9; i < 12; ++i) P_(i,i) = (P_(i,i) < P_MIN_DIAG) ? P_MIN_DIAG : ((P_(i,i) > P_MAX_GBIAS) ? P_MAX_GBIAS : P_(i,i));
    for (int i = 12; i < 15; ++i) P_(i,i) = (P_(i,i) < P_MIN_DIAG) ? P_MIN_DIAG : ((P_(i,i) > P_MAX_ABIAS) ? P_MAX_ABIAS : P_(i,i));
}

// =====================================================================
// Save snapshot
// =====================================================================

void ESKF15::save_snapshot() {
    StateSnapshot snap;
    snap.state.copy_from(state_);
    snap.P = P_;
    snap.t = state_.t;
    snap.valid = true;
    snap_buf_.push(snap);
}

// =====================================================================
// Re-propagate — replay IMU từ snapshot đến now
// =====================================================================

/**
 * Sau khi rollback state/P về snapshot (t_snap),
 * cần re-propagate bằng chuỗi IMU đã lưu từ t_snap → t_now.
 *
 * Tìm tất cả IMU sample trong buffer có t > t_snap,
 * rồi propagate lại từng bước.
 */
void ESKF15::repropagate_to(double t_now) {
    // Lấy tất cả IMU samples sau thời điểm hiện tại của state_
    // (state_.t đã được rollback về t_snap)
    static ImuSample replay_buf[IMU_BUF_SIZE];
    int n = imu_buf_.get_after(state_.t, replay_buf, IMU_BUF_SIZE);

    for (int i = 0; i < n; ++i) {
        if (replay_buf[i].t > t_now + 1e-9) break;  // Không vượt quá now
        propagate_one_step(replay_buf[i]);
    }

    // Đảm bảo timestamp đúng
    state_.t = t_now;
}

}  // namespace eskf
