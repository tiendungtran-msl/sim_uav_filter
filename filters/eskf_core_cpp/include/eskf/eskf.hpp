/**
 * @file eskf.hpp
 * @brief API chính của ESKF15 — Error-State Kalman Filter 15 chiều.
 *
 * Đây là interface duy nhất mà bên ngoài (replay tool, embedded main loop) cần gọi.
 *
 * Pipeline xử lý:
 * 1) predict(imu)        — IMU propagation mỗi tick (400Hz)
 * 2) update_gps(gps)     — GPS position+velocity update (10Hz, có delay)
 * 3) update_mag(mag)     — Magnetometer heading update (50Hz)
 * 4) update_baro(baro)   — Barometer altitude update (50Hz)
 *
 * Delay compensation (PX4-style):
 * - Mỗi N bước IMU, lưu snapshot (state + P) vào ring buffer
 * - Khi nhận measurement có t_stamp < now:
 *   a) Rollback state/P về snapshot gần t_stamp
 *   b) Apply measurement update
 *   c) Re-propagate forward bằng IMU history
 * - Cơ chế này đảm bảo filter hoạt động đúng với GPS delay 200-300ms
 *
 * Ràng buộc embedded:
 * - Không new/delete, không exceptions
 * - Tất cả buffer trên stack (compile-time size)
 * - Không dùng STL nặng (vector, map, string)
 *
 * Error-state 15D:
 *   δx = [δp(3), δv(3), δθ(3), δbg(3), δba(3)]
 *   Trong đó δθ là rotation error (small angle rotation vector)
 *
 * Nominal state 16D:
 *   x = [p(3), v(3), q(4), bg(3), ba(3)]
 */

#ifndef ESKF_ESKF_HPP
#define ESKF_ESKF_HPP

#include "config.hpp"
#include "static_matrix.hpp"
#include "math.hpp"
#include "eskf_types.hpp"
#include "ring_buffer.hpp"

namespace eskf {

// Alias cho ma trận ESKF
using PMatrix = Mat<STATE_DIM, STATE_DIM>;  ///< Covariance 15×15
using FMatrix = Mat<STATE_DIM, STATE_DIM>;  ///< State transition 15×15

/**
 * ESKF15 — Error-State Kalman Filter 15 chiều.
 *
 * Sử dụng:
 *   ESKF15 filter;
 *   filter.init(t0);
 *
 *   // Mỗi tick IMU:
 *   filter.predict(imu_sample);
 *
 *   // Khi có GPS (có delay):
 *   filter.update_gps(gps_sample);
 *
 *   // Khi có MAG:
 *   filter.update_mag(mag_sample);
 *
 *   // Lấy kết quả:
 *   const NominalState& est = filter.state();
 */
class ESKF15 {
public:
    ESKF15();

    /**
     * Khởi tạo filter.
     * Reset state về mặc định, P = P0, clear buffers.
     *
     * @param t0  Timestamp bắt đầu (s)
     */
    void init(double t0);

    // =================================================================
    // Predict (IMU propagation)
    // =================================================================

    /**
     * IMU propagation — gọi mỗi tick IMU (400Hz).
     *
     * Thực hiện:
     * 1) Trừ bias: ω = gyro - bg, a = accel - ba
     * 2) Cập nhật nominal: p, v, q
     * 3) Tính F, Q
     * 4) Propagate P = F*P*F^T + Q
     * 5) Symmetrize + clamp P
     * 6) Lưu IMU vào buffer, lưu snapshot định kỳ
     *
     * @param imu  Mẫu IMU thô (chưa trừ bias)
     */
    void predict(const ImuSample& imu);

    // =================================================================
    // Measurement updates
    // =================================================================

    /**
     * GPS update — position + velocity (6D).
     *
     * Có delay compensation:
     * - Nếu gps.t_stamp < now: rollback, update, re-propagate
     * - Nếu gps.t_stamp ≈ now: direct update
     *
     * Joseph form: P = (I-KH)P(I-KH)^T + KRK^T
     * Gating: NIS = y^T * S^{-1} * y < χ² threshold
     *
     * @param gps  Mẫu GPS
     * @return InnovationRecord (để log)
     */
    InnovationRecord update_gps(const GpsSample& gps);

    /**
     * MAG update — từ trường 3D body.
     *
     * Observation model:
     *   h(x) = R(q)^T * mag_earth_ned
     *   y = mag_meas - h(x)
     *
     * Dùng full 3D mag vector (không chỉ heading).
     * Có gating.
     *
     * @param mag  Mẫu magnetometer
     * @return InnovationRecord
     */
    InnovationRecord update_mag(const MagSample& mag);

    /**
     * Baro update — altitude (1D).
     *
     * Observation: h(x) = -p_D  (altitude = -down)
     * y = baro_alt - h(x)
     *
     * @param baro  Mẫu barometer
     * @return InnovationRecord
     */
    InnovationRecord update_baro(const BaroSample& baro);

    // =================================================================
    // Accessors
    // =================================================================

    /** Trạng thái ước lượng hiện tại */
    const NominalState& state() const { return state_; }

    /** Ma trận covariance hiện tại */
    const PMatrix& covariance() const { return P_; }

    /** Timestamp hiện tại */
    double time() const { return state_.t; }

    /** Số bước IMU đã xử lý */
    int imu_count() const { return imu_count_; }

    /** Số lần GPS update thành công (passed gating) */
    int gps_accept_count() const { return gps_accept_; }

    /** Số lần GPS bị reject (failed gating) */
    int gps_reject_count() const { return gps_reject_; }

    /** Số lần MAG accept */
    int mag_accept_count() const { return mag_accept_; }

    /** Số lần MAG reject */
    int mag_reject_count() const { return mag_reject_; }

    /** Cấu hình từ trường chuẩn NED (µT) — có thể thay đổi trước init() */
    Vec3 mag_earth_ned;

private:
    // -----------------------------------------------------------------
    // Nominal state + covariance
    // -----------------------------------------------------------------
    NominalState state_;
    PMatrix P_;

    // -----------------------------------------------------------------
    // Bộ đếm
    // -----------------------------------------------------------------
    int imu_count_;
    int step_since_snapshot_;
    int gps_accept_, gps_reject_;
    int mag_accept_, mag_reject_;

    // -----------------------------------------------------------------
    // Ring buffers cho delay compensation
    // -----------------------------------------------------------------
    RingBuffer<ImuSample, IMU_BUF_SIZE> imu_buf_;
    RingBuffer<StateSnapshot, SNAPSHOT_BUF_SIZE> snap_buf_;

    // -----------------------------------------------------------------
    // Internal methods
    // -----------------------------------------------------------------

    /**
     * IMU propagation nội bộ — cập nhật state_ và P_ cho 1 bước dt.
     * Dùng cả cho forward propagation lần đầu và re-propagation khi rollback.
     *
     * @param imu  Mẫu IMU
     */
    void propagate_one_step(const ImuSample& imu);

    /**
     * Lưu snapshot state + P vào ring buffer.
     */
    void save_snapshot();

    /**
     * Delay compensation: rollback, apply update, re-propagate.
     *
     * @param t_meas  Timestamp thực sự của measurement
     * @param update_fn  Hàm thực hiện measurement update (GPS/MAG/Baro)
     * @return true nếu rollback thành công
     */
    bool delayed_update(double t_meas);

    /**
     * Re-propagate từ thời điểm hiện tại của state_ đến now,
     * dùng IMU history trong buffer.
     *
     * @param t_now  Timestamp hiện tại (trước rollback)
     */
    void repropagate_to(double t_now);

    /**
     * Inject error-state δx vào nominal state.
     * Sau measurement update, δx ≠ 0 → cần cập nhật nominal.
     *
     * p += δp
     * v += δv
     * q = q ⊗ δq(δθ)
     * bg += δbg
     * ba += δba
     *
     * Sau đó reset error-state = 0.
     *
     * @param dx  Error-state vector 15×1
     */
    void inject_error_state(const Mat<STATE_DIM, 1>& dx);

    /**
     * Symmetrize và clamp covariance P.
     * Gọi sau mỗi predict/update để duy trì tính ổn định số.
     */
    void regularize_P();
};

}  // namespace eskf

#endif  // ESKF_ESKF_HPP
