/**
 * @file eskf_types.hpp
 * @brief Các kiểu dữ liệu chính cho ESKF: NominalState, ErrorState, Snapshot.
 *
 * Thiết kế:
 * - NominalState: trạng thái danh nghĩa (p, v, q, bg, ba)
 * - ErrorState: vector sai số 15D (δp, δv, δθ, δbg, δba)
 * - StateSnapshot: lưu state + covariance tại 1 thời điểm → dùng cho rollback
 * - ImuSample: 1 mẫu IMU đã trừ bias (hoặc chưa, tuỳ context)
 *
 * Tất cả POD hoặc aggregate — không heap, không virtual.
 */

#ifndef ESKF_TYPES_HPP
#define ESKF_TYPES_HPP

#include "static_matrix.hpp"
#include "config.hpp"

namespace eskf {

// =====================================================================
// Nominal state — trạng thái danh nghĩa (true estimate)
// =====================================================================

/**
 * Trạng thái danh nghĩa ESKF (16 scalars).
 *
 * Quy ước:
 * - p: vị trí NED (m)
 * - v: vận tốc NED (m/s)
 * - q: quaternion body→NED [w,x,y,z]
 * - bg: gyro bias body (rad/s)
 * - ba: accel bias body (m/s²)
 */
struct NominalState {
    Vec3 p;     ///< Vị trí NED (3)
    Vec3 v;     ///< Vận tốc NED (3)
    Vec4 q;     ///< Quaternion body→NED [w,x,y,z] (4)
    Vec3 bg;    ///< Gyro bias estimate body (3)
    Vec3 ba;    ///< Accel bias estimate body (3)
    double t;   ///< Timestamp (s)

    /** Khởi tạo mặc định: hovering tại gốc, không bias */
    void reset() {
        p = Vec3::zero();
        v = Vec3::zero();
        q = Vec4::zero(); q[0] = 1.0;  // identity quaternion
        bg = Vec3::zero();
        ba = Vec3::zero();
        t = 0.0;
    }

    /** Copy state từ nguồn khác */
    void copy_from(const NominalState& src) {
        p = src.p; v = src.v; q = src.q;
        bg = src.bg; ba = src.ba; t = src.t;
    }
};

// =====================================================================
// IMU sample — 1 mẫu đo IMU đã lưu trong buffer
// =====================================================================

/**
 * Mẫu IMU thô (raw measurement), chưa trừ bias.
 * Lưu trong ring buffer để replay khi rollback.
 */
struct ImuSample {
    double t;       ///< Timestamp (s)
    Vec3 gyro;      ///< Gyro measurement body (rad/s) — gồm bias + noise
    Vec3 accel;     ///< Accel measurement body (m/s²) — gồm bias + noise
    double dt;      ///< Delta-time từ sample trước (s)

    void clear() {
        t = 0; gyro = Vec3::zero(); accel = Vec3::zero(); dt = 0;
    }
};

// =====================================================================
// State snapshot — cho delay compensation rollback
// =====================================================================

/**
 * Snapshot toàn bộ state + covariance tại 1 thời điểm.
 *
 * Dùng cho cơ chế delay compensation kiểu PX4/EKF2:
 * - Khi nhận measurement có delay (t_stamp < now):
 *   1) Tìm snapshot gần t_stamp nhất
 *   2) Rollback state/P về snapshot đó
 *   3) Apply measurement update
 *   4) Re-propagate forward bằng IMU history từ snapshot → now
 *
 * Kích thước: 16 + 15*15 = 241 doubles ≈ 1.9 KB per snapshot.
 * Với 128 snapshots = ~245 KB — chấp nhận được cho embedded.
 */
struct StateSnapshot {
    NominalState state;
    Mat<STATE_DIM, STATE_DIM> P;     ///< Error covariance (15×15)
    double t;                         ///< Timestamp
    bool valid;                       ///< Snapshot có hợp lệ không

    void clear() {
        state.reset();
        P = Mat<STATE_DIM, STATE_DIM>::zero();
        t = 0;
        valid = false;
    }
};

// =====================================================================
// Measurement samples — cho sensor fusion
// =====================================================================

/**
 * Mẫu GPS (position + velocity).
 * - t_stamp: thời điểm GPS thực sự đo (trước delay)
 * - t_receive: thời điểm filter nhận (sau delay)
 * - pos_ned: vị trí NED (m)
 * - vel_ned: vận tốc NED (m/s)
 */
struct GpsSample {
    double t_stamp;     ///< Thời điểm đo (trước delay)
    double t_receive;   ///< Thời điểm nhận (sau delay)
    Vec3 pos;           ///< Vị trí NED (m)
    Vec3 vel;           ///< Vận tốc NED (m/s)
    bool valid;

    void clear() {
        t_stamp = 0; t_receive = 0;
        pos = Vec3::zero(); vel = Vec3::zero();
        valid = false;
    }
};

/**
 * Mẫu Magnetometer (BMM150).
 * - mag_body: vector từ trường đo trong body frame (µT)
 */
struct MagSample {
    double t;
    Vec3 mag_body;      ///< Từ trường đo body (µT)
    bool valid;

    void clear() { t = 0; mag_body = Vec3::zero(); valid = false; }
};

/**
 * Mẫu Barometer.
 * - alt: altitude (m), dương hướng lên (= -p_D)
 */
struct BaroSample {
    double t;
    double alt;         ///< Altitude (m), dương hướng lên
    bool valid;

    void clear() { t = 0; alt = 0; valid = false; }
};

// =====================================================================
// Innovation output — để GUI vẽ
// =====================================================================

/**
 * Kết quả innovation từ 1 lần measurement update.
 * Lưu lại để xuất file innovations.csv cho GUI vẽ.
 */
struct InnovationRecord {
    double t;               ///< Timestamp
    int sensor_id;          ///< 0=GPS, 1=MAG, 2=BARO
    double innovation[6];   ///< Innovation vector (tối đa 6D cho GPS)
    double nis;             ///< Normalized Innovation Squared
    int dim;                ///< Số chiều innovation thực tế
    bool accepted;          ///< Passed gating hay bị reject

    void clear() {
        t = 0; sensor_id = 0; nis = 0; dim = 0; accepted = false;
        for (int i = 0; i < 6; ++i) innovation[i] = 0;
    }
};

}  // namespace eskf

#endif  // ESKF_TYPES_HPP
