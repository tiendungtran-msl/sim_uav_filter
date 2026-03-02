/**
 * @file config.hpp
 * @brief Cấu hình biên dịch và tham số mặc định cho ESKF core.
 *
 * Mọi hằng số compile-time được định nghĩa ở đây, bao gồm:
 * - Kích thước state vector (15 hoặc 24)
 * - Tần số sensor mặc định
 * - Ngưỡng gating chi-square
 * - Kích thước ring buffer
 *
 * Thiết kế embedded-friendly:
 * - Không dùng heap/dynamic allocation
 * - Tất cả kích thước xác định lúc compile
 * - Tương thích NuttX/RTOS
 */

#ifndef ESKF_CONFIG_HPP
#define ESKF_CONFIG_HPP

#include <cstdint>

namespace eskf {

// =====================================================================
// Kích thước state vector
// =====================================================================

/** Số chiều error-state ESKF15: δp(3) + δv(3) + δθ(3) + δbg(3) + δba(3) */
static constexpr int STATE_DIM = 15;

/** Số chiều nominal state: p(3) + v(3) + q(4) + bg(3) + ba(3) = 16 */
static constexpr int NOMINAL_DIM = 16;

// =====================================================================
// Tham số vật lý
// =====================================================================

/** Gia tốc trọng trường (m/s²) */
static constexpr double GRAVITY = 9.80665;

// =====================================================================
// Tham số IMU (ICM42688P-style)
// =====================================================================

/** Tần số IMU mặc định (Hz) */
static constexpr double IMU_RATE_HZ = 400.0;

/** IMU dt mặc định (s) */
static constexpr double IMU_DT = 1.0 / IMU_RATE_HZ;

/** Gyro noise spectral density (rad/s/√Hz) — ARW */
static constexpr double GYRO_NOISE_PSD = 0.005;

/** Gyro bias random walk (rad/s²/√Hz) */
static constexpr double GYRO_BIAS_RW = 0.0002;

/** Accel noise spectral density (m/s²/√Hz) — VRW */
static constexpr double ACCEL_NOISE_PSD = 0.1;

/** Accel bias random walk (m/s³/√Hz) */
static constexpr double ACCEL_BIAS_RW = 0.005;

// =====================================================================
// Tham số GPS
// =====================================================================

/** Tần số GPS mặc định (Hz) */
static constexpr double GPS_RATE_HZ = 10.0;

/** GPS position noise 1σ (m) per axis */
static constexpr double GPS_POS_NOISE = 1.5;

/** GPS velocity noise 1σ (m/s) per axis */
static constexpr double GPS_VEL_NOISE = 0.15;

/** GPS delay mặc định (s) */
static constexpr double GPS_DELAY_DEFAULT = 0.20;

// =====================================================================
// Tham số Magnetometer (BMM150-style)
// =====================================================================

/** Tần số mag mặc định (Hz) */
static constexpr double MAG_RATE_HZ = 50.0;

/** Mag noise 1σ (µT) per axis */
static constexpr double MAG_NOISE = 0.8;

/** Từ trường chuẩn NED (µT) — vĩ độ trung bình miền Bắc */
static constexpr double MAG_EARTH_N = 22.0;
static constexpr double MAG_EARTH_E = 1.0;
static constexpr double MAG_EARTH_D = 41.0;

// =====================================================================
// Tham số Barometer
// =====================================================================

/** Baro noise 1σ (m) */
static constexpr double BARO_NOISE = 0.5;

/** Baro rate (Hz) */
static constexpr double BARO_RATE_HZ = 50.0;

// =====================================================================
// Gating — chi-square thresholds
// =====================================================================

/**
 * Ngưỡng chi-square cho GPS 6D (pos+vel).
 * Với 6 bậc tự do, χ²(0.99) ≈ 16.81
 * Dùng giá trị rộng hơn để không reject quá nhiều.
 */
static constexpr double GPS_GATE_CHI2 = 50.0;

/**
 * Ngưỡng chi-square cho MAG 3D.
 * Với 3 bậc tự do, χ²(0.99) ≈ 11.34
 */
static constexpr double MAG_GATE_CHI2 = 15.0;

/**
 * Ngưỡng chi-square cho Baro 1D.
 * Với 1 bậc tự do, χ²(0.99) ≈ 6.63
 */
static constexpr double BARO_GATE_CHI2 = 10.0;

// =====================================================================
// Ring buffer — replay delay compensation
// =====================================================================

/**
 * Số snapshot state tối đa lưu trong ring buffer.
 * Với IMU 400Hz và lưu mỗi 10 bước → 40Hz snapshot → 2s = 80 entries.
 * Dùng 128 để có dư (power of 2 tối ưu modulo).
 */
static constexpr int SNAPSHOT_BUF_SIZE = 128;

/**
 * Số IMU sample tối đa lưu cho replay.
 * 400Hz × 2s = 800. Dùng 1024 (power of 2).
 */
static constexpr int IMU_BUF_SIZE = 1024;

/**
 * Bước snapshot: lưu state mỗi N bước IMU.
 * Lưu mỗi 10 bước → 40Hz → đủ chính xác cho rollback.
 */
static constexpr int SNAPSHOT_INTERVAL = 10;

// =====================================================================
// Covariance bounds — tránh divergence
// =====================================================================

/** P diagonal tối đa cho position (m²) */
static constexpr double P_MAX_POS = 1e6;

/** P diagonal tối đa cho velocity (m/s)² */
static constexpr double P_MAX_VEL = 1e4;

/** P diagonal tối đa cho attitude (rad²) */
static constexpr double P_MAX_ATT = 1.0;

/** P diagonal tối đa cho gyro bias */
static constexpr double P_MAX_GBIAS = 0.1;

/** P diagonal tối đa cho accel bias */
static constexpr double P_MAX_ABIAS = 1.0;

/** P diagonal tối thiểu (tránh singularity) */
static constexpr double P_MIN_DIAG = 1e-15;

// =====================================================================
// Initial covariance
// =====================================================================

static constexpr double P0_POS = 100.0;      // m² per axis (vị trí ban đầu chưa biết)
static constexpr double P0_VEL = 10.0;       // (m/s)² per axis
static constexpr double P0_ATT = 0.1;        // rad² per axis (~18°)
static constexpr double P0_GBIAS = 1e-4;     // (rad/s)²
static constexpr double P0_ABIAS = 0.04;     // (m/s²)²

}  // namespace eskf

#endif  // ESKF_CONFIG_HPP
