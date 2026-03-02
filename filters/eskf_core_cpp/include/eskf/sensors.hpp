/**
 * @file sensors.hpp
 * @brief Hàm đọc sensor log CSV — parse input cho ESKF.
 *
 * Format CSV input (từ simulator):
 * Cột: t, phase, truth_pN..., imu_gx..., gps_valid, gps_ts, gps_pN..., baro_valid, baro_alt, mag_valid, mag_mx...
 *
 * File này chỉ dùng cho tools (replay_main, bench_main),
 * KHÔNG nằm trong ESKF core runtime (embedded không đọc CSV).
 */

#ifndef ESKF_SENSORS_HPP
#define ESKF_SENSORS_HPP

#include "eskf_types.hpp"
#include <cstdio>
#include <cstring>
#include <cstdlib>

namespace eskf {

/**
 * Một hàng dữ liệu từ sensor log CSV.
 * Chứa truth + measurement cho 1 tick IMU.
 */
struct LogRow {
    double t;

    // Truth
    Vec3 truth_pos;     // NED (m)
    Vec3 truth_vel;     // NED (m/s)
    Vec4 truth_quat;    // [w,x,y,z] body→NED
    Vec3 truth_omega;   // body (rad/s)
    Vec3 truth_sf;      // specific force body (m/s²)
    Vec3 truth_mag;     // mag body (µT)
    Vec3 truth_bg;      // gyro bias body (rad/s)
    Vec3 truth_ba;      // accel bias body (m/s²)

    // IMU measurement
    Vec3 imu_gyro;      // body (rad/s)
    Vec3 imu_accel;     // body (m/s²)

    // GPS measurement
    bool gps_valid;
    double gps_ts;      // timestamp lúc đo
    Vec3 gps_pos;       // NED (m)
    Vec3 gps_vel;       // NED (m/s)

    // Baro measurement
    bool baro_valid;
    double baro_alt;    // altitude (m), dương lên

    // Mag measurement
    bool mag_valid;
    Vec3 mag_meas;      // body (µT)
};

/**
 * Interface đọc sensor log CSV.
 *
 * Sử dụng:
 *   SensorLogReader reader;
 *   if (!reader.open("sensor_log.csv")) { ... error ... }
 *   LogRow row;
 *   while (reader.read_next(row)) {
 *       // xử lý row
 *   }
 *   reader.close();
 */
class SensorLogReader {
public:
    SensorLogReader() : fp_(nullptr), line_num_(0) {}
    ~SensorLogReader() { close(); }

    /**
     * Mở file CSV sensor log.
     * @return true nếu mở thành công và header hợp lệ.
     */
    bool open(const char* path) {
        fp_ = std::fopen(path, "r");
        if (!fp_) return false;

        // Đọc và bỏ qua header line
        char buf[4096];
        if (!std::fgets(buf, sizeof(buf), fp_)) {
            close();
            return false;
        }
        line_num_ = 1;
        return true;
    }

    /**
     * Đọc 1 hàng tiếp theo từ CSV.
     * @param row  Output — dữ liệu đã parse
     * @return true nếu đọc thành công, false nếu EOF hoặc lỗi
     */
    bool read_next(LogRow& row) {
        if (!fp_) return false;

        char buf[4096];
        if (!std::fgets(buf, sizeof(buf), fp_)) return false;
        line_num_++;

        // Parse CSV: 47 cột
        // t, phase, truth_pN..pD, truth_vN..vD, truth_qw..qz,
        // truth_wx..wz, truth_sfx..sfz, truth_mx..mz,
        // truth_bgx..bgz, truth_bax..baz,
        // imu_gx..gz, imu_ax..az,
        // gps_valid, gps_ts, gps_pN..pD, gps_vN..vD,
        // baro_valid, baro_alt,
        // mag_valid, mag_mx..mz

        // Dùng strtok để tách — đơn giản, embedded-friendly
        double vals[50];
        int col = 0;
        char* token = std::strtok(buf, ",");

        // Cột 0: t (số)
        if (!token) return false;
        vals[col++] = std::atof(token);

        // Cột 1: phase (string) — skip
        token = std::strtok(nullptr, ",");
        if (!token) return false;

        // Cột 2-46: số
        for (int i = 0; i < 45; ++i) {
            token = std::strtok(nullptr, ",\n\r");
            if (!token) {
                // Thiếu cột — dùng 0
                vals[col++] = 0.0;
            } else {
                vals[col++] = std::atof(token);
            }
        }

        // Unpack vào row
        int c = 0;
        row.t = vals[c++];

        // Truth pos (3)
        row.truth_pos[0] = vals[c++];
        row.truth_pos[1] = vals[c++];
        row.truth_pos[2] = vals[c++];

        // Truth vel (3)
        row.truth_vel[0] = vals[c++];
        row.truth_vel[1] = vals[c++];
        row.truth_vel[2] = vals[c++];

        // Truth quat (4): w,x,y,z
        row.truth_quat[0] = vals[c++];
        row.truth_quat[1] = vals[c++];
        row.truth_quat[2] = vals[c++];
        row.truth_quat[3] = vals[c++];

        // Truth omega (3)
        row.truth_omega[0] = vals[c++];
        row.truth_omega[1] = vals[c++];
        row.truth_omega[2] = vals[c++];

        // Truth specific force (3)
        row.truth_sf[0] = vals[c++];
        row.truth_sf[1] = vals[c++];
        row.truth_sf[2] = vals[c++];

        // Truth mag (3)
        row.truth_mag[0] = vals[c++];
        row.truth_mag[1] = vals[c++];
        row.truth_mag[2] = vals[c++];

        // Truth gyro bias (3)
        row.truth_bg[0] = vals[c++];
        row.truth_bg[1] = vals[c++];
        row.truth_bg[2] = vals[c++];

        // Truth accel bias (3)
        row.truth_ba[0] = vals[c++];
        row.truth_ba[1] = vals[c++];
        row.truth_ba[2] = vals[c++];

        // IMU gyro (3)
        row.imu_gyro[0] = vals[c++];
        row.imu_gyro[1] = vals[c++];
        row.imu_gyro[2] = vals[c++];

        // IMU accel (3)
        row.imu_accel[0] = vals[c++];
        row.imu_accel[1] = vals[c++];
        row.imu_accel[2] = vals[c++];

        // GPS valid (1), gps_ts (1), gps_pos (3), gps_vel (3)
        row.gps_valid = (vals[c++] > 0.5);
        row.gps_ts = vals[c++];
        row.gps_pos[0] = vals[c++];
        row.gps_pos[1] = vals[c++];
        row.gps_pos[2] = vals[c++];
        row.gps_vel[0] = vals[c++];
        row.gps_vel[1] = vals[c++];
        row.gps_vel[2] = vals[c++];

        // Baro valid (1), baro_alt (1)
        row.baro_valid = (vals[c++] > 0.5);
        row.baro_alt = vals[c++];

        // Mag valid (1), mag_meas (3)
        row.mag_valid = (vals[c++] > 0.5);
        row.mag_meas[0] = vals[c++];
        row.mag_meas[1] = vals[c++];
        row.mag_meas[2] = vals[c++];

        return true;
    }

    void close() {
        if (fp_) {
            std::fclose(fp_);
            fp_ = nullptr;
        }
    }

    int line_number() const { return line_num_; }

private:
    FILE* fp_;
    int line_num_;
};

}  // namespace eskf

#endif  // ESKF_SENSORS_HPP
