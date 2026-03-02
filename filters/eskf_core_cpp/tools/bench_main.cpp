/**
 * @file bench_main.cpp
 * @brief Benchmark tool — đo thời gian chạy predict/update trung bình.
 *
 * Cách dùng:
 *   ./eskf_bench <sensor_log.csv>
 *
 * Đo:
 * - Thời gian predict trung bình per IMU step (µs)
 * - Thời gian GPS update trung bình (µs)
 * - Thời gian MAG update trung bình (µs)
 * - Thời gian Baro update trung bình (µs)
 * - Throughput: steps/second
 *
 * Đảm bảo ESKF core đủ nhanh cho realtime (< 2.5ms per IMU step @ 400Hz).
 */

#include "eskf/eskf.hpp"
#include "eskf/sensors.hpp"
#include <cstdio>
#include <chrono>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::printf("Cách dùng: %s <sensor_log.csv>\n", argv[0]);
        return 1;
    }

    const char* input_path = argv[1];

    eskf::SensorLogReader reader;
    if (!reader.open(input_path)) {
        std::printf("LỖI: Không mở được file: %s\n", input_path);
        return 1;
    }

    std::printf("=== ESKF15 Benchmark ===\n");
    std::printf("  Input: %s\n", input_path);

    // Khởi tạo filter
    eskf::ESKF15 filter;
    eskf::LogRow row;

    if (!reader.read_next(row)) {
        std::printf("LỖI: File trống\n");
        return 1;
    }
    filter.init(row.t);

    // Bộ đếm
    int n_predict = 0, n_gps = 0, n_mag = 0, n_baro = 0;
    double total_predict_us = 0, total_gps_us = 0;
    double total_mag_us = 0, total_baro_us = 0;
    double max_predict_us = 0, max_gps_us = 0;
    double max_mag_us = 0, max_baro_us = 0;
    double prev_t = row.t;

    auto t_overall_start = std::chrono::high_resolution_clock::now();

    do {
        // Predict
        eskf::ImuSample imu;
        imu.t = row.t;
        imu.gyro = row.imu_gyro;
        imu.accel = row.imu_accel;
        imu.dt = row.t - prev_t;
        if (imu.dt <= 0) imu.dt = eskf::IMU_DT;
        prev_t = row.t;

        auto t0 = std::chrono::high_resolution_clock::now();
        filter.predict(imu);
        auto t1 = std::chrono::high_resolution_clock::now();
        double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
        total_predict_us += us;
        if (us > max_predict_us) max_predict_us = us;
        n_predict++;

        // GPS
        if (row.gps_valid) {
            eskf::GpsSample gps;
            gps.t_stamp = row.gps_ts;
            gps.t_receive = row.t;
            gps.pos = row.gps_pos;
            gps.vel = row.gps_vel;
            gps.valid = true;

            t0 = std::chrono::high_resolution_clock::now();
            filter.update_gps(gps);
            t1 = std::chrono::high_resolution_clock::now();
            us = std::chrono::duration<double, std::micro>(t1 - t0).count();
            total_gps_us += us;
            if (us > max_gps_us) max_gps_us = us;
            n_gps++;
        }

        // MAG
        if (row.mag_valid) {
            eskf::MagSample mag;
            mag.t = row.t;
            mag.mag_body = row.mag_meas;
            mag.valid = true;

            t0 = std::chrono::high_resolution_clock::now();
            filter.update_mag(mag);
            t1 = std::chrono::high_resolution_clock::now();
            us = std::chrono::duration<double, std::micro>(t1 - t0).count();
            total_mag_us += us;
            if (us > max_mag_us) max_mag_us = us;
            n_mag++;
        }

        // Baro
        if (row.baro_valid) {
            eskf::BaroSample baro;
            baro.t = row.t;
            baro.alt = row.baro_alt;
            baro.valid = true;

            t0 = std::chrono::high_resolution_clock::now();
            filter.update_baro(baro);
            t1 = std::chrono::high_resolution_clock::now();
            us = std::chrono::duration<double, std::micro>(t1 - t0).count();
            total_baro_us += us;
            if (us > max_baro_us) max_baro_us = us;
            n_baro++;
        }

    } while (reader.read_next(row));

    auto t_overall_end = std::chrono::high_resolution_clock::now();
    double total_ms = std::chrono::duration<double, std::milli>(t_overall_end - t_overall_start).count();

    reader.close();

    // In kết quả
    std::printf("\n=== KẾT QUẢ BENCHMARK ===\n");
    std::printf("  Tổng thời gian:     %.1f ms\n", total_ms);
    std::printf("  Tổng steps:         %d\n", n_predict);
    std::printf("  Throughput:         %.0f steps/s\n",
                n_predict / (total_ms * 0.001));
    std::printf("\n");
    std::printf("  PREDICT (IMU):\n");
    std::printf("    Số lần:  %d\n", n_predict);
    std::printf("    Mean:    %.2f µs\n", total_predict_us / n_predict);
    std::printf("    Max:     %.2f µs\n", max_predict_us);
    std::printf("\n");

    if (n_gps > 0) {
        std::printf("  GPS UPDATE:\n");
        std::printf("    Số lần:  %d\n", n_gps);
        std::printf("    Mean:    %.2f µs\n", total_gps_us / n_gps);
        std::printf("    Max:     %.2f µs\n", max_gps_us);
        std::printf("\n");
    }

    if (n_mag > 0) {
        std::printf("  MAG UPDATE:\n");
        std::printf("    Số lần:  %d\n", n_mag);
        std::printf("    Mean:    %.2f µs\n", total_mag_us / n_mag);
        std::printf("    Max:     %.2f µs\n", max_mag_us);
        std::printf("\n");
    }

    if (n_baro > 0) {
        std::printf("  BARO UPDATE:\n");
        std::printf("    Số lần:  %d\n", n_baro);
        std::printf("    Mean:    %.2f µs\n", total_baro_us / n_baro);
        std::printf("    Max:     %.2f µs\n", max_baro_us);
    }

    std::printf("==========================\n");

    // Check: nếu mean predict > 2000µs → cảnh báo
    double mean_predict = total_predict_us / n_predict;
    if (mean_predict > 2000.0) {
        std::printf("⚠ CẢNH BÁO: Mean predict %.0f µs > 2000 µs budget @ 400Hz!\n",
                    mean_predict);
    } else {
        std::printf("✓ OK: Mean predict %.1f µs — đủ nhanh cho 400Hz realtime\n",
                    mean_predict);
    }

    return 0;
}
