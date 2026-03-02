/**
 * @file replay_main.cpp
 * @brief Offline replay tool — đọc sensor log CSV → chạy ESKF → xuất estimated.csv
 *
 * Cách dùng:
 *   ./eskf_replay <sensor_log.csv> [estimated.csv] [innovations.csv]
 *
 * Pipeline:
 * 1) Đọc sensor_log.csv từng hàng (400Hz IMU ticks)
 * 2) Mỗi hàng: predict(IMU)
 * 3) Nếu GPS valid: update_gps (có delay compensation)
 * 4) Nếu MAG valid: update_mag
 * 5) Nếu Baro valid: update_baro
 * 6) Ghi estimated state + truth ra estimated.csv
 * 7) Ghi innovation ra innovations.csv
 * 8) In summary: RMSE, số reject, runtime
 *
 * Output estimated.csv format:
 *   t, p_est_N/E/D, v_est_N/E/D, q_est_w/x/y/z,
 *   bg_est_x/y/z, ba_est_x/y/z,
 *   p_true_N/E/D, v_true_N/E/D, q_true_w/x/y/z,
 *   bg_true_x/y/z, ba_true_x/y/z,
 *   roll_est, pitch_est, yaw_est,
 *   roll_true, pitch_true, yaw_true
 */

#include "eskf/eskf.hpp"
#include "eskf/sensors.hpp"
#include "eskf/innovations.hpp"
#include <cstdio>
#include <cmath>
#include <cstring>

// Dùng clock đơn giản cho timing (cross-platform)
#include <chrono>

/**
 * Tính RMSE từ mảng error²
 */
static double rmse(const double* sum_sq, int n, int dim) {
    if (n == 0) return 0;
    double total = 0;
    for (int i = 0; i < dim; ++i) total += sum_sq[i];
    return std::sqrt(total / (n * dim));
}

int main(int argc, char** argv) {
    // ================================================================
    // Parse arguments
    // ================================================================
    if (argc < 2) {
        std::printf("Cách dùng: %s <sensor_log.csv> [estimated.csv] [innovations.csv]\n", argv[0]);
        return 1;
    }

    const char* input_path = argv[1];
    const char* est_path = (argc >= 3) ? argv[2] : "estimated.csv";
    const char* innov_path = (argc >= 4) ? argv[3] : "innovations.csv";

    std::printf("=== ESKF15 Offline Replay ===\n");
    std::printf("  Input:        %s\n", input_path);
    std::printf("  Estimated:    %s\n", est_path);
    std::printf("  Innovations:  %s\n", innov_path);

    // ================================================================
    // Mở input
    // ================================================================
    eskf::SensorLogReader reader;
    if (!reader.open(input_path)) {
        std::printf("LỖI: Không mở được file input: %s\n", input_path);
        return 1;
    }

    // ================================================================
    // Mở output files
    // ================================================================
    FILE* fp_est = std::fopen(est_path, "w");
    if (!fp_est) {
        std::printf("LỖI: Không tạo được file estimated: %s\n", est_path);
        return 1;
    }

    // Header estimated.csv
    std::fprintf(fp_est,
        "t,"
        "p_est_N,p_est_E,p_est_D,"
        "v_est_N,v_est_E,v_est_D,"
        "q_est_w,q_est_x,q_est_y,q_est_z,"
        "bg_est_x,bg_est_y,bg_est_z,"
        "ba_est_x,ba_est_y,ba_est_z,"
        "p_true_N,p_true_E,p_true_D,"
        "v_true_N,v_true_E,v_true_D,"
        "q_true_w,q_true_x,q_true_y,q_true_z,"
        "bg_true_x,bg_true_y,bg_true_z,"
        "ba_true_x,ba_true_y,ba_true_z,"
        "roll_est,pitch_est,yaw_est,"
        "roll_true,pitch_true,yaw_true\n");

    eskf::InnovationWriter innov_writer;
    innov_writer.open(innov_path);

    // ================================================================
    // Khởi tạo ESKF
    // ================================================================
    eskf::ESKF15 filter;

    // Đọc hàng đầu để lấy t0
    eskf::LogRow row;
    if (!reader.read_next(row)) {
        std::printf("LỖI: File input trống\n");
        return 1;
    }

    filter.init(row.t);

    // ================================================================
    // Biến thống kê
    // ================================================================
    int n_rows = 0;
    double sum_sq_pos[3] = {0, 0, 0};
    double sum_sq_vel[3] = {0, 0, 0};
    double sum_sq_att[3] = {0, 0, 0};  // euler error (rad)
    double prev_t = row.t;

    auto t_start = std::chrono::high_resolution_clock::now();

    // ================================================================
    // Main loop — xử lý từng hàng
    // ================================================================
    do {
        // ----- IMU predict -----
        eskf::ImuSample imu;
        imu.t = row.t;
        imu.gyro = row.imu_gyro;
        imu.accel = row.imu_accel;
        imu.dt = row.t - prev_t;
        if (imu.dt <= 0) imu.dt = eskf::IMU_DT;
        prev_t = row.t;

        filter.predict(imu);

        // ----- GPS update -----
        if (row.gps_valid) {
            eskf::GpsSample gps;
            gps.t_stamp = row.gps_ts;
            gps.t_receive = row.t;
            gps.pos = row.gps_pos;
            gps.vel = row.gps_vel;
            gps.valid = true;

            eskf::InnovationRecord rec = filter.update_gps(gps);
            innov_writer.write(rec);
        }

        // ----- MAG update -----
        if (row.mag_valid) {
            eskf::MagSample mag;
            mag.t = row.t;
            mag.mag_body = row.mag_meas;
            mag.valid = true;

            eskf::InnovationRecord rec = filter.update_mag(mag);
            innov_writer.write(rec);
        }

        // ----- Baro update -----
        if (row.baro_valid) {
            eskf::BaroSample baro;
            baro.t = row.t;
            baro.alt = row.baro_alt;
            baro.valid = true;

            eskf::InnovationRecord rec = filter.update_baro(baro);
            innov_writer.write(rec);
        }

        // ----- Ghi estimated output -----
        const eskf::NominalState& est = filter.state();
        eskf::Vec3 euler_est = eskf::quat_to_euler(est.q);
        eskf::Vec3 euler_true = eskf::quat_to_euler(row.truth_quat);

        std::fprintf(fp_est,
            "%.6f,"
            "%.8f,%.8f,%.8f,"   // p_est
            "%.8f,%.8f,%.8f,"   // v_est
            "%.8f,%.8f,%.8f,%.8f,"  // q_est
            "%.8f,%.8f,%.8f,"   // bg_est
            "%.8f,%.8f,%.8f,"   // ba_est
            "%.8f,%.8f,%.8f,"   // p_true
            "%.8f,%.8f,%.8f,"   // v_true
            "%.8f,%.8f,%.8f,%.8f,"  // q_true
            "%.8f,%.8f,%.8f,"   // bg_true
            "%.8f,%.8f,%.8f,"   // ba_true
            "%.6f,%.6f,%.6f,"   // euler_est (rad)
            "%.6f,%.6f,%.6f\n", // euler_true (rad)
            row.t,
            est.p[0], est.p[1], est.p[2],
            est.v[0], est.v[1], est.v[2],
            est.q[0], est.q[1], est.q[2], est.q[3],
            est.bg[0], est.bg[1], est.bg[2],
            est.ba[0], est.ba[1], est.ba[2],
            row.truth_pos[0], row.truth_pos[1], row.truth_pos[2],
            row.truth_vel[0], row.truth_vel[1], row.truth_vel[2],
            row.truth_quat[0], row.truth_quat[1], row.truth_quat[2], row.truth_quat[3],
            row.truth_bg[0], row.truth_bg[1], row.truth_bg[2],
            row.truth_ba[0], row.truth_ba[1], row.truth_ba[2],
            euler_est[0], euler_est[1], euler_est[2],
            euler_true[0], euler_true[1], euler_true[2]);

        // ----- Tính RMSE -----
        for (int i = 0; i < 3; ++i) {
            double ep = est.p[i] - row.truth_pos[i];
            sum_sq_pos[i] += ep * ep;
            double ev = est.v[i] - row.truth_vel[i];
            sum_sq_vel[i] += ev * ev;
            double ea = eskf::wrap_pi(euler_est[i] - euler_true[i]);
            sum_sq_att[i] += ea * ea;
        }
        n_rows++;

    } while (reader.read_next(row));

    auto t_end = std::chrono::high_resolution_clock::now();
    double elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    // ================================================================
    // Đóng files
    // ================================================================
    std::fclose(fp_est);
    innov_writer.close();
    reader.close();

    // ================================================================
    // In summary
    // ================================================================
    double rmse_pos = rmse(sum_sq_pos, n_rows, 3);
    double rmse_vel = rmse(sum_sq_vel, n_rows, 3);
    double rmse_att = rmse(sum_sq_att, n_rows, 3);

    std::printf("\n=== KẾT QUẢ ===\n");
    std::printf("  Số hàng xử lý:    %d\n", n_rows);
    std::printf("  Thời gian:         %.1f ms (%.2f µs/step)\n",
                elapsed_ms, elapsed_ms * 1000.0 / n_rows);
    std::printf("  RMSE position:     %.4f m\n", rmse_pos);
    std::printf("  RMSE velocity:     %.4f m/s\n", rmse_vel);
    std::printf("  RMSE attitude:     %.4f rad (%.2f deg)\n",
                rmse_att, rmse_att * 180.0 / M_PI);
    std::printf("  GPS accept/reject: %d / %d\n",
                filter.gps_accept_count(), filter.gps_reject_count());
    std::printf("  MAG accept/reject: %d / %d\n",
                filter.mag_accept_count(), filter.mag_reject_count());
    std::printf("  Output:            %s, %s\n", est_path, innov_path);
    std::printf("===============\n");

    return 0;
}
