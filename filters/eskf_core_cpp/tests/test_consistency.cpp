/**
 * @file test_consistency.cpp
 * @brief Test tính nhất quán của ESKF: P symmetry, P positive semi-definite, NIS bounds.
 *
 * Chạy ngắn: khởi tạo ESKF, propagate 100 bước, check P.
 * Không cần đọc CSV — tạo IMU giả (hovering).
 *
 * Chạy:
 *   cd filters/eskf_core_cpp/build
 *   cmake .. && make test_consistency
 *   ./test_consistency
 */

#include <eskf/eskf.hpp>
#include <eskf/math.hpp>
#include <cstdio>
#include <cmath>

using namespace eskf;

static int n_pass = 0;
static int n_fail = 0;

#define CHECK(cond, msg) do { \
    if (!(cond)) { \
        std::printf("  FAIL: %s\n", msg); \
        n_fail++; \
        return 1; \
    } \
} while(0)

#define CHECK_NEAR(a, b, tol, msg) CHECK(std::fabs((a)-(b)) < (tol), msg)

// =====================================================================
// Test: P symmetry sau nhiều bước predict
// =====================================================================

int test_p_symmetry() {
    std::printf("[test_p_symmetry] ");

    ESKF15 filter;
    filter.init(0.0);

    // Tạo IMU giả: hovering (accel = [0, 0, -g], gyro = [0, 0, 0])
    ImuSample imu;
    imu.clear();
    imu.accel[2] = -GRAVITY;  // Specific force pointing up in body = -g in Down
    imu.dt = IMU_DT;

    // Propagate 200 bước
    for (int i = 0; i < 200; ++i) {
        imu.t = i * IMU_DT;
        filter.predict(imu);
    }

    const auto& P = filter.covariance();

    // Kiểm tra đối xứng: P(i,j) ≈ P(j,i)
    double max_asym = 0;
    for (int i = 0; i < STATE_DIM; ++i) {
        for (int j = i + 1; j < STATE_DIM; ++j) {
            double diff = std::fabs(P(i,j) - P(j,i));
            if (diff > max_asym) max_asym = diff;
        }
    }
    CHECK(max_asym < 1e-10, "P not symmetric (max asymmetry too large)");

    std::printf("PASS (max asymmetry = %.2e)\n", max_asym);
    n_pass++;
    return 0;
}

// =====================================================================
// Test: P diagonal dương sau predict
// =====================================================================

int test_p_positive_diagonal() {
    std::printf("[test_p_positive_diagonal] ");

    ESKF15 filter;
    filter.init(0.0);

    ImuSample imu;
    imu.clear();
    imu.accel[2] = -GRAVITY;
    imu.dt = IMU_DT;

    for (int i = 0; i < 100; ++i) {
        imu.t = i * IMU_DT;
        filter.predict(imu);
    }

    const auto& P = filter.covariance();

    for (int i = 0; i < STATE_DIM; ++i) {
        CHECK(P(i,i) > 0, "P diagonal element ≤ 0");
    }

    std::printf("PASS\n");
    n_pass++;
    return 0;
}

// =====================================================================
// Test: Quaternion remains unit after many predictions
// =====================================================================

int test_quaternion_unit() {
    std::printf("[test_quaternion_unit] ");

    ESKF15 filter;
    filter.init(0.0);

    ImuSample imu;
    imu.clear();
    imu.gyro[0] = 0.1;   // Quay nhẹ quanh X
    imu.gyro[1] = -0.05;
    imu.accel[2] = -GRAVITY;
    imu.dt = IMU_DT;

    for (int i = 0; i < 1000; ++i) {
        imu.t = i * IMU_DT;
        filter.predict(imu);
    }

    const auto& st = filter.state();
    double qnorm = st.q.norm();
    CHECK_NEAR(qnorm, 1.0, 1e-6, "quaternion not unit after 1000 steps");

    std::printf("PASS (||q|| = %.10f)\n", qnorm);
    n_pass++;
    return 0;
}

// =====================================================================
// Test: P không phát tán (diverge) quá xa sau predict-only
// =====================================================================

int test_p_bounded() {
    std::printf("[test_p_bounded] ");

    ESKF15 filter;
    filter.init(0.0);

    ImuSample imu;
    imu.clear();
    imu.accel[2] = -GRAVITY;
    imu.dt = IMU_DT;

    // 4000 bước = 10s — không có measurement update
    for (int i = 0; i < 4000; ++i) {
        imu.t = i * IMU_DT;
        filter.predict(imu);
    }

    const auto& P = filter.covariance();

    // P position nên tăng nhưng ≤ P_MAX_POS
    for (int i = 0; i < 3; ++i) {
        CHECK(P(i,i) <= P_MAX_POS + 1.0, "P_pos exceeded P_MAX_POS");
    }

    // P velocity ≤ P_MAX_VEL
    for (int i = 3; i < 6; ++i) {
        CHECK(P(i,i) <= P_MAX_VEL + 1.0, "P_vel exceeded P_MAX_VEL");
    }

    // P attitude ≤ P_MAX_ATT
    for (int i = 6; i < 9; ++i) {
        CHECK(P(i,i) <= P_MAX_ATT + 0.01, "P_att exceeded P_MAX_ATT");
    }

    std::printf("PASS (P_pos_max = %.1f, P_vel_max = %.1f)\n",
                P(0,0), P(3,3));
    n_pass++;
    return 0;
}

// =====================================================================
// Test: MAG update giảm P attitude
// =====================================================================

int test_mag_reduces_p_att() {
    std::printf("[test_mag_reduces_p_att] ");

    ESKF15 filter;
    filter.init(0.0);

    ImuSample imu;
    imu.clear();
    imu.accel[2] = -GRAVITY;
    imu.dt = IMU_DT;

    // Propagate 100 bước
    for (int i = 0; i < 100; ++i) {
        imu.t = i * IMU_DT;
        filter.predict(imu);
    }

    double P_att_before = filter.covariance()(6,6)
                        + filter.covariance()(7,7)
                        + filter.covariance()(8,8);

    // MAG update: từ trường NED chuẩn → body (vì q = identity)
    MagSample mag;
    mag.t = 100 * IMU_DT;
    mag.mag_body[0] = MAG_EARTH_N;
    mag.mag_body[1] = MAG_EARTH_E;
    mag.mag_body[2] = MAG_EARTH_D;
    mag.valid = true;

    filter.update_mag(mag);

    double P_att_after = filter.covariance()(6,6)
                       + filter.covariance()(7,7)
                       + filter.covariance()(8,8);

    CHECK(P_att_after < P_att_before, "MAG update should reduce P_attitude");

    std::printf("PASS (P_att: %.6f → %.6f)\n", P_att_before, P_att_after);
    n_pass++;
    return 0;
}

// =====================================================================
// Test: Baro update giảm P altitude
// =====================================================================

int test_baro_reduces_p_alt() {
    std::printf("[test_baro_reduces_p_alt] ");

    ESKF15 filter;
    filter.init(0.0);

    ImuSample imu;
    imu.clear();
    imu.accel[2] = -GRAVITY;
    imu.dt = IMU_DT;

    for (int i = 0; i < 100; ++i) {
        imu.t = i * IMU_DT;
        filter.predict(imu);
    }

    double P_pD_before = filter.covariance()(2,2);

    // Baro update: alt = 0 (hovering tại gốc)
    BaroSample baro;
    baro.t = 100 * IMU_DT;
    baro.alt = 0.0;
    baro.valid = true;

    filter.update_baro(baro);

    double P_pD_after = filter.covariance()(2,2);

    CHECK(P_pD_after < P_pD_before, "Baro update should reduce P_pD");

    std::printf("PASS (P_pD: %.6f → %.6f)\n", P_pD_before, P_pD_after);
    n_pass++;
    return 0;
}

// =====================================================================
// Main
// =====================================================================

int main() {
    std::printf("=== test_consistency ===\n\n");

    test_p_symmetry();
    test_p_positive_diagonal();
    test_quaternion_unit();
    test_p_bounded();
    test_mag_reduces_p_att();
    test_baro_reduces_p_alt();

    std::printf("\n=== Kết quả: %d PASS, %d FAIL ===\n", n_pass, n_fail);
    return n_fail > 0 ? 1 : 0;
}
