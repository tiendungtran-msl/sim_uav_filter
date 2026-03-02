/**
 * @file test_buffer.cpp
 * @brief Unit tests cho RingBuffer template.
 *
 * Chạy:
 *   cd filters/eskf_core_cpp/build
 *   cmake .. && make test_buffer
 *   ./test_buffer
 */

#include <eskf/ring_buffer.hpp>
#include <eskf/eskf_types.hpp>
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
// Test: Push + size
// =====================================================================

int test_push_and_size() {
    std::printf("[test_push_and_size] ");
    RingBuffer<ImuSample, 8> buf;
    buf.reset();

    CHECK(buf.empty(), "buffer should be empty after reset");
    CHECK(buf.size() == 0, "size should be 0");

    ImuSample s;
    s.clear();
    s.t = 1.0;
    buf.push(s);

    CHECK(!buf.empty(), "buffer should not be empty after push");
    CHECK(buf.size() == 1, "size should be 1");

    // Push thêm 7 cái
    for (int i = 2; i <= 8; ++i) {
        s.t = (double)i;
        buf.push(s);
    }
    CHECK(buf.size() == 8, "size should be 8 (full)");

    // Push thêm 1 cái → ghi đè cũ nhất, size vẫn 8
    s.t = 9.0;
    buf.push(s);
    CHECK(buf.size() == 8, "size should still be 8 after overflow");

    std::printf("PASS\n");
    n_pass++;
    return 0;
}

// =====================================================================
// Test: at() — truy cập theo index
// =====================================================================

int test_at() {
    std::printf("[test_at] ");
    RingBuffer<ImuSample, 8> buf;
    buf.reset();

    // Push 5 elements: t = 10, 20, 30, 40, 50
    for (int i = 1; i <= 5; ++i) {
        ImuSample s;
        s.clear();
        s.t = i * 10.0;
        buf.push(s);
    }

    // at(0) = oldest = t=10
    CHECK_NEAR(buf.at(0).t, 10.0, 0.001, "at(0) should be oldest (10)");
    // at(4) = newest = t=50
    CHECK_NEAR(buf.at(4).t, 50.0, 0.001, "at(4) should be newest (50)");
    // at(2) = t=30
    CHECK_NEAR(buf.at(2).t, 30.0, 0.001, "at(2) should be 30");

    std::printf("PASS\n");
    n_pass++;
    return 0;
}

// =====================================================================
// Test: find_nearest — tìm phần tử gần nhất theo timestamp
// =====================================================================

int test_find_nearest() {
    std::printf("[test_find_nearest] ");
    RingBuffer<ImuSample, 16> buf;
    buf.reset();

    // Push IMU at t = 0.000, 0.0025, 0.005, ..., 0.0375 (16 samples)
    for (int i = 0; i < 16; ++i) {
        ImuSample s;
        s.clear();
        s.t = i * 0.0025;
        buf.push(s);
    }

    // Tìm gần t = 0.006 → nearest là t=0.005 (index 2) hoặc t=0.0075 (index 3)
    int idx = buf.find_nearest(0.006);
    CHECK(idx >= 0, "find_nearest should return valid index");
    double t_found = buf.at(idx).t;
    double err = std::fabs(t_found - 0.006);
    CHECK(err < 0.002, "find_nearest: found sample too far");

    // Tìm chính xác
    idx = buf.find_nearest(0.01);
    CHECK(idx >= 0, "find_nearest exact");
    CHECK_NEAR(buf.at(idx).t, 0.01, 0.001, "find_nearest exact value");

    std::printf("PASS\n");
    n_pass++;
    return 0;
}

// =====================================================================
// Test: find_latest_before — tìm sample mới nhất có t ≤ target
// =====================================================================

int test_find_latest_before() {
    std::printf("[test_find_latest_before] ");
    RingBuffer<ImuSample, 16> buf;
    buf.reset();

    for (int i = 0; i < 10; ++i) {
        ImuSample s;
        s.clear();
        s.t = i * 1.0;
        buf.push(s);
    }

    // Tìm latest before t=5.5 → t=5
    int idx = buf.find_latest_before(5.5);
    CHECK(idx >= 0, "find_latest_before should find");
    CHECK_NEAR(buf.at(idx).t, 5.0, 0.001, "find_latest_before: should be 5.0");

    // Tìm latest before t=0.0 → t=0
    idx = buf.find_latest_before(0.0);
    CHECK(idx >= 0, "find_latest_before at t=0");
    CHECK_NEAR(buf.at(idx).t, 0.0, 0.001, "find_latest_before: should be 0.0");

    std::printf("PASS\n");
    n_pass++;
    return 0;
}

// =====================================================================
// Test: get_after — lấy danh sách phần tử có t > target
// =====================================================================

int test_get_after() {
    std::printf("[test_get_after] ");
    RingBuffer<ImuSample, 16> buf;
    buf.reset();

    // Push 10 elements: t = 0, 1, 2, ..., 9
    for (int i = 0; i < 10; ++i) {
        ImuSample s;
        s.clear();
        s.t = i * 1.0;
        buf.push(s);
    }

    // Get after t=6 → should get t=7, 8, 9 (3 elements)
    ImuSample out[16];
    int n = buf.get_after(6.0, out, 16);
    CHECK(n == 3, "get_after(6): expected 3 elements");
    CHECK_NEAR(out[0].t, 7.0, 0.001, "get_after[0] = 7");
    CHECK_NEAR(out[1].t, 8.0, 0.001, "get_after[1] = 8");
    CHECK_NEAR(out[2].t, 9.0, 0.001, "get_after[2] = 9");

    // Get after t=-1 → tất cả 10 elements
    n = buf.get_after(-1.0, out, 16);
    CHECK(n == 10, "get_after(-1): expected 10 elements");

    std::printf("PASS\n");
    n_pass++;
    return 0;
}

// =====================================================================
// Test: Overflow — ghi đè vòng
// =====================================================================

int test_overflow() {
    std::printf("[test_overflow] ");
    RingBuffer<ImuSample, 4> buf;  // Chỉ 4 slots
    buf.reset();

    // Push 6 elements: t = 10, 20, 30, 40, 50, 60
    for (int i = 1; i <= 6; ++i) {
        ImuSample s;
        s.clear();
        s.t = i * 10.0;
        buf.push(s);
    }

    // Size vẫn = 4 (capacity)
    CHECK(buf.size() == 4, "size should be 4");

    // Oldest should be t=30 (10 và 20 đã bị ghi đè)
    CHECK_NEAR(buf.at(0).t, 30.0, 0.001, "oldest after overflow should be 30");
    // Newest should be t=60
    CHECK_NEAR(buf.at(3).t, 60.0, 0.001, "newest should be 60");

    std::printf("PASS\n");
    n_pass++;
    return 0;
}

// =====================================================================
// Test: Reset
// =====================================================================

int test_reset() {
    std::printf("[test_reset] ");
    RingBuffer<ImuSample, 8> buf;
    buf.reset();

    // Push vài items
    for (int i = 0; i < 5; ++i) {
        ImuSample s;
        s.clear();
        s.t = i * 1.0;
        buf.push(s);
    }
    CHECK(buf.size() == 5, "size before reset");

    buf.reset();
    CHECK(buf.size() == 0, "size after reset should be 0");
    CHECK(buf.empty(), "should be empty after reset");

    std::printf("PASS\n");
    n_pass++;
    return 0;
}

// =====================================================================
// Test: StateSnapshot buffer
// =====================================================================

int test_snapshot_buffer() {
    std::printf("[test_snapshot_buffer] ");
    RingBuffer<StateSnapshot, 8> snapbuf;
    snapbuf.reset();

    // Push 5 snapshots
    for (int i = 0; i < 5; ++i) {
        StateSnapshot snap;
        snap.clear();
        snap.t = i * 0.025;  // Mỗi 25ms
        snap.valid = true;
        snapbuf.push(snap);
    }

    CHECK(snapbuf.size() == 5, "snapshot buffer size");

    // Tìm snapshot gần t = 0.06 → nearest là 0.05 (index 2) hoặc 0.075 (index 3)
    int idx = snapbuf.find_nearest(0.06);
    CHECK(idx >= 0, "find snapshot");
    double t_snap = snapbuf.at(idx).t;
    CHECK(std::fabs(t_snap - 0.06) < 0.02, "snapshot timestamp close enough");

    std::printf("PASS\n");
    n_pass++;
    return 0;
}

// =====================================================================
// Main
// =====================================================================

int main() {
    std::printf("=== test_buffer ===\n\n");

    test_push_and_size();
    test_at();
    test_find_nearest();
    test_find_latest_before();
    test_get_after();
    test_overflow();
    test_reset();
    test_snapshot_buffer();

    std::printf("\n=== Kết quả: %d PASS, %d FAIL ===\n", n_pass, n_fail);
    return n_fail > 0 ? 1 : 0;
}
