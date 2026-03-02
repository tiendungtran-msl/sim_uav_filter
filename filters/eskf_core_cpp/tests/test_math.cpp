/**
 * @file test_math.cpp
 * @brief Unit tests cho math.hpp — quaternion, rotation, skew-symmetric.
 *
 * Chạy:
 *   cd filters/eskf_core_cpp/build
 *   cmake .. && make test_math
 *   ./test_math
 *
 * Quy ước test: mỗi hàm test trả 0 nếu PASS, 1 nếu FAIL.
 * Tổng hợp kết quả ở main().
 */

#include <eskf/math.hpp>
#include <eskf/static_matrix.hpp>
#include <cstdio>
#include <cmath>

using namespace eskf;

static constexpr double TOL = 1e-10;
static constexpr double TOL_LOOSE = 1e-6;

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
// Test: Ma trận tĩnh cơ bản
// =====================================================================

int test_mat_identity() {
    std::printf("[test_mat_identity] ");
    auto I = Mat3::identity();
    CHECK_NEAR(I(0,0), 1.0, TOL, "I(0,0) != 1");
    CHECK_NEAR(I(1,1), 1.0, TOL, "I(1,1) != 1");
    CHECK_NEAR(I(2,2), 1.0, TOL, "I(2,2) != 1");
    CHECK_NEAR(I(0,1), 0.0, TOL, "I(0,1) != 0");
    CHECK_NEAR(I(1,0), 0.0, TOL, "I(1,0) != 0");
    std::printf("PASS\n");
    n_pass++;
    return 0;
}

int test_mat_multiply() {
    std::printf("[test_mat_multiply] ");
    auto I = Mat3::identity();
    Vec3 v = Vec3::zero();
    v[0] = 1.0; v[1] = 2.0; v[2] = 3.0;
    auto r = I * v;
    CHECK_NEAR(r[0], 1.0, TOL, "I*v[0] != 1");
    CHECK_NEAR(r[1], 2.0, TOL, "I*v[1] != 2");
    CHECK_NEAR(r[2], 3.0, TOL, "I*v[2] != 3");
    std::printf("PASS\n");
    n_pass++;
    return 0;
}

int test_mat_transpose() {
    std::printf("[test_mat_transpose] ");
    Mat<2,3> A = Mat<2,3>::zero();
    A(0,0) = 1; A(0,1) = 2; A(0,2) = 3;
    A(1,0) = 4; A(1,1) = 5; A(1,2) = 6;
    auto At = A.T();
    CHECK_NEAR(At(0,0), 1.0, TOL, "At(0,0)");
    CHECK_NEAR(At(1,0), 2.0, TOL, "At(1,0)");
    CHECK_NEAR(At(2,0), 3.0, TOL, "At(2,0)");
    CHECK_NEAR(At(0,1), 4.0, TOL, "At(0,1)");
    CHECK_NEAR(At(1,1), 5.0, TOL, "At(1,1)");
    CHECK_NEAR(At(2,1), 6.0, TOL, "At(2,1)");
    std::printf("PASS\n");
    n_pass++;
    return 0;
}

int test_mat_invert3() {
    std::printf("[test_mat_invert3] ");
    // A = [[2,1,0],[1,3,1],[0,1,2]]  — symmetric positive definite
    Mat3 A = Mat3::zero();
    A(0,0) = 2; A(0,1) = 1; A(0,2) = 0;
    A(1,0) = 1; A(1,1) = 3; A(1,2) = 1;
    A(2,0) = 0; A(2,1) = 1; A(2,2) = 2;

    Mat3 A_inv;
    bool ok = invert3(A, A_inv);
    CHECK(ok, "invert3 failed");

    // A * A_inv ≈ I
    auto prod = A * A_inv;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double expected = (i == j) ? 1.0 : 0.0;
            CHECK_NEAR(prod(i,j), expected, TOL_LOOSE, "A*A_inv != I");
        }
    }
    std::printf("PASS\n");
    n_pass++;
    return 0;
}

int test_mat_invert6() {
    std::printf("[test_mat_invert6] ");
    // 6×6 block diagonal: [[2I, 0], [0, 3I]]
    auto A = Mat<6,6>::zero();
    for (int i = 0; i < 3; ++i) A(i,i) = 2.0;
    for (int i = 3; i < 6; ++i) A(i,i) = 3.0;

    Mat<6,6> A_inv;
    bool ok = invert6(A, A_inv);
    CHECK(ok, "invert6 failed");

    auto prod = A * A_inv;
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            double expected = (i == j) ? 1.0 : 0.0;
            CHECK_NEAR(prod(i,j), expected, TOL_LOOSE, "A*A_inv != I (6x6)");
        }
    }
    std::printf("PASS\n");
    n_pass++;
    return 0;
}

// =====================================================================
// Test: Quaternion
// =====================================================================

int test_quat_identity() {
    std::printf("[test_quat_identity] ");
    Vec4 q; q[0] = 1; q[1] = 0; q[2] = 0; q[3] = 0;
    Vec3 v; v[0] = 1; v[1] = 2; v[2] = 3;

    Vec3 r = quat_rotate(q, v);
    CHECK_NEAR(r[0], 1.0, TOL, "quat_rotate with identity q[0]");
    CHECK_NEAR(r[1], 2.0, TOL, "quat_rotate with identity q[1]");
    CHECK_NEAR(r[2], 3.0, TOL, "quat_rotate with identity q[2]");
    std::printf("PASS\n");
    n_pass++;
    return 0;
}

int test_quat_90_yaw() {
    std::printf("[test_quat_90_yaw] ");
    // 90° yaw (quay quanh trục Z/Down):
    // body X (North) → NED East direction
    Vec3 phi = Vec3::zero();
    phi[2] = M_PI / 2.0;  // 90° quanh Z
    Vec4 q = quat_from_rotvec(phi);

    Vec3 body_x; body_x[0] = 1; body_x[1] = 0; body_x[2] = 0;
    Vec3 ned = quat_rotate(q, body_x);

    // Body X rotated 90° yaw → should be East (0, 1, 0)
    CHECK_NEAR(ned[0], 0.0, TOL_LOOSE, "90° yaw: N != 0");
    CHECK_NEAR(ned[1], 1.0, TOL_LOOSE, "90° yaw: E != 1");
    CHECK_NEAR(ned[2], 0.0, TOL_LOOSE, "90° yaw: D != 0");
    std::printf("PASS\n");
    n_pass++;
    return 0;
}

int test_quat_mult_inverse() {
    std::printf("[test_quat_mult_inverse] ");
    // q ⊗ q* = identity
    Vec3 phi; phi[0] = 0.3; phi[1] = -0.5; phi[2] = 0.7;
    Vec4 q = quat_from_rotvec(phi);
    Vec4 q_inv = quat_conj(q);
    Vec4 prod = quat_mult(q, q_inv);

    CHECK_NEAR(prod[0], 1.0, TOL_LOOSE, "q*q_inv: w != 1");
    CHECK_NEAR(prod[1], 0.0, TOL_LOOSE, "q*q_inv: x != 0");
    CHECK_NEAR(prod[2], 0.0, TOL_LOOSE, "q*q_inv: y != 0");
    CHECK_NEAR(prod[3], 0.0, TOL_LOOSE, "q*q_inv: z != 0");
    std::printf("PASS\n");
    n_pass++;
    return 0;
}

int test_quat_normalize() {
    std::printf("[test_quat_normalize] ");
    Vec4 q; q[0] = 2; q[1] = 0; q[2] = 0; q[3] = 0;
    Vec4 qn = quat_normalize(q);
    CHECK_NEAR(qn[0], 1.0, TOL, "normalize: w != 1");
    CHECK_NEAR(qn.norm(), 1.0, TOL, "normalize: norm != 1");
    std::printf("PASS\n");
    n_pass++;
    return 0;
}

int test_quat_from_rotvec_small() {
    std::printf("[test_quat_from_rotvec_small] ");
    // Góc rất nhỏ → xấp xỉ Taylor
    Vec3 phi; phi[0] = 1e-14; phi[1] = 0; phi[2] = 0;
    Vec4 q = quat_from_rotvec(phi);
    CHECK_NEAR(q[0], 1.0, TOL, "small rotvec: w != 1");
    CHECK_NEAR(q.norm(), 1.0, TOL, "small rotvec: norm != 1");
    std::printf("PASS\n");
    n_pass++;
    return 0;
}

// =====================================================================
// Test: Rotation matrix
// =====================================================================

int test_rot_from_quat_identity() {
    std::printf("[test_rot_from_quat_identity] ");
    Vec4 q; q[0] = 1; q[1] = 0; q[2] = 0; q[3] = 0;
    Mat3 R = rot_from_quat(q);

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            CHECK_NEAR(R(i,j), (i==j) ? 1.0 : 0.0, TOL, "R(identity) != I");

    std::printf("PASS\n");
    n_pass++;
    return 0;
}

int test_rot_orthogonal() {
    std::printf("[test_rot_orthogonal] ");
    // Kiểm tra R * R^T = I cho quaternion bất kỳ
    Vec3 phi; phi[0] = 0.5; phi[1] = -0.3; phi[2] = 1.2;
    Vec4 q = quat_from_rotvec(phi);
    Mat3 R = rot_from_quat(q);
    auto RRt = R * R.T();

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            CHECK_NEAR(RRt(i,j), (i==j) ? 1.0 : 0.0, TOL_LOOSE, "R*R^T != I");

    std::printf("PASS\n");
    n_pass++;
    return 0;
}

int test_rot_consistent_with_quat_rotate() {
    std::printf("[test_rot_consistent_with_quat_rotate] ");
    // R * v phải = quat_rotate(q, v)
    Vec3 phi; phi[0] = 0.7; phi[1] = -0.4; phi[2] = 0.9;
    Vec4 q = quat_from_rotvec(phi);
    Mat3 R = rot_from_quat(q);

    Vec3 v; v[0] = 1.5; v[1] = -2.3; v[2] = 0.8;
    Vec3 r1 = R * v;
    Vec3 r2 = quat_rotate(q, v);

    CHECK_NEAR(r1[0], r2[0], TOL_LOOSE, "R*v != quat_rotate [0]");
    CHECK_NEAR(r1[1], r2[1], TOL_LOOSE, "R*v != quat_rotate [1]");
    CHECK_NEAR(r1[2], r2[2], TOL_LOOSE, "R*v != quat_rotate [2]");
    std::printf("PASS\n");
    n_pass++;
    return 0;
}

// =====================================================================
// Test: Euler
// =====================================================================

int test_euler_roundtrip() {
    std::printf("[test_euler_roundtrip] ");
    // Tạo quaternion từ rotvec, chuyển sang Euler, kiểm tra range
    Vec3 phi; phi[0] = 0.3; phi[1] = -0.2; phi[2] = 1.5;
    Vec4 q = quat_from_rotvec(phi);
    Vec3 euler = quat_to_euler(q);

    // Roll, pitch, yaw phải trong [-π, π]
    CHECK(euler[0] >= -M_PI && euler[0] <= M_PI, "roll out of range");
    CHECK(euler[1] >= -M_PI/2 && euler[1] <= M_PI/2, "pitch out of range");
    CHECK(euler[2] >= -M_PI && euler[2] <= M_PI, "yaw out of range");
    std::printf("PASS\n");
    n_pass++;
    return 0;
}

int test_euler_identity() {
    std::printf("[test_euler_identity] ");
    Vec4 q; q[0] = 1; q[1] = 0; q[2] = 0; q[3] = 0;
    Vec3 euler = quat_to_euler(q);
    CHECK_NEAR(euler[0], 0.0, TOL, "identity: roll != 0");
    CHECK_NEAR(euler[1], 0.0, TOL, "identity: pitch != 0");
    CHECK_NEAR(euler[2], 0.0, TOL, "identity: yaw != 0");
    std::printf("PASS\n");
    n_pass++;
    return 0;
}

// =====================================================================
// Test: Skew-symmetric + Cross product
// =====================================================================

int test_skew_cross() {
    std::printf("[test_skew_cross] ");
    Vec3 a; a[0] = 1; a[1] = 2; a[2] = 3;
    Vec3 b; b[0] = 4; b[1] = 5; b[2] = 6;

    // [a]× * b == a × b
    Mat3 Sa = skew(a);
    Vec3 r1 = Sa * b;
    Vec3 r2 = cross(a, b);

    CHECK_NEAR(r1[0], r2[0], TOL, "skew*b != cross [0]");
    CHECK_NEAR(r1[1], r2[1], TOL, "skew*b != cross [1]");
    CHECK_NEAR(r1[2], r2[2], TOL, "skew*b != cross [2]");

    // Cross product: (1,2,3)×(4,5,6) = (2*6-3*5, 3*4-1*6, 1*5-2*4) = (-3, 6, -3)
    CHECK_NEAR(r2[0], -3.0, TOL, "cross[0] != -3");
    CHECK_NEAR(r2[1],  6.0, TOL, "cross[1] != 6");
    CHECK_NEAR(r2[2], -3.0, TOL, "cross[2] != -3");
    std::printf("PASS\n");
    n_pass++;
    return 0;
}

int test_skew_antisymmetric() {
    std::printf("[test_skew_antisymmetric] ");
    Vec3 v; v[0] = 3.14; v[1] = -2.71; v[2] = 1.41;
    Mat3 S = skew(v);

    // S + S^T = 0 (antisymmetric)
    auto sum = S + S.T();
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            CHECK_NEAR(sum(i,j), 0.0, TOL, "skew not antisymmetric");

    std::printf("PASS\n");
    n_pass++;
    return 0;
}

// =====================================================================
// Test: wrap_pi
// =====================================================================

int test_wrap_pi() {
    std::printf("[test_wrap_pi] ");
    CHECK_NEAR(wrap_pi(0.0), 0.0, TOL, "wrap_pi(0)");
    CHECK_NEAR(wrap_pi(M_PI), M_PI, TOL, "wrap_pi(pi)");
    CHECK_NEAR(wrap_pi(-M_PI), -M_PI, TOL, "wrap_pi(-pi)");
    CHECK_NEAR(wrap_pi(3 * M_PI), M_PI, TOL_LOOSE, "wrap_pi(3pi)");
    CHECK_NEAR(wrap_pi(-3 * M_PI), -M_PI, TOL_LOOSE, "wrap_pi(-3pi)");
    CHECK_NEAR(wrap_pi(2 * M_PI + 0.1), 0.1, TOL_LOOSE, "wrap_pi(2pi+0.1)");
    std::printf("PASS\n");
    n_pass++;
    return 0;
}

// =====================================================================
// Test: dot product
// =====================================================================

int test_dot() {
    std::printf("[test_dot] ");
    Vec3 a; a[0] = 1; a[1] = 2; a[2] = 3;
    Vec3 b; b[0] = 4; b[1] = 5; b[2] = 6;
    CHECK_NEAR(dot(a, b), 32.0, TOL, "dot(a,b) != 32");
    std::printf("PASS\n");
    n_pass++;
    return 0;
}

// =====================================================================
// Main
// =====================================================================

int main() {
    std::printf("=== test_math ===\n\n");

    // Ma trận
    test_mat_identity();
    test_mat_multiply();
    test_mat_transpose();
    test_mat_invert3();
    test_mat_invert6();

    // Quaternion
    test_quat_identity();
    test_quat_90_yaw();
    test_quat_mult_inverse();
    test_quat_normalize();
    test_quat_from_rotvec_small();

    // Rotation
    test_rot_from_quat_identity();
    test_rot_orthogonal();
    test_rot_consistent_with_quat_rotate();

    // Euler
    test_euler_roundtrip();
    test_euler_identity();

    // Skew / Cross
    test_skew_cross();
    test_skew_antisymmetric();

    // Misc
    test_wrap_pi();
    test_dot();

    std::printf("\n=== Kết quả: %d PASS, %d FAIL ===\n", n_pass, n_fail);
    return n_fail > 0 ? 1 : 0;
}
