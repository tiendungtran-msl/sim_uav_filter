/**
 * @file math.hpp
 * @brief Hàm toán học 3D: quaternion, rotation matrix, skew-symmetric.
 *
 * Quy ước:
 * - Quaternion: q = [w, x, y, z] (Hamilton, scalar-first)
 * - Frame: q biểu diễn xoay body → NED
 *   nghĩa là v_NED = R(q) * v_body
 * - Rotation vector φ: trục × góc (rad), dùng cho error-state δθ
 *
 * Tất cả hàm KHÔNG dùng heap, chỉ dùng Mat<> trên stack.
 * Tương thích embedded (NuttX/RTOS).
 */

#ifndef ESKF_MATH_HPP
#define ESKF_MATH_HPP

#include "static_matrix.hpp"
#include <cmath>

namespace eskf {

// =====================================================================
// Quaternion cơ bản
// =====================================================================

/**
 * Nhân Hamilton hai quaternion: q ⊗ r
 *
 * Phép nhân quaternion tiêu chuẩn Hamilton:
 *   (q0 + q1*i + q2*j + q3*k) × (r0 + r1*i + r2*j + r3*k)
 */
inline Vec4 quat_mult(const Vec4& q, const Vec4& r) {
    Vec4 out;
    double w0 = q[0], x0 = q[1], y0 = q[2], z0 = q[3];
    double w1 = r[0], x1 = r[1], y1 = r[2], z1 = r[3];
    out[0] = w0*w1 - x0*x1 - y0*y1 - z0*z1;
    out[1] = w0*x1 + x0*w1 + y0*z1 - z0*y1;
    out[2] = w0*y1 - x0*z1 + y0*w1 + z0*x1;
    out[3] = w0*z1 + x0*y1 - y0*x1 + z0*w1;
    return out;
}

/**
 * Liên hợp (conjugate) quaternion: q* = [w, -x, -y, -z]
 * Đây cũng là nghịch đảo nếu q là unit quaternion.
 */
inline Vec4 quat_conj(const Vec4& q) {
    Vec4 out;
    out[0] = q[0]; out[1] = -q[1]; out[2] = -q[2]; out[3] = -q[3];
    return out;
}

/**
 * Chuẩn hoá quaternion về đơn vị.
 * Nếu norm ≈ 0 → trả về identity [1,0,0,0].
 */
inline Vec4 quat_normalize(const Vec4& q) {
    double n = q.norm();
    if (n < 1e-12) {
        Vec4 id; id[0] = 1; id[1] = 0; id[2] = 0; id[3] = 0;
        return id;
    }
    return q * (1.0 / n);
}

/**
 * Xoay vector v bằng quaternion q: v' = q ⊗ [0,v] ⊗ q*
 *
 * Nếu q là body→NED thì:
 *   v_NED = quat_rotate(q, v_body)
 */
inline Vec3 quat_rotate(const Vec4& q, const Vec3& v) {
    Vec4 v_q;
    v_q[0] = 0.0; v_q[1] = v[0]; v_q[2] = v[1]; v_q[3] = v[2];
    Vec4 res = quat_mult(quat_mult(q, v_q), quat_conj(q));
    Vec3 out;
    out[0] = res[1]; out[1] = res[2]; out[2] = res[3];
    return out;
}

/**
 * Tạo quaternion từ rotation vector (axis × angle).
 *
 * Công thức:
 *   angle = ||φ||
 *   axis = φ / angle
 *   q = [cos(angle/2), axis * sin(angle/2)]
 *
 * Xử lý góc nhỏ bằng xấp xỉ Taylor bậc 1 để tránh chia 0.
 */
inline Vec4 quat_from_rotvec(const Vec3& phi) {
    double angle = phi.norm();
    Vec4 q;
    if (angle < 1e-12) {
        // Xấp xỉ bậc 1: sin(a/2)/a ≈ 0.5
        q[0] = 1.0;
        q[1] = phi[0] * 0.5;
        q[2] = phi[1] * 0.5;
        q[3] = phi[2] * 0.5;
    } else {
        double half = angle * 0.5;
        double s = std::sin(half) / angle;
        q[0] = std::cos(half);
        q[1] = phi[0] * s;
        q[2] = phi[1] * s;
        q[3] = phi[2] * s;
    }
    return quat_normalize(q);
}

// =====================================================================
// Ma trận xoay từ quaternion
// =====================================================================

/**
 * Tạo ma trận xoay 3×3 (DCM) từ quaternion body→NED.
 *
 * R sao cho: v_NED = R * v_body
 *
 * Lưu ý: Đây là DCM chuẩn, KHÔNG cần transpose thêm.
 */
inline Mat3 rot_from_quat(const Vec4& q) {
    double w = q[0], x = q[1], y = q[2], z = q[3];
    double xx = x*x, yy = y*y, zz = z*z;
    double xy = x*y, xz = x*z, yz = y*z;
    double wx = w*x, wy = w*y, wz = w*z;

    Mat3 R;
    R(0,0) = 1 - 2*(yy + zz);  R(0,1) = 2*(xy - wz);      R(0,2) = 2*(xz + wy);
    R(1,0) = 2*(xy + wz);      R(1,1) = 1 - 2*(xx + zz);  R(1,2) = 2*(yz - wx);
    R(2,0) = 2*(xz - wy);      R(2,1) = 2*(yz + wx);      R(2,2) = 1 - 2*(xx + yy);
    return R;
}

/**
 * Chuyển quaternion → Euler angles (roll, pitch, yaw) theo ZYX convention.
 * Trả Vec3 = [roll, pitch, yaw] tính bằng rad.
 *
 * roll  = atan2(2(wx + yz), 1 - 2(xx + yy))
 * pitch = asin(2(wy - xz))
 * yaw   = atan2(2(wz + xy), 1 - 2(yy + zz))
 */
inline Vec3 quat_to_euler(const Vec4& q) {
    double w = q[0], x = q[1], y = q[2], z = q[3];

    double sinp = 2.0 * (w*y - z*x);
    // Clamp để tránh NaN ở gimbal lock
    if (sinp > 1.0) sinp = 1.0;
    if (sinp < -1.0) sinp = -1.0;

    Vec3 euler;
    euler[0] = std::atan2(2.0*(w*x + y*z), 1.0 - 2.0*(x*x + y*y));  // roll
    euler[1] = std::asin(sinp);                                        // pitch
    euler[2] = std::atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z));  // yaw
    return euler;
}

// =====================================================================
// Skew-symmetric matrix
// =====================================================================

/**
 * Tạo ma trận skew-symmetric (phản đối xứng) từ vector 3D.
 *
 * [v]× = [ 0   -vz   vy ]
 *         [ vz   0   -vx ]
 *         [-vy   vx   0  ]
 *
 * Tính chất: [v]× * u = v × u (tích có hướng)
 *
 * Dùng trong:
 * - Đạo hàm rotation error: -R*[a×]
 * - Propagation ma trận F
 */
inline Mat3 skew(const Vec3& v) {
    Mat3 S = Mat3::zero();
    S(0,1) = -v[2];  S(0,2) =  v[1];
    S(1,0) =  v[2];  S(1,2) = -v[0];
    S(2,0) = -v[1];  S(2,1) =  v[0];
    return S;
}

// =====================================================================
// Cross product
// =====================================================================

/** Tích có hướng: a × b */
inline Vec3 cross(const Vec3& a, const Vec3& b) {
    Vec3 r;
    r[0] = a[1]*b[2] - a[2]*b[1];
    r[1] = a[2]*b[0] - a[0]*b[2];
    r[2] = a[0]*b[1] - a[1]*b[0];
    return r;
}

/** Dot product */
inline double dot(const Vec3& a, const Vec3& b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

// =====================================================================
// Wrap angle — giữ góc trong [-π, π]
// =====================================================================

inline double wrap_pi(double x) {
    while (x > M_PI)  x -= 2.0 * M_PI;
    while (x < -M_PI) x += 2.0 * M_PI;
    return x;
}

}  // namespace eskf

#endif  // ESKF_MATH_HPP
