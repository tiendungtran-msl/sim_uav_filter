/**
 * @file static_matrix.hpp
 * @brief Ma trận kích thước cố định compile-time — không dùng heap.
 *
 * Thiết kế cho embedded:
 * - Toàn bộ dữ liệu nằm trên stack (std::array)
 * - Không new/delete, không exceptions
 * - Kích thước xác định lúc biên dịch qua template parameters
 * - Hỗ trợ: cộng, trừ, nhân, transpose, identity, zero
 * - Dùng cho P(15×15), F(15×15), H(6×15), K(15×6), v.v.
 *
 * Lưu ý: Đây KHÔNG phải là thư viện matrix tổng quát.
 * Chỉ implement đủ phép tính cần cho ESKF.
 * Cho production-scale nên dùng Eigen (nhưng Eigen có thể dùng heap).
 */

#ifndef ESKF_STATIC_MATRIX_HPP
#define ESKF_STATIC_MATRIX_HPP

#include <cmath>
#include <cstring>

namespace eskf {

/**
 * Ma trận tĩnh R×C, dữ liệu row-major trên stack.
 *
 * Ví dụ:
 *   Mat<3,3> R = Mat<3,3>::identity();
 *   Mat<3,1> v = Mat<3,1>::zero();
 *   auto Rv = R * v;
 */
template <int ROWS, int COLS>
struct Mat {
    double d[ROWS * COLS];  ///< Dữ liệu row-major

    // -----------------------------------------------------------------
    // Truy cập phần tử
    // -----------------------------------------------------------------

    /** Truy cập phần tử (i, j), 0-indexed */
    inline double& operator()(int i, int j) { return d[i * COLS + j]; }
    inline const double& operator()(int i, int j) const { return d[i * COLS + j]; }

    /** Truy cập phẳng — dùng cho vector (Nx1 hoặc 1xN) */
    inline double& operator[](int k) { return d[k]; }
    inline const double& operator[](int k) const { return d[k]; }

    // -----------------------------------------------------------------
    // Factory methods
    // -----------------------------------------------------------------

    /** Ma trận zero */
    static Mat zero() {
        Mat m;
        for (int i = 0; i < ROWS * COLS; ++i) m.d[i] = 0.0;
        return m;
    }

    /** Ma trận đơn vị (chỉ cho ma trận vuông) */
    static Mat identity() {
        static_assert(ROWS == COLS, "identity() chỉ cho ma trận vuông");
        Mat m = zero();
        for (int i = 0; i < ROWS; ++i) m(i, i) = 1.0;
        return m;
    }

    // -----------------------------------------------------------------
    // Phép cộng / trừ
    // -----------------------------------------------------------------

    Mat operator+(const Mat& rhs) const {
        Mat r;
        for (int i = 0; i < ROWS * COLS; ++i) r.d[i] = d[i] + rhs.d[i];
        return r;
    }

    Mat operator-(const Mat& rhs) const {
        Mat r;
        for (int i = 0; i < ROWS * COLS; ++i) r.d[i] = d[i] - rhs.d[i];
        return r;
    }

    Mat& operator+=(const Mat& rhs) {
        for (int i = 0; i < ROWS * COLS; ++i) d[i] += rhs.d[i];
        return *this;
    }

    Mat& operator-=(const Mat& rhs) {
        for (int i = 0; i < ROWS * COLS; ++i) d[i] -= rhs.d[i];
        return *this;
    }

    // -----------------------------------------------------------------
    // Nhân scalar
    // -----------------------------------------------------------------

    Mat operator*(double s) const {
        Mat r;
        for (int i = 0; i < ROWS * COLS; ++i) r.d[i] = d[i] * s;
        return r;
    }

    friend Mat operator*(double s, const Mat& m) { return m * s; }

    // -----------------------------------------------------------------
    // Nhân ma trận: (R×C) * (C×K) → (R×K)
    // -----------------------------------------------------------------

    template <int K>
    Mat<ROWS, K> operator*(const Mat<COLS, K>& rhs) const {
        Mat<ROWS, K> r = Mat<ROWS, K>::zero();
        for (int i = 0; i < ROWS; ++i) {
            for (int j = 0; j < K; ++j) {
                double sum = 0.0;
                for (int k = 0; k < COLS; ++k) {
                    sum += (*this)(i, k) * rhs(k, j);
                }
                r(i, j) = sum;
            }
        }
        return r;
    }

    // -----------------------------------------------------------------
    // Transpose: (R×C) → (C×R)
    // -----------------------------------------------------------------

    Mat<COLS, ROWS> T() const {
        Mat<COLS, ROWS> r;
        for (int i = 0; i < ROWS; ++i)
            for (int j = 0; j < COLS; ++j)
                r(j, i) = (*this)(i, j);
        return r;
    }

    // -----------------------------------------------------------------
    // Symmetrize: P = 0.5*(P + P^T), chỉ cho ma trận vuông
    // -----------------------------------------------------------------

    void symmetrize() {
        static_assert(ROWS == COLS, "symmetrize() chỉ cho ma trận vuông");
        for (int i = 0; i < ROWS; ++i) {
            for (int j = i + 1; j < COLS; ++j) {
                double avg = 0.5 * ((*this)(i, j) + (*this)(j, i));
                (*this)(i, j) = avg;
                (*this)(j, i) = avg;
            }
        }
    }

    // -----------------------------------------------------------------
    // Clamp diagonal — giữ P diagonal trong [min, max]
    // -----------------------------------------------------------------

    void clamp_diag(double lo, double hi) {
        static_assert(ROWS == COLS, "clamp_diag() chỉ cho ma trận vuông");
        for (int i = 0; i < ROWS; ++i) {
            if ((*this)(i, i) < lo) (*this)(i, i) = lo;
            if ((*this)(i, i) > hi) (*this)(i, i) = hi;
        }
    }

    /** Lấy phần tử đường chéo thứ i */
    double diag(int i) const { return (*this)(i, i); }

    // -----------------------------------------------------------------
    // Block operations — trích/gán block con
    // -----------------------------------------------------------------

    /**
     * Trích block con kích thước (BR×BC) bắt đầu tại (r0, c0).
     * Ví dụ: P.block<3,3>(0,0) — lấy block 3×3 góc trên trái.
     */
    template <int BR, int BC>
    Mat<BR, BC> block(int r0, int c0) const {
        Mat<BR, BC> b;
        for (int i = 0; i < BR; ++i)
            for (int j = 0; j < BC; ++j)
                b(i, j) = (*this)(r0 + i, c0 + j);
        return b;
    }

    /**
     * Gán block con kích thước (BR×BC) vào vị trí (r0, c0).
     */
    template <int BR, int BC>
    void set_block(int r0, int c0, const Mat<BR, BC>& b) {
        for (int i = 0; i < BR; ++i)
            for (int j = 0; j < BC; ++j)
                (*this)(r0 + i, c0 + j) = b(i, j);
    }

    // -----------------------------------------------------------------
    // Trace — tổng đường chéo (cho ma trận vuông)
    // -----------------------------------------------------------------

    double trace() const {
        static_assert(ROWS == COLS, "trace() chỉ cho ma trận vuông");
        double s = 0.0;
        for (int i = 0; i < ROWS; ++i) s += (*this)(i, i);
        return s;
    }

    // -----------------------------------------------------------------
    // Norm cho vector (Nx1)
    // -----------------------------------------------------------------

    double norm() const {
        double s = 0.0;
        for (int i = 0; i < ROWS * COLS; ++i) s += d[i] * d[i];
        return std::sqrt(s);
    }

    /** Chuẩn hoá vector về đơn vị */
    Mat normalized() const {
        double n = norm();
        if (n < 1e-15) return *this;
        return (*this) * (1.0 / n);
    }
};

// =====================================================================
// Type aliases tiện dụng
// =====================================================================

using Vec3 = Mat<3, 1>;    ///< Vector 3D
using Vec4 = Mat<4, 1>;    ///< Vector 4D (quaternion)
using Mat3 = Mat<3, 3>;    ///< Ma trận 3×3 (rotation, skew, v.v.)

/**
 * Nghịch đảo ma trận 3×3 (closed-form, không heap).
 * Trả false nếu singular (det ≈ 0).
 *
 * Dùng cho: S = H P H^T + R → S_inv để tính Kalman gain.
 */
inline bool invert3(const Mat3& A, Mat3& out) {
    // Cofactor expansion
    double a = A(0,0), b = A(0,1), c = A(0,2);
    double d = A(1,0), e = A(1,1), f = A(1,2);
    double g = A(2,0), h = A(2,1), k = A(2,2);

    double det = a*(e*k - f*h) - b*(d*k - f*g) + c*(d*h - e*g);
    if (std::fabs(det) < 1e-30) return false;

    double inv_det = 1.0 / det;
    out(0,0) = (e*k - f*h) * inv_det;
    out(0,1) = (c*h - b*k) * inv_det;
    out(0,2) = (b*f - c*e) * inv_det;
    out(1,0) = (f*g - d*k) * inv_det;
    out(1,1) = (a*k - c*g) * inv_det;
    out(1,2) = (c*d - a*f) * inv_det;
    out(2,0) = (d*h - e*g) * inv_det;
    out(2,1) = (b*g - a*h) * inv_det;
    out(2,2) = (a*e - b*d) * inv_det;
    return true;
}

/**
 * Nghịch đảo ma trận 6×6 bằng block inversion.
 * Chia A thành 4 block 3×3: [[A11, A12], [A21, A22]]
 * Schur complement: S = A22 - A21 * A11^{-1} * A12
 * Trả false nếu singular.
 */
inline bool invert6(const Mat<6,6>& A, Mat<6,6>& out) {
    Mat3 A11 = A.block<3,3>(0,0);
    Mat3 A12 = A.block<3,3>(0,3);
    Mat3 A21 = A.block<3,3>(3,0);
    Mat3 A22 = A.block<3,3>(3,3);

    Mat3 A11_inv;
    if (!invert3(A11, A11_inv)) return false;

    // Schur complement S = A22 - A21 * A11^{-1} * A12
    Mat3 S = A22 - A21 * A11_inv * A12;
    Mat3 S_inv;
    if (!invert3(S, S_inv)) return false;

    // Block inverse:
    // [[A11_inv + A11_inv*A12*S_inv*A21*A11_inv, -A11_inv*A12*S_inv],
    //  [-S_inv*A21*A11_inv,                        S_inv            ]]
    Mat3 neg_S_inv_A21_A11inv = (S_inv * A21 * A11_inv) * (-1.0);
    Mat3 neg_A11inv_A12_Sinv = (A11_inv * A12 * S_inv) * (-1.0);
    Mat3 top_left = A11_inv - neg_A11inv_A12_Sinv * A21 * A11_inv;

    out.set_block(0, 0, top_left);
    out.set_block(0, 3, neg_A11inv_A12_Sinv);
    out.set_block(3, 0, neg_S_inv_A21_A11inv);
    out.set_block(3, 3, S_inv);
    return true;
}

/**
 * Nghịch đảo scalar 1×1 (trivial).
 */
inline bool invert1(const Mat<1,1>& A, Mat<1,1>& out) {
    if (std::fabs(A(0,0)) < 1e-30) return false;
    out(0,0) = 1.0 / A(0,0);
    return true;
}

}  // namespace eskf

#endif  // ESKF_STATIC_MATRIX_HPP
