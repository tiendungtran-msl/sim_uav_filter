/**
 * @file state.h
 * @brief EKF2 (ESKF) state definitions — NuttX/PX4 portable.
 *
 * No dynamic allocation. No STL. No exceptions.
 * Fixed-size matrix arithmetic via inline helpers.
 *
 * Nominal state:
 *   x = [p(3), v(3), q(4), b_g(3), b_a(3)]
 *
 * Error state (15-dim):
 *   dx = [dp(3), dv(3), dth(3), dbg(3), dba(3)]
 *
 * Coordinate frame: NED (North-East-Down)
 * Quaternion convention: q = [w, x, y, z],  q_body_to_ned
 */
#pragma once

#include <stdint.h>
#include <math.h>

namespace ekf2 {

static constexpr int  STATE_N     = 15;   ///< error-state dimension
static constexpr int  BUF_SIZE    = 400;  ///< ~1 s at 400 Hz

// ---------------------------------------------------------------------------
// Minimal fixed-size matrix (row-major, no heap)
// ---------------------------------------------------------------------------
template<int R, int C>
struct Mat {
    float d[R][C];

    Mat() { zero(); }

    void zero() {
        for (int i = 0; i < R; i++)
            for (int j = 0; j < C; j++)
                d[i][j] = 0.f;
    }

    void identity() {
        zero();
        int n = R < C ? R : C;
        for (int i = 0; i < n; i++) d[i][i] = 1.f;
    }

    float& operator()(int r, int c) { return d[r][c]; }
    float  operator()(int r, int c) const { return d[r][c]; }

    Mat<R,C> operator+(const Mat<R,C>& o) const {
        Mat<R,C> res;
        for (int i=0;i<R;i++) for (int j=0;j<C;j++) res.d[i][j]=d[i][j]+o.d[i][j];
        return res;
    }

    Mat<R,C>& operator+=(const Mat<R,C>& o) {
        for (int i=0;i<R;i++) for (int j=0;j<C;j++) d[i][j]+=o.d[i][j];
        return *this;
    }

    Mat<R,C> operator*(float s) const {
        Mat<R,C> res;
        for (int i=0;i<R;i++) for (int j=0;j<C;j++) res.d[i][j]=d[i][j]*s;
        return res;
    }

    template<int K>
    Mat<R,K> operator*(const Mat<C,K>& o) const {
        Mat<R,K> res;
        for (int i=0;i<R;i++)
            for (int k=0;k<K;k++) {
                float s=0.f;
                for (int j=0;j<C;j++) s+=d[i][j]*o.d[j][k];
                res.d[i][k]=s;
            }
        return res;
    }

    Mat<C,R> T() const {
        Mat<C,R> res;
        for (int i=0;i<R;i++) for (int j=0;j<C;j++) res.d[j][i]=d[i][j];
        return res;
    }
};

using Vec3  = Mat<3,1>;
using Vec4  = Mat<4,1>;
using Vec6  = Mat<6,1>;
using Vec15 = Mat<15,1>;
using Mat3  = Mat<3,3>;
using Mat15 = Mat<15,15>;
using Mat6  = Mat<6,6>;

// ---------------------------------------------------------------------------
// Quaternion helpers  q = [w,x,y,z]
// ---------------------------------------------------------------------------
struct Quat {
    float w, x, y, z;

    Quat() : w(1.f), x(0.f), y(0.f), z(0.f) {}
    Quat(float w_, float x_, float y_, float z_)
        : w(w_), x(x_), y(y_), z(z_) {}

    Quat operator*(const Quat& r) const {
        return {
            w*r.w - x*r.x - y*r.y - z*r.z,
            w*r.x + x*r.w + y*r.z - z*r.y,
            w*r.y - x*r.z + y*r.w + z*r.x,
            w*r.z + x*r.y - y*r.x + z*r.w
        };
    }

    Quat conj() const { return {w,-x,-y,-z}; }

    void normalise() {
        float n = sqrtf(w*w+x*x+y*y+z*z);
        if (n > 1e-10f) { w/=n; x/=n; y/=n; z/=n; }
    }

    /// q ⊗ [0,v]  ⊗ q*  — rotate v by quaternion
    void rotate(const float v[3], float out[3]) const {
        // out = R(q)*v
        float tx = 2.f*(y*v[2] - z*v[1]);
        float ty = 2.f*(z*v[0] - x*v[2]);
        float tz = 2.f*(x*v[1] - y*v[0]);
        out[0] = v[0] + w*tx + y*tz - z*ty;
        out[1] = v[1] + w*ty + z*tx - x*tz;
        out[2] = v[2] + w*tz + x*ty - y*tx;
    }

    /// Rotation matrix body → NED
    void toR(Mat3& R) const {
        R(0,0)=1.f-2.f*(y*y+z*z); R(0,1)=2.f*(x*y-w*z); R(0,2)=2.f*(x*z+w*y);
        R(1,0)=2.f*(x*y+w*z); R(1,1)=1.f-2.f*(x*x+z*z); R(1,2)=2.f*(y*z-w*x);
        R(2,0)=2.f*(x*z-w*y); R(2,1)=2.f*(y*z+w*x); R(2,2)=1.f-2.f*(x*x+y*y);
    }
};

/// Small-angle rotation-vector → quaternion
inline Quat rotvec_to_quat(float phi[3]) {
    float angle = sqrtf(phi[0]*phi[0]+phi[1]*phi[1]+phi[2]*phi[2]);
    if (angle < 1e-10f) {
        return {1.f, phi[0]*0.5f, phi[1]*0.5f, phi[2]*0.5f};
    }
    float s = sinf(angle*0.5f)/angle;
    return {cosf(angle*0.5f), phi[0]*s, phi[1]*s, phi[2]*s};
}

// ---------------------------------------------------------------------------
// Nominal state
// ---------------------------------------------------------------------------
struct NominalState {
    float p[3]  = {};  ///< position    NED  (m)
    float v[3]  = {};  ///< velocity    NED  (m/s)
    Quat  q;           ///< attitude    body→NED
    float bg[3] = {};  ///< gyro bias   body (rad/s)
    float ba[3] = {};  ///< accel bias  body (m/s²)
    float t     = 0.f; ///< timestamp   (s)
};

// ---------------------------------------------------------------------------
// Buffered state entry (for GPS delay compensation)
// ---------------------------------------------------------------------------
struct BuffState {
    NominalState nom;
    Mat15        P;
    bool         valid{false};
};

// ---------------------------------------------------------------------------
// IMU measurement
// ---------------------------------------------------------------------------
struct ImuSample {
    float t;
    float gyr[3];   ///< rad/s  (raw, bias not removed)
    float acc[3];   ///< m/s²   (raw, bias not removed)
    bool  valid{true};
};

// ---------------------------------------------------------------------------
// GPS measurement
// ---------------------------------------------------------------------------
struct GpsSample {
    float t_stamp;       ///< time sample was taken
    float t_receive;     ///< time it arrived at filter
    float pos[3];        ///< NED position (m)
    float vel[3];        ///< NED velocity (m/s)
    bool  valid;
};

// ---------------------------------------------------------------------------
// Barometer measurement
// ---------------------------------------------------------------------------
struct BaroSample {
    float t;
    float alt;           ///< metres positive up
    bool  valid;
};

}  // namespace ekf2
