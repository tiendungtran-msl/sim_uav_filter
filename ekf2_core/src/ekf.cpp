/**
 * @file ekf.cpp
 * @brief EKF2 (ESKF) constructor, error injection, helpers.
 *
 * NuttX-portable. No heap. No STL.
 */
#include "ekf.h"
#include <string.h>

namespace ekf2 {

Ekf::Ekf(const Params& p) : par_(p) {
    // Zero nominal state
    nom_ = NominalState{};
    nom_.q = Quat{};    // identity quaternion

    // Realistic initial covariance matching expected initial errors
    P_.zero();
    // Position: ±5 m
    P_(0,0) = 25.f;  P_(1,1) = 25.f;  P_(2,2) = 25.f;
    // Velocity: ±1 m/s
    P_(3,3) = 1.f;   P_(4,4) = 1.f;   P_(5,5) = 1.f;
    // Attitude: ±10 deg roll/pitch, ±30 deg yaw
    float att_rp = 0.1745f * 0.1745f;   // ~10 deg
    float att_y  = 0.5236f * 0.5236f;   // ~30 deg
    P_(6,6) = att_rp;  P_(7,7) = att_rp;  P_(8,8) = att_y;
    // Gyro bias: ±0.01 rad/s
    float bg2 = 0.01f * 0.01f;
    P_(9,9)  = bg2; P_(10,10) = bg2; P_(11,11) = bg2;
    // Accel bias: ±0.2 m/s²  
    float ba2 = 0.2f * 0.2f;
    P_(12,12) = ba2; P_(13,13) = ba2; P_(14,14) = ba2;

    // Zero acceleration estimate
    a_ned_[0] = 0.f; a_ned_[1] = 0.f; a_ned_[2] = 0.f;

    buildQ();
}

Ekf::Ekf() : Ekf(Params{}) {}

// ---------------------------------------------------------------------------
void Ekf::buildQ() {
    Q_.zero();
    float dt  = par_.dt;

    // Position: tuned process noise (absorbs unmodeled dynamics)
    float qp = par_.q_pos_sigma;
    Q_(0,0) = qp*qp; Q_(1,1) = qp*qp; Q_(2,2) = qp*qp;

    // Velocity: accel white noise integrated over dt, scaled by q_vel_scale
    float qa = par_.sigma_a_white * dt * sqrtf(par_.q_vel_scale);
    Q_(3,3) = qa*qa; Q_(4,4) = qa*qa; Q_(5,5) = qa*qa;

    // Attitude: gyro white noise integrated over dt
    float qg = par_.sigma_g_white * dt;
    Q_(6,6) = qg*qg; Q_(7,7) = qg*qg; Q_(8,8) = qg*qg;

    // Gyro bias random walk
    float qbg = par_.sigma_bg_rw * sqrtf(dt);
    Q_(9,9)   = qbg*qbg; Q_(10,10) = qbg*qbg; Q_(11,11) = qbg*qbg;

    // Accel bias random walk
    float qba = par_.sigma_ba_rw * sqrtf(dt);
    Q_(12,12) = qba*qba; Q_(13,13) = qba*qba; Q_(14,14) = qba*qba;
}

// ---------------------------------------------------------------------------
void Ekf::injectError(const float dx[STATE_N]) {
    nom_.p[0] += dx[0]; nom_.p[1] += dx[1]; nom_.p[2] += dx[2];
    nom_.v[0] += dx[3]; nom_.v[1] += dx[4]; nom_.v[2] += dx[5];

    float phi[3] = {dx[6], dx[7], dx[8]};
    Quat dq = rotvec_to_quat(phi);
    nom_.q = nom_.q * dq;
    nom_.q.normalise();

    nom_.bg[0] += dx[9];  nom_.bg[1] += dx[10]; nom_.bg[2] += dx[11];
    nom_.ba[0] += dx[12]; nom_.ba[1] += dx[13]; nom_.ba[2] += dx[14];
}

// ---------------------------------------------------------------------------
void Ekf::symmetrizeP() {
    for (int i = 0; i < STATE_N; i++)
        for (int j = i+1; j < STATE_N; j++) {
            float avg = 0.5f*(P_(i,j)+P_(j,i));
            P_(i,j) = avg; P_(j,i) = avg;
        }
    // Ensure diagonal stays positive
    for (int i = 0; i < STATE_N; i++) {
        if (P_(i,i) < 1e-8f) P_(i,i) = 1e-8f;
        if (P_(i,i) > 1e6f)  P_(i,i) = 1e6f;
    }
}

// ---------------------------------------------------------------------------
void Ekf::saveBuffer() {
    int idx = buf_head_;
    buf_[idx].nom   = nom_;
    buf_[idx].P     = P_;
    buf_[idx].valid = true;
    buf_head_ = (buf_head_ + 1) % BUF_SIZE;
    if (buf_count_ < BUF_SIZE) buf_count_++;
}

// ---------------------------------------------------------------------------
int Ekf::findBuffered(float t_stamp) const {
    int   best = -1;
    float best_dt = 0.4f;   // accept within 400 ms
    int   start  = (buf_head_ - buf_count_ + BUF_SIZE) % BUF_SIZE;

    for (int k = 0; k < buf_count_; k++) {
        int idx = (start + k) % BUF_SIZE;
        if (!buf_[idx].valid) continue;
        float dt = buf_[idx].nom.t - t_stamp;
        if (dt < 0.f) dt = -dt;
        if (dt < best_dt) {
            best_dt = dt;
            best    = idx;
        }
    }
    return best;
}

}  // namespace ekf2
