/**
 * @file ekf.h
 * @brief EKF2 (ESKF) main class declaration.
 *
 * NuttX-portable: no heap, no STL, no exceptions.
 */
#pragma once
#include "state.h"

namespace ekf2 {

class Ekf {
public:
    // ── Noise configuration ───────────────────────────────────────────────
    struct Params {
        float sigma_g_white   = 0.005f;   ///< rad/s/√Hz
        float sigma_a_white   = 0.1f;     ///< m/s²/√Hz
        float sigma_bg_rw     = 0.0002f;  ///< rad/s²/√Hz bias RW
        float sigma_ba_rw     = 0.005f;   ///< m/s³/√Hz bias RW
        float sigma_gps_pos   = 1.5f;     ///< m per-axis
        float sigma_gps_vel   = 0.15f;    ///< m/s per-axis (base, before adaptive)
        float sigma_baro      = 0.5f;     ///< m (sensor σ)
        float sigma_baro_bias = 0.3f;     ///< m (unmodeled baro bias σ)
        float dt              = 0.0025f;  ///< IMU interval (s)
        float gravity         = 9.80665f; ///< m/s²
        float gps_delay       = 0.2f;     ///< GPS delay (s) for adaptive R
        float q_pos_sigma     = 0.08f;    ///< position process noise per step (m)
        float q_vel_scale     = 6.0f;     ///< velocity Q multiplier (applied to σ_a·dt)
    };

    explicit Ekf(const Params& p);
    Ekf();  // uses default Params

    /// Feed one IMU sample → predict step
    void imuUpdate(const ImuSample& imu);

    /// Feed one GPS measurement (already delayed; t_stamp < current t)
    void gpsUpdate(const GpsSample& gps);

    /// Feed one barometer measurement
    void baroUpdate(const BaroSample& baro);

    /// Read current nominal state
    const NominalState& state() const { return nom_; }

    /// Read current error-state covariance
    const Mat15& covariance() const { return P_; }

private:
    Params  par_;
    NominalState nom_;
    Mat15   P_;

    // Last estimated NED acceleration (stored by imuUpdate for adaptive GPS R)
    float a_ned_[3] = {};

    // Ring buffer
    BuffState buf_[BUF_SIZE];
    int       buf_head_{0};
    int       buf_count_{0};

    // Process noise (Q) — built once in constructor
    Mat15 Q_;

    void   buildQ();
    void   propagateCovariance(const Mat15& F);
    void   injectError(const float dx[STATE_N]);
    void   symmetrizeP();
    int    findBuffered(float t_stamp) const;   ///< returns buf index or -1
    void   saveBuffer();
};

}  // namespace ekf2
