/**
 * @file replay_main.cpp
 * @brief CSV log replay tool.
 *
 * Reads a CSV file produced by the Python simulator and feeds it through
 * the C++ EKF2 core. This proves the C++ logic is independent of Python.
 *
 * CSV format (written by Python bridge/logger.py):
 *   t, imu_gx, imu_gy, imu_gz, imu_ax, imu_ay, imu_az,
 *   gps_t, gps_px, gps_py, gps_pz, gps_vx, gps_vy, gps_vz, gps_valid,
 *   baro_alt, baro_valid
 *
 * Output columns:
 *   t, est_px, est_py, est_pz, est_vx, est_vy, est_vz,
 *   est_qw, est_qx, est_qy, est_qz, est_bgx, est_bgy, est_bgz,
 *   est_bax, est_bay, est_baz, cov_p0, cov_v0, cov_th0
 */
#include "ekf.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace ekf2;

int main(int argc, char* argv[]) {
    if (argc < 3) {
        fprintf(stderr, "Usage: ekf2_replay <input.csv> <output.csv>\n");
        return 1;
    }

    FILE* fin  = fopen(argv[1], "r");
    FILE* fout = fopen(argv[2], "w");
    if (!fin || !fout) {
        fprintf(stderr, "Cannot open files.\n");
        return 1;
    }

    // Write output header
    fprintf(fout, "t,px,py,pz,vx,vy,vz,qw,qx,qy,qz,bgx,bgy,bgz,bax,bay,baz,"
                  "std_px,std_py,std_pz,std_vx,std_vy,std_vz\n");

    Ekf::Params par;
    par.dt = 0.0025f;
    Ekf ekf(par);

    char line[512];
    // Skip header
    fgets(line, sizeof(line), fin);

    while (fgets(line, sizeof(line), fin)) {
        float t;
        ImuSample  imu{}; imu.valid = true;
        GpsSample  gps{}; gps.valid = false;
        BaroSample baro{}; baro.valid = false;

        // Parse columns
        float gps_t, gps_px, gps_py, gps_pz, gps_vx, gps_vy, gps_vz;
        int gps_valid;
        float baro_alt; int baro_valid_i;

        int n = sscanf(line,
            "%f,"
            "%f,%f,%f,%f,%f,%f,"
            "%f,%f,%f,%f,%f,%f,%f,%d,"
            "%f,%d",
            &t,
            &imu.gyr[0], &imu.gyr[1], &imu.gyr[2],
            &imu.acc[0], &imu.acc[1], &imu.acc[2],
            &gps_t, &gps_px, &gps_py, &gps_pz,
            &gps_vx, &gps_vy, &gps_vz, &gps_valid,
            &baro_alt, &baro_valid_i);

        if (n < 17) continue;

        imu.t = t;

        // IMU update
        ekf.imuUpdate(imu);

        // GPS update if valid
        if (gps_valid) {
            gps.t_stamp   = gps_t;
            gps.t_receive = t;
            gps.pos[0]=gps_px; gps.pos[1]=gps_py; gps.pos[2]=gps_pz;
            gps.vel[0]=gps_vx; gps.vel[1]=gps_vy; gps.vel[2]=gps_vz;
            gps.valid = true;
            ekf.gpsUpdate(gps);
        }

        // Baro update if valid
        if (baro_valid_i) {
            baro.t    = t;
            baro.alt  = baro_alt;
            baro.valid= true;
            ekf.baroUpdate(baro);
        }

        // Write output
        const NominalState& s = ekf.state();
        const Mat15&        P = ekf.covariance();
        fprintf(fout, "%.6f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
                      "%.6f,%.6f,%.6f,%.6f,"
                      "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                      "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
            s.t,
            s.p[0], s.p[1], s.p[2],
            s.v[0], s.v[1], s.v[2],
            s.q.w, s.q.x, s.q.y, s.q.z,
            s.bg[0], s.bg[1], s.bg[2],
            s.ba[0], s.ba[1], s.ba[2],
            sqrtf(P(0,0)), sqrtf(P(1,1)), sqrtf(P(2,2)),
            sqrtf(P(3,3)), sqrtf(P(4,4)), sqrtf(P(5,5)));
    }

    fclose(fin);
    fclose(fout);
    printf("Replay complete -> %s\n", argv[2]);
    return 0;
}
