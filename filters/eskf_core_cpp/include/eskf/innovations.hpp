/**
 * @file innovations.hpp
 * @brief Quản lý ghi innovation output cho CSV export.
 *
 * Lưu lịch sử innovation từ GPS/MAG/Baro updates để:
 * - GUI vẽ biểu đồ innovation vs time
 * - Tính NIS (Normalized Innovation Squared) cho consistency check
 * - Xuất file innovations.csv
 *
 * Dùng file-based writer (chỉ cho tools, không cho embedded runtime).
 */

#ifndef ESKF_INNOVATIONS_HPP
#define ESKF_INNOVATIONS_HPP

#include "eskf_types.hpp"
#include <cstdio>

namespace eskf {

/**
 * Writer ghi innovation records ra CSV.
 */
class InnovationWriter {
public:
    InnovationWriter() : fp_(nullptr) {}
    ~InnovationWriter() { close(); }

    bool open(const char* path) {
        fp_ = std::fopen(path, "w");
        if (!fp_) return false;

        // Header
        std::fprintf(fp_,
            "t,sensor,innov_0,innov_1,innov_2,innov_3,innov_4,innov_5,"
            "nis,dim,accepted\n");
        return true;
    }

    void write(const InnovationRecord& rec) {
        if (!fp_) return;
        // sensor_id: 0=GPS, 1=MAG, 2=BARO
        const char* names[] = {"GPS", "MAG", "BARO"};
        const char* sname = (rec.sensor_id >= 0 && rec.sensor_id <= 2)
                            ? names[rec.sensor_id] : "UNK";
        std::fprintf(fp_, "%.6f,%s,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.6f,%d,%d\n",
            rec.t, sname,
            rec.innovation[0], rec.innovation[1], rec.innovation[2],
            rec.innovation[3], rec.innovation[4], rec.innovation[5],
            rec.nis, rec.dim, rec.accepted ? 1 : 0);
    }

    void close() {
        if (fp_) { std::fclose(fp_); fp_ = nullptr; }
    }

private:
    FILE* fp_;
};

}  // namespace eskf

#endif  // ESKF_INNOVATIONS_HPP
