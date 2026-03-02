/**
 * @file ring_buffer.hpp
 * @brief Ring buffer template tĩnh (compile-time size) — không heap.
 *
 * Dùng cho:
 * - Lưu IMU samples (1024 entries) để replay khi rollback
 * - Lưu state snapshots (128 entries) để delay compensation
 *
 * Thiết kế embedded-friendly:
 * - Kích thước CAPACITY là template parameter → compile-time
 * - Dữ liệu trên stack (std::array-style, nhưng dùng C array)
 * - O(1) push, O(1) truy cập theo index hoặc tìm gần nhất theo timestamp
 * - Power-of-2 capacity → modulo bằng bitwise AND (nhanh hơn %)
 *
 * Lưu ý: CAPACITY nên là power of 2 (128, 256, 512, 1024…)
 * để tối ưu modulo.
 */

#ifndef ESKF_RING_BUFFER_HPP
#define ESKF_RING_BUFFER_HPP

namespace eskf {

/**
 * Ring buffer tĩnh chứa T, với CAPACITY phần tử.
 *
 * T phải có:
 * - member `double t` (timestamp) để tìm kiếm theo thời gian
 * - method `void clear()` để reset
 *
 * @tparam T     Kiểu phần tử (ImuSample, StateSnapshot, v.v.)
 * @tparam CAP   Số phần tử tối đa (nên là power of 2)
 */
template <typename T, int CAP>
class RingBuffer {
public:
    RingBuffer() : head_(0), count_(0) {}

    /** Reset toàn bộ buffer */
    void reset() {
        head_ = 0;
        count_ = 0;
        for (int i = 0; i < CAP; ++i) {
            buf_[i].clear();
        }
    }

    /** Thêm phần tử mới vào buffer (ghi đè cũ nhất nếu đầy) */
    void push(const T& item) {
        buf_[head_] = item;
        head_ = (head_ + 1) & (CAP - 1);  // head_ = (head_+1) % CAP
        if (count_ < CAP) ++count_;
    }

    /** Số phần tử hiện có */
    int size() const { return count_; }

    /** Buffer có trống không */
    bool empty() const { return count_ == 0; }

    /** Buffer có đầy không */
    bool full() const { return count_ == CAP; }

    /**
     * Truy cập phần tử thứ i (0 = cũ nhất, size()-1 = mới nhất).
     *
     * KHÔNG kiểm tra bounds — caller phải đảm bảo 0 <= i < size().
     */
    const T& at(int i) const {
        // Phần tử cũ nhất nằm ở vị trí (head_ - count_ + i) mod CAP
        int idx = (head_ - count_ + i) & (CAP - 1);
        return buf_[idx];
    }

    T& at(int i) {
        int idx = (head_ - count_ + i) & (CAP - 1);
        return buf_[idx];
    }

    /** Phần tử mới nhất (cuối) */
    const T& newest() const { return at(count_ - 1); }

    /** Phần tử cũ nhất (đầu) */
    const T& oldest() const { return at(0); }

    /**
     * Tìm index của phần tử có timestamp gần nhất với t_target.
     * Dùng linear search (buffer nhỏ → OK cho embedded).
     *
     * Trả -1 nếu buffer trống.
     *
     * @param t_target  Timestamp cần tìm
     * @return Index (0-based từ cũ nhất) hoặc -1
     */
    int find_nearest(double t_target) const {
        if (count_ == 0) return -1;
        int best = 0;
        double best_dt = 1e30;
        for (int i = 0; i < count_; ++i) {
            double dt = at(i).t - t_target;
            if (dt < 0) dt = -dt;  // abs
            if (dt < best_dt) {
                best_dt = dt;
                best = i;
            }
        }
        return best;
    }

    /**
     * Tìm index của snapshot gần nhất nhưng <= t_target.
     * Dùng cho rollback: muốn snapshot TRƯỚC hoặc ĐÚNG thời điểm đo.
     *
     * Trả -1 nếu không tìm thấy (tất cả snapshot đều sau t_target).
     */
    int find_latest_before(double t_target) const {
        if (count_ == 0) return -1;
        int best = -1;
        for (int i = 0; i < count_; ++i) {
            if (at(i).t <= t_target + 1e-9) {
                best = i;
            }
        }
        return best;
    }

    /**
     * Lấy tất cả các phần tử có t > t_start.
     * Dùng để lấy chuỗi IMU sample cần replay từ snapshot → now.
     *
     * @param t_start   Timestamp bắt đầu (exclusive)
     * @param out       Mảng output (caller cung cấp)
     * @param max_out   Kích thước tối đa out
     * @return Số phần tử đã copy vào out
     */
    int get_after(double t_start, T* out, int max_out) const {
        int n = 0;
        for (int i = 0; i < count_ && n < max_out; ++i) {
            if (at(i).t > t_start + 1e-9) {
                out[n++] = at(i);
            }
        }
        return n;
    }

private:
    T buf_[CAP];    ///< Mảng tĩnh trên stack
    int head_;      ///< Con trỏ ghi tiếp theo
    int count_;     ///< Số phần tử hợp lệ
};

}  // namespace eskf

#endif  // ESKF_RING_BUFFER_HPP
