# tests/ — Unit Tests cho ESKF Core

## Chạy tests

```bash
cd filters/eskf_core_cpp/build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

./test_math         # 19 tests — quaternion, rotation, skew, matrix ops
./test_buffer       #  8 tests — RingBuffer push/search/overflow
./test_consistency  #  6 tests — ESKF P symmetry, positive, bounded, update effect
```

## Danh sách test

### test_math (19 tests)

| Test | Kiểm tra |
|------|----------|
| test_mat_identity | Identity matrix |
| test_mat_multiply | I * v = v |
| test_mat_transpose | Transpose 2×3 |
| test_mat_invert3 | 3×3 inverse: A * A^{-1} = I |
| test_mat_invert6 | 6×6 block inverse |
| test_quat_identity | quat_rotate(identity, v) = v |
| test_quat_90_yaw | 90° yaw: body X → NED East |
| test_quat_mult_inverse | q ⊗ q* = identity |
| test_quat_normalize | Normalize non-unit quaternion |
| test_quat_from_rotvec_small | Taylor approximation for tiny angle |
| test_rot_from_quat_identity | R(identity_q) = I |
| test_rot_orthogonal | R * R^T = I |
| test_rot_consistent_with_quat_rotate | R*v == quat_rotate(q,v) |
| test_euler_roundtrip | Euler angles in valid range |
| test_euler_identity | Identity q → (0,0,0) Euler |
| test_skew_cross | [a]× * b = a × b |
| test_skew_antisymmetric | S + S^T = 0 |
| test_wrap_pi | Angle wrapping to [-π, π] |
| test_dot | Dot product |

### test_buffer (8 tests)

| Test | Kiểm tra |
|------|----------|
| test_push_and_size | Push + size count |
| test_at | Index-based access (oldest → newest) |
| test_find_nearest | Timestamp nearest-search |
| test_find_latest_before | Latest sample with t ≤ target |
| test_get_after | Get all samples after timestamp |
| test_overflow | Circular overwrite when full |
| test_reset | Reset clears buffer |
| test_snapshot_buffer | StateSnapshot in ring buffer |

### test_consistency (6 tests)

| Test | Kiểm tra |
|------|----------|
| test_p_symmetry | P(i,j) = P(j,i) after 200 predict steps |
| test_p_positive_diagonal | All P(i,i) > 0 |
| test_quaternion_unit | ‖q‖ = 1 after 1000 steps with rotation |
| test_p_bounded | P diagonal ≤ P_MAX after 4000 predict-only steps |
| test_mag_reduces_p_att | MAG update decreases trace(P_att) |
| test_baro_reduces_p_alt | Baro update decreases P(2,2) |

## Kết quả gần nhất

```
test_math:        19 PASS, 0 FAIL
test_buffer:       8 PASS, 0 FAIL
test_consistency:  6 PASS, 0 FAIL
────────────────────────────────
TOTAL:            33 PASS, 0 FAIL
```
