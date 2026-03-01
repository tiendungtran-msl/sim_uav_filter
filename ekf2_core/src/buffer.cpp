/**
 * @file buffer.cpp
 * @brief Ring buffer for delayed-measurement state history.
 *
 * Implementation is fully contained in ekf.cpp (saveBuffer / findBuffered).
 * This file provides documentation and optional test stub.
 *
 * Ring buffer semantics:
 *   - buf_head_ always points to the NEXT write slot.
 *   - buf_count_ tracks filled entries (clamped to BUF_SIZE).
 *   - Entries are indexed from oldest = (buf_head_ - buf_count_ + BUF_SIZE) % BUF_SIZE
 *     to newest = (buf_head_ - 1 + BUF_SIZE) % BUF_SIZE.
 *
 * findBuffered(t_stamp):
 *   Searches all valid entries for the one whose .nom.t is closest to t_stamp.
 *   Accepts the result only if |t_diff| < 400 ms (GPS delay bound).
 *
 * Memory cost:
 *   BUF_SIZE * sizeof(BuffState) = 400 * (sizeof(NominalState) + 15*15*4)
 *   ≈ 400 * (52 + 900) = ~381 kB   — acceptable for a Cortex-M7 with 512 kB RAM.
 *
 * To reduce RAM, lower BUF_SIZE or store only the state (not P) and use
 * a simplified repropagation that does not update the covariance from the
 * buffered snapshot.
 */
#include "ekf.h"

namespace ekf2 {
// saveBuffer() and findBuffered() are defined in ekf.cpp
}
