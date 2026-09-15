// Copperline: shared board copper statistics (D7).
//
// One home for the trace-loop length rescans previously cloned in
// optimizer.cpp (double mm sum), tuning.cpp (integer-nm net sum +
// candidate/base measurement) and diffpair.cpp (integer-nm set sum).
//
// Two measures exist and MUST NOT be conflated (golden JSON is
// byte-identical only when each call site keeps its exact arithmetic):
//   - millimeter doubles (optimizer reports): per-segment sqrt in mm,
//     summed as doubles;
//   - integer nanometres (tuner/pair/verifier): per-segment
//     euclid_len_nm (integer-rounded Euclidean), summed as Coord.
// Header-only so no build re-configure is needed.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "router/board.h"
#include "router/simplify.h"

namespace copperline {

// Double-mm length of one segment (optimizer reporting arithmetic).
inline double trace_seg_length_mm(const TraceSeg& s) {
    double dx = nm_to_mm(s.b.x - s.a.x), dy = nm_to_mm(s.b.y - s.a.y);
    return std::sqrt(dx * dx + dy * dy);
}

// Double-mm total over all committed traces (optimizer length_before/after,
// per-mutation shorter check).
inline double board_total_length_mm(const Board& board) {
    double total = 0;
    for (const auto& s : board.traces) total += trace_seg_length_mm(s);
    return total;
}

// Integer-nm length of one segment (tuner base/candidate arithmetic).
inline Coord trace_seg_length_nm(const TraceSeg& s) {
    return euclid_len_nm(s.a, s.b);
}

// Integer-nm total of a trace set (pair members, candidate chains).
inline Coord trace_set_length_nm(const std::vector<TraceSeg>& traces) {
    Coord total = 0;
    for (const auto& t : traces) total += euclid_len_nm(t.a, t.b);
    return total;
}

// Integer-nm total committed copper of one net (tuner measure, verifier
// pair-length parity).
inline Coord board_net_length_nm(const Board& board, NetId net) {
    Coord total = 0;
    for (const auto& t : board.traces)
        if (t.net == net) total += euclid_len_nm(t.a, t.b);
    return total;
}

inline std::size_t board_via_count(const Board& board) {
    return board.vias.size();
}

// Tight bounding union of two rects (touched-bbox tracking for S3 delta
// pre-checks; callers expand with their own halo).
inline Rect rect_union(const Rect& a, const Rect& b) {
    return {std::min(a.x1, b.x1), std::min(a.y1, b.y1), std::max(a.x2, b.x2),
            std::max(a.y2, b.y2)};
}

}  // namespace copperline
