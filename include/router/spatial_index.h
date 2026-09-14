// Copperline: uniform-grid spatial index over copper rectangles.
//
// A supporting query structure only — the router's principal representation
// stays vector geometry (segments/rects) plus the sparse routing graph, per
// the architecture spec. Never a fine raster of the board.
#pragma once

#include <cstdint>
#include <vector>

#include "router/geometry.h"

namespace copperline {

enum class CopperKind { kTrace = 0, kPad = 1, kVia = 2, kKeepout = 3 };

struct IndexedRect {
    Rect rect{};
    NetId net = -1;  // -1 for keepouts (foreign to every net)
    LayerId layer = kAllLayers;
    CopperKind kind = CopperKind::kTrace;
    int index = -1;  // index into the source array
};

class SpatialIndex {
  public:
    explicit SpatialIndex(Coord cell_nm = mm_to_nm(1.0));

    void insert(const IndexedRect& item);
    // All items whose rect intersects the query (touching counts).
    std::vector<int> query(const Rect& area) const;
    const IndexedRect& item(int id) const { return items_[id]; }
    std::size_t size() const { return items_.size(); }

  private:
    Coord cell_nm_;
    std::vector<IndexedRect> items_;
    // cell key -> item ids. Key packs two 32-bit cell coords; cell coords for
    // realistic boards (< 2m) fit comfortably.
    std::vector<std::pair<std::int64_t, std::vector<int>>> cells_;

    static std::int64_t key(Coord cx, Coord cy);
    std::vector<int>& cell(std::int64_t k);
    const std::vector<int>* find_cell(std::int64_t k) const;
};

}  // namespace copperline
