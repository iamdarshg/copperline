#include "router/spatial_index.h"

namespace copperline {

SpatialIndex::SpatialIndex(Coord cell_nm) : cell_nm_(cell_nm > 0 ? cell_nm : mm_to_nm(1.0)) {}

std::int64_t SpatialIndex::key(Coord cx, Coord cy) {
    return (cx << 32) ^ (cy & 0xFFFFFFFFLL);
}

std::vector<int>& SpatialIndex::cell(std::int64_t k) {
    for (auto& [key, vec] : cells_)
        if (key == k) return vec;
    cells_.push_back({k, {}});
    return cells_.back().second;
}

const std::vector<int>* SpatialIndex::find_cell(std::int64_t k) const {
    for (const auto& [key, vec] : cells_)
        if (key == k) return &vec;
    return nullptr;
}

void SpatialIndex::insert(const IndexedRect& item) {
    int id = static_cast<int>(items_.size());
    items_.push_back(item);
    Coord cx0 = item.rect.x1 / cell_nm_;
    Coord cy0 = item.rect.y1 / cell_nm_;
    Coord cx1 = item.rect.x2 / cell_nm_;
    Coord cy1 = item.rect.y2 / cell_nm_;
    // Negative coords: C++ truncation goes toward zero; widen by one to be safe.
    if (item.rect.x1 < 0) --cx0;
    if (item.rect.y1 < 0) --cy0;
    for (Coord cx = cx0; cx <= cx1; ++cx) {
        for (Coord cy = cy0; cy <= cy1; ++cy) {
            cell(key(cx, cy)).push_back(id);
        }
    }
}

std::vector<int> SpatialIndex::query(const Rect& area) const {
    std::vector<int> out;
    std::vector<char> seen(items_.size(), 0);
    Coord cx0 = area.x1 / cell_nm_;
    Coord cy0 = area.y1 / cell_nm_;
    Coord cx1 = area.x2 / cell_nm_;
    Coord cy1 = area.y2 / cell_nm_;
    if (area.x1 < 0) --cx0;
    if (area.y1 < 0) --cy0;
    for (Coord cx = cx0; cx <= cx1; ++cx) {
        for (Coord cy = cy0; cy <= cy1; ++cy) {
            const std::vector<int>* vec = find_cell(key(cx, cy));
            if (!vec) continue;
            for (int id : *vec) {
                if (seen[id]) continue;
                seen[id] = 1;
                if (items_[id].rect.intersects(area)) out.push_back(id);
            }
        }
    }
    return out;
}

}  // namespace copperline
