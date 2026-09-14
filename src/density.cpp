#include "router/density.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <numbers>

namespace copperline {

DensityEstimator::DensityEstimator(Coord radius_nm, double dense_factor)
    : radius_nm_(radius_nm), dense_factor_(dense_factor) {}

double DensityEstimator::local_density(const Board& board, Point p) const {
    if (board.terminals.empty()) return 0.0;
    __int128 r2 = (__int128)radius_nm_ * radius_nm_;
    int count = 0;
    for (const auto& t : board.terminals) {
        Coord dx = t.pos.x - p.x;
        Coord dy = t.pos.y - p.y;
        __int128 d2 = (__int128)dx * dx + (__int128)dy * dy;
        if (d2 <= r2) ++count;
    }
    double area_mm2 = std::numbers::pi * nm_to_mm(radius_nm_) * nm_to_mm(radius_nm_);
    if (area_mm2 <= 0) return 0.0;
    return count / area_mm2;
}

DensityResult DensityEstimator::analyze(const Board& board) const {
    DensityResult out;
    out.terminal_density.assign(board.terminals.size(), 0.0);
    for (std::size_t i = 0; i < board.terminals.size(); ++i) {
        out.terminal_density[i] = local_density(board, board.terminals[i].pos);
    }
    double board_area_mm2 = nm_to_mm(board.width_nm) * nm_to_mm(board.height_nm);
    out.board_avg_per_mm2 =
        board_area_mm2 > 0 ? board.terminals.size() / board_area_mm2 : 0.0;

    // Group terminals by component for footprint scoring; terminals without a
    // component name fall back to a proximity cluster key.
    std::map<std::string, std::vector<std::size_t>> groups;
    for (std::size_t i = 0; i < board.terminals.size(); ++i) {
        const auto& t = board.terminals[i];
        if (!t.component.empty()) {
            groups["comp:" + t.component].push_back(i);
        } else {
            // Coarse spatial bucket (4mm) as a grouping key.
            long long bx = static_cast<long long>(t.pos.x / mm_to_nm(4.0));
            long long by = static_cast<long long>(t.pos.y / mm_to_nm(4.0));
            groups["cell:" + std::to_string(bx) + "," + std::to_string(by)].push_back(i);
        }
    }
    double threshold = std::max(out.board_avg_per_mm2 * dense_factor_, 0.5);
    for (const auto& [key, members] : groups) {
        if (members.size() < 4) continue;
        Coord x1 = board.terminals[members[0]].pos.x, y1 = board.terminals[members[0]].pos.y;
        Coord x2 = x1, y2 = y1;
        double peak = 0.0;
        for (std::size_t i : members) {
            const auto& t = board.terminals[i];
            x1 = std::min(x1, t.pos.x);
            y1 = std::min(y1, t.pos.y);
            x2 = std::max(x2, t.pos.x);
            y2 = std::max(y2, t.pos.y);
            peak = std::max(peak, out.terminal_density[i]);
        }
        double w_mm = nm_to_mm(x2 - x1), h_mm = nm_to_mm(y2 - y1);
        double area = std::max(w_mm * h_mm, 0.25);  // floor avoids div-by-zero
        double dens = members.size() / area;
        if (dens < threshold) continue;
        DenseFootprint fp;
        fp.centroid = {(x1 + x2) / 2, (y1 + y2) / 2};
        fp.radius_nm = std::max(x2 - x1, y2 - y1) / 2;
        fp.pin_count = static_cast<int>(members.size());
        fp.pins_per_mm2 = dens;
        if (key.rfind("comp:", 0) == 0) fp.component = key.substr(5);
        for (std::size_t i : members) fp.members.push_back(board.terminals[i].id);
        std::sort(fp.members.begin(), fp.members.end());
        out.dense_footprints.push_back(std::move(fp));
    }
    std::sort(out.dense_footprints.begin(), out.dense_footprints.end(),
              [](const DenseFootprint& a, const DenseFootprint& b) {
                  return a.pins_per_mm2 > b.pins_per_mm2;
              });

    out.net_peak_density.assign(board.nets.size(), 0.0);
    for (std::size_t ni = 0; ni < board.nets.size(); ++ni) {
        double peak = 0.0;
        for (TermId tid : board.nets[ni].terminals) {
            for (std::size_t i = 0; i < board.terminals.size(); ++i) {
                if (board.terminals[i].id == tid) {
                    peak = std::max(peak, out.terminal_density[i]);
                    break;
                }
            }
        }
        out.net_peak_density[ni] = peak;
    }
    return out;
}

}  // namespace copperline
