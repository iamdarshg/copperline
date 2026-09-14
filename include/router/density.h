// Copperline: pin-density analysis.
//
// Density is a first-class routing signal: footprint-local density, endpoint
// density, escape-region density. In phase 1 it feeds difficulty scoring,
// tie-breaking hints and `router analyze` output. Centre-out ordering for
// fine-pitch parts arrives in Prompt 2; density never overrides it.
#pragma once

#include <string>
#include <vector>

#include "router/board.h"

namespace copperline {

struct DenseFootprint {
    Point centroid{};
    Coord radius_nm = 0;
    int pin_count = 0;
    double pins_per_mm2 = 0.0;
    std::string component;  // grouping key when known
    std::vector<TermId> members;
};

struct DensityResult {
    double board_avg_per_mm2 = 0.0;
    // Per-terminal local density, parallel to Board::terminals order.
    std::vector<double> terminal_density;
    std::vector<DenseFootprint> dense_footprints;
    // Max endpoint density per net, parallel to Board::nets order.
    std::vector<double> net_peak_density;
};

class DensityEstimator {
  public:
    explicit DensityEstimator(Coord radius_nm = mm_to_nm(2.0), double dense_factor = 3.0);

    DensityResult analyze(const Board& board) const;
    // Pads per mm^2 within the kernel radius of point p.
    double local_density(const Board& board, Point p) const;

  private:
    Coord radius_nm_;
    double dense_factor_;
};

}  // namespace copperline
