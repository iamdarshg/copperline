#include "router/parallel.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <map>
#include <set>
#include <thread>

#include "router/diffpair.h"
#include "router/via_bundle.h"
#include "router/simplify.h"

namespace copperline {

namespace {

// D2: rect_gap2/gap_ok_rect/seg_ok_rect live in geometry.h (single definition).
const Terminal* find_term(const Board& b, TermId id) { return b.find_terminal(id); }

// Issue #12: the coupled sibling net of a pair member (-1 when the net is
// not in a pair). Corridor legality exempts the sibling's pads (they sit
// inside the reserved envelope by construction; the materializer enforces
// the exact pair gap instead of the voltage table there).
NetId pair_partner_of(const Board& b, NetId net) {
    for (const auto& pr : b.diffpairs) {
        if (pr.net_p == net) return pr.net_n;
        if (pr.net_n == net) return pr.net_p;
    }
    return -1;
}

// Issue #11: per-task A* layer-cost bias. Eligible impedance layers are
// discounted (selected most), ineligible layers cost double, so the search
// prefers the solved layer while staying free to leave it when the corridor
// demands. Identity copy when the net has no impedance target.
std::vector<double> effective_layer_mult(const Board& snapshot, const RuleResolver& resolver,
                                         const ConnectionTask& task,
                                         const std::vector<double>& base) {
    const NetInfo* n = snapshot.find_net(task.net);
    if (!n || !resolver.impedance().has_target(*n)) return base;
    std::vector<double> mult = base;
    for (const auto& l : snapshot.layers) {
        double b = resolver.impedanceLayerMultiplier(task.net, l.id);
        if (l.id >= 0 && l.id < static_cast<int>(mult.size())) mult[l.id] *= b;
    }
    return mult;
}

Coord max_clear_for(const Board& board, const RuleResolver& resolver, NetId net,
                    const ElectricalContext& ctx) {
    Coord m = 0;
    std::string cs;
    for (const auto& o : board.nets) {
        if (o.id == net) continue;
        m = std::max(m, resolver.requiredClearance(net, o.id, 0, ctx, &cs));
    }
    return m;
}

double term_density_at(const Board& board, TermId tid, const std::vector<double>& dens) {
    for (std::size_t i = 0; i < board.terminals.size(); ++i)
        if (board.terminals[i].id == tid && i < dens.size()) return dens[i];
    return 0.0;
}

}  // namespace

// ---- Issue #8: materialized cost from final copper ----

// 8-way segment direction (8 = degenerate point).
static int seg_dir8(const TraceSeg& s) {
    Coord dx = s.b.x - s.a.x;
    Coord dy = s.b.y - s.a.y;
    if (dx == 0 && dy == 0) return 8;
    int sx = (dx > 0) ? 1 : ((dx < 0) ? -1 : 0);
    int sy = (dy > 0) ? 1 : ((dy < 0) ? -1 : 0);
    if (sx > 0 && sy == 0) return 0;   // E
    if (sx > 0 && sy > 0) return 1;    // NE
    if (sx == 0 && sy > 0) return 2;   // N
    if (sx < 0 && sy > 0) return 3;    // NW
    if (sx < 0 && sy == 0) return 4;   // W
    if (sx < 0 && sy < 0) return 5;    // SW
    if (sx == 0 && sy < 0) return 6;   // S
    return 7;                          // SE
}

Coord materialized_route_cost(const std::vector<TraceSeg>& traces,
                              const std::vector<Via>& vias,
                              const std::vector<double>& layer_mult,
                              const AStarConfig& cfg) {
    Coord total = 0;
    auto mult_for = [&](LayerId l) -> double {
        if (l >= 0 && l < static_cast<int>(layer_mult.size()) && layer_mult[l] > 0)
            return layer_mult[l];
        return 1.0;
    };
    for (const auto& t : traces) {
        Coord len = euclid_len_nm(t.a, t.b);
        total += static_cast<Coord>(std::llround(static_cast<double>(len) * mult_for(t.layer)));
    }
    if (!vias.empty()) {
        if (cfg.via_cost_nm > 0) {
            // Saturating add for via term.
            __int128 add = (__int128)vias.size() * cfg.via_cost_nm;
            __int128 sum = (__int128)total + add;
            total = sum > std::numeric_limits<Coord>::max()
                        ? std::numeric_limits<Coord>::max()
                        : static_cast<Coord>(sum);
        }
    }
    // Bends: direction change between consecutively chained same-layer runs.
    // Count a bend when trace[i] continues trace[i-1] on the same layer
    // (a == prev.b) with a different 8-way direction.
    int bends = 0;
    int prev_dir = 8;
    bool have_prev = false;
    const TraceSeg* prev = nullptr;
    for (const auto& t : traces) {
        int d = seg_dir8(t);
        if (d == 8) continue;
        if (have_prev && prev != nullptr && t.layer == prev->layer && t.a == prev->b &&
            d != prev_dir) {
            ++bends;
        }
        prev = &t;
        prev_dir = d;
        have_prev = true;
    }
    if (bends > 0 && cfg.bend_cost_nm > 0) {
        __int128 add = (__int128)bends * cfg.bend_cost_nm;
        __int128 sum = (__int128)total + add;
        total = sum > std::numeric_limits<Coord>::max()
                    ? std::numeric_limits<Coord>::max()
                    : static_cast<Coord>(sum);
    }
    return total;
}

// ---- Difficulty ----

DifficultyVector compute_difficulty(const Board& board, const RuleResolver& resolver,
                                    const ConnectionTask& task, const ElectricalContext& ctx,
                                    const std::vector<double>& terminal_density,
                                    const std::map<TermId, int>& centre_depth, int prev_failures) {
    DifficultyVector d;
    const Terminal* ta = find_term(board, task.a);
    const Terminal* tb = find_term(board, task.b);
    const NetInfo* net = board.find_net(task.net);
    if (!ta || !tb) return d;
    // Issue #12: pair corridors size by the occupied envelope (both traces
    // + gap + external clearance) and span the pad midpoints.
    Coord pair_occ = 0;
    const DiffPair* pair = nullptr;
    if (task.is_pair_corridor) {
        for (const auto& pr : board.diffpairs) {
            if (pr.id == task.pair_id) {
                pair = &pr;
                break;
            }
        }
        if (pair) pair_occ = diffpair_occupied_width(board, resolver, *pair, ctx);
    }
    // Issue #4: span is to the actual routing target (copper point when
    // present), not the representative terminal. All other burdens stay
    // terminal-anchored for stability.
    const Point dst_p = task_dst_point(board, task);
    const LayerId dst_l = task_dst_layer(board, task);

    d.span_mm = nm_to_mm(manhattan(ta->pos, dst_p));
    std::string wsource;
    Coord width = resolver.requiredTraceWidth(task.net, ta->layer, ctx, &wsource);
    // Issue #11: impedance-controlled nets size difficulty by the
    // conservative (max across layers) reconciled width.
    if (const NetInfo* dini = board.find_net(task.net);
        dini && resolver.impedance().has_target(*dini))
        width = resolver.maxRequiredWidth(task.net, ctx);
    // Issue #12: the corridor reserves both traces + gap + external
    // clearance as one atomic resource.
    if (pair) width = pair_occ;
    d.width_mm = nm_to_mm(width);
    d.clearance_mm = nm_to_mm(max_clear_for(board, resolver, task.net, ctx));
    d.endpoint_density =
        std::max(term_density_at(board, task.a, terminal_density),
                 term_density_at(board, task.b, terminal_density));
    if (task.is_pair_corridor) {
        // Issue #12: both members' endpoints burden the corridor.
        d.endpoint_density = std::max(
            d.endpoint_density,
            std::max(term_density_at(board, task.pair_a_other, terminal_density),
                     term_density_at(board, task.pair_b_other, terminal_density)));
    }

    // Free routing space inside the probable corridor.
    Rect corr = Rect::from_points(ta->pos, dst_p).expanded(width / 2 + mm_to_nm(0.5));
    double corr_area = static_cast<double>(corr.width()) * static_cast<double>(corr.height());
    double blocked = 0.0;
    if (corr_area > 0) {
        for (const auto& ko : board.keepouts) {
            if (!ko.rect.intersects(corr)) continue;
            Rect inter{std::max(ko.rect.x1, corr.x1), std::max(ko.rect.y1, corr.y1),
                       std::min(ko.rect.x2, corr.x2), std::min(ko.rect.y2, corr.y2)};
            blocked += static_cast<double>(inter.width()) * static_cast<double>(inter.height());
        }
        for (const auto& t : board.terminals) {
            if (t.net == task.net) continue;
            Rect pr = t.pad_rect();
            if (!pr.intersects(corr)) continue;
            Rect inter{std::max(pr.x1, corr.x1), std::max(pr.y1, corr.y1),
                       std::min(pr.x2, corr.x2), std::min(pr.y2, corr.y2)};
            blocked += static_cast<double>(inter.width()) * static_cast<double>(inter.height());
        }
    }
    double cover = corr_area > 0 ? std::min(1.0, blocked / corr_area) : 0.0;
    d.free_space = 1.0 - cover;
    d.corridor_count = 2.0 * std::max<std::size_t>(1, board.layers.size());
    d.corridor_scarcity = cover + 1.0 / d.corridor_count;

    // Layer restrictions: costly layers shrink options.
    int costly = 0;
    for (const auto& l : board.layers)
        if (l.cost_multiplier > 1.25) ++costly;
    d.layer_restriction =
        board.layers.empty() ? 0 : 0.5 * static_cast<double>(costly) / board.layers.size();

    // Via restrictions: multi-layer span needs a via; over-current styles
    // force parallel vias; a missing preferred class hurts.
    // Issue #4: span is source layer -> actual dst layer (copper contact).
    d.via_restriction = (ta->layer == dst_l) ? 0.0 : 0.5;
    {
        ViaStyle style;
        LayerSpan full{board.layers.front().id, board.layers.back().id};
        if (!resolver.select_via(task.net, full, style)) {
            d.via_restriction += 2.0;
        } else {
            ElectricalContext c2 = ctx;
            int n = resolver.current().vias_required(style, *board.find_net(task.net),
                                                     board.defaults, c2);
            if (n > 1) d.via_restriction += 1.5 * (n - 1);
        }
    }

    d.prev_failures = static_cast<double>(std::max(0, prev_failures));
    {
        auto it = centre_depth.find(task.a);
        int da = it != centre_depth.end() ? it->second : 0;
        it = centre_depth.find(task.b);
        int db = it != centre_depth.end() ? it->second : 0;
        int dc = 0, dd = 0;
        if (task.is_pair_corridor) {
            // Issue #12: the deeper member sets the corridor depth.
            it = centre_depth.find(task.pair_a_other);
            if (it != centre_depth.end()) dc = it->second;
            it = centre_depth.find(task.pair_b_other);
            if (it != centre_depth.end()) dd = it->second;
        }
        d.fine_pitch_depth = static_cast<double>(std::max(std::max(da, db),
                                                           std::max(dc, dd)));
    }

    d.total = d.span_mm + 40.0 * d.width_mm + 30.0 * d.clearance_mm + 0.5 * d.endpoint_density +
              4.0 * (1.0 - d.free_space) + 2.0 * d.corridor_scarcity + d.layer_restriction +
              d.via_restriction + 3.0 * d.prev_failures + 5.0 * d.fine_pitch_depth;
    if (wsource == "ipc_estimate" || wsource == "ampacity") d.total += 0.5;
    if (net && net->terminals.size() > 2) d.total += 0.25 * net->terminals.size();
    // Issue #12: coupled routing is inherently harder than either member
    // alone (two traces must stay parallel inside one envelope).
    if (pair) d.total += 1.0;
    return d;
}

// ---- Corridors + interference ----

Corridor probable_corridor(const Board& board, const RuleResolver& resolver,
                           const ConnectionTask& task, const ElectricalContext& ctx) {
    Corridor c;
    const Terminal* ta = find_term(board, task.a);
    const Terminal* tb = find_term(board, task.b);
    if (!ta || !tb) return c;
    std::string ws;
    c.width_nm = resolver.requiredTraceWidth(task.net, ta->layer, ctx, &ws);
    // Issue #11: corridor resource width is the conservative reconciled
    // width for impedance-controlled nets.
    if (const NetInfo* cni = board.find_net(task.net);
        cni && resolver.impedance().has_target(*cni))
        c.width_nm = resolver.maxRequiredWidth(task.net, ctx);
    // Issue #12: one atomic envelope for both members + gap + external
    // clearance (task_src/dst_point already resolve the pad midpoints).
    if (task.is_pair_corridor) {
        for (const auto& pr : board.diffpairs) {
            if (pr.id == task.pair_id) {
                c.width_nm = diffpair_occupied_width(board, resolver, pr, ctx);
                break;
            }
        }
    }
    c.clear_nm = max_clear_for(board, resolver, task.net, ctx);
    c.rect = Rect::from_points(task_src_point(board, task), task_dst_point(board, task))
                 .expanded(c.width_nm / 2 + c.clear_nm);
    // Issue #3: electrical scarcity for interference weighting (mirrors the
    // difficulty layer/via burdens, but stored on the corridor so the weight
    // reflects resource demand, not pure bbox overlap).
    {
        int costly = 0;
        for (const auto& l : board.layers)
            if (l.cost_multiplier > 1.25) ++costly;
        c.layer_scarcity =
            board.layers.empty() ? 0 : 0.5 * static_cast<double>(costly) / board.layers.size();
        const Terminal* tdst = find_term(board, task.b);
        LayerId dst_l = tdst ? tdst->layer : ta->layer;
        c.via_scarcity = (ta->layer == dst_l) ? 0.0 : 0.5;
        LayerSpan full{board.layers.front().id, board.layers.back().id};
        ViaStyle style;
        if (!resolver.select_via(task.net, full, style)) {
            c.via_scarcity += 2.0;
        } else if (const NetInfo* vn = board.find_net(task.net)) {
            ElectricalContext c2 = ctx;
            int n = resolver.current().vias_required(style, *vn, board.defaults, c2);
            if (n > 1) c.via_scarcity += 1.5 * (n - 1);
        }
        c.electrical_weight = 1.0 + c.layer_scarcity + c.via_scarcity;
    }
    return c;
}

double interference_weight(const Corridor& a, const Corridor& b) {
    if (!a.rect.intersects(b.rect)) return 0.0;
    Rect inter{std::max(a.rect.x1, b.rect.x1), std::max(a.rect.y1, b.rect.y1),
               std::min(a.rect.x2, b.rect.x2), std::min(a.rect.y2, b.rect.y2)};
    double overlap_mm2 = nm_to_mm(inter.width()) * nm_to_mm(inter.height());
    if (overlap_mm2 <= 0) return 0.0;
    // Wider traces + larger clearances consume more shared resource.
    double wi = nm_to_mm(a.width_nm), wj = nm_to_mm(b.width_nm);
    double ci = nm_to_mm(a.clear_nm), cj = nm_to_mm(b.clear_nm);
    double base = overlap_mm2 * (1.0 + 2.0 * (ci + cj) + (wi + wj));
    // Issue #3: layer/via scarcity scales the shared-resource demand.
    double elec = 0.5 * (a.electrical_weight + b.electrical_weight);
    if (!(elec > 0)) elec = 1.0;
    return base * elec;
}

std::vector<std::vector<double>> build_interference(const std::vector<ConnectionTask>& tasks,
                                                    const std::vector<Corridor>& corridors) {
    std::size_t n = tasks.size();
    std::vector<std::vector<double>> w(n, std::vector<double>(n, 0.0));
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            double v = interference_weight(corridors[i], corridors[j]);
            w[i][j] = w[j][i] = v;
        }
    }
    return w;
}

int memory_bounded_batch_width(int requested_width, std::size_t budget_bytes,
                               std::size_t per_task_bytes) {
    if (requested_width < 1) requested_width = 1;
    if (per_task_bytes == 0) per_task_bytes = kPerCandidateBytes;
    if (budget_bytes == 0) return requested_width;
    std::size_t bound = budget_bytes / per_task_bytes;
    if (bound < 1) bound = 1;
    return static_cast<int>(std::min<std::size_t>(requested_width, bound));
}

int resolve_worker_threads(int requested_threads) {
    if (requested_threads > 0) return requested_threads;
    unsigned hw = std::thread::hardware_concurrency();
    if (hw == 0) hw = 4;
    return static_cast<int>(hw);
}

BatchSelection select_interference_batch(const std::vector<int>& ordered_global_idx,
                                         const std::vector<ConnectionTask>& tasks,
                                         const std::vector<Corridor>& corridors,
                                         const BatchSchedOptions& opts) {
    BatchSelection out;
    out.effective_width =
        memory_bounded_batch_width(opts.batch_width, opts.memory_budget_bytes,
                                   opts.per_task_bytes);
    out.threshold_used = opts.interference_threshold;
    if (ordered_global_idx.empty() || out.effective_width < 1) return out;
    double threshold = opts.interference_threshold;
    double relax = opts.relax_factor > 1.0 ? opts.relax_factor : 2.0;
    int max_relax = std::max(0, opts.max_relax_steps);
    std::vector<char> taken(tasks.size(), 0);
    std::set<NetId> picked_nets;
    auto max_vs_selected = [&](int gi) {
        double m = 0.0;
        for (int sj : out.selected) {
            double w = 0.0;
            if (gi < (int)corridors.size() && sj < (int)corridors.size())
                w = interference_weight(corridors[gi], corridors[sj]);
            if (w > m) m = w;
        }
        return m;
    };
    // First pass + relaxed passes stream rows: O(batch * remaining), no dense
    // N^2 allocation; corridor buffers are reused by the caller each epoch.
    // The seed (highest-value eligible) is always selected so a degenerate
    // threshold can never starve the epoch. Relaxation only engages when the
    // first pass yields fewer than 2 tasks: when independent work already
    // fills the batch, competitors stay deferred (separation); when only
    // competitors remain, the threshold relaxes so workers never idle and
    // deferred tasks eventually run (final infinite pass guarantees it).
    bool seeded = false;
    for (int gi : ordered_global_idx) {
        if ((int)out.selected.size() >= out.effective_width) break;
        if (gi < 0 || gi >= (int)tasks.size() || taken[gi]) continue;
        if (picked_nets.count(tasks[gi].net)) continue;  // one task per net
        if (!seeded) {
            out.selected.push_back(gi);
            taken[gi] = 1;
            picked_nets.insert(tasks[gi].net);
            seeded = true;
            continue;
        }
        if (max_vs_selected(gi) < threshold) {
            out.selected.push_back(gi);
            taken[gi] = 1;
            picked_nets.insert(tasks[gi].net);
        }
    }
    if ((int)out.selected.size() >= out.effective_width || out.selected.size() >= 2) {
        out.threshold_used = threshold;
        out.relax_steps = 0;
    } else {
        for (int pass = 0;; ++pass) {
            if ((int)out.selected.size() >= out.effective_width) break;
            if (pass >= max_relax) {
                if (pass > max_relax) break;
                // Final infinite-threshold sweep: take remaining eligible in
                // order so workers never idle.
                for (int gi : ordered_global_idx) {
                    if ((int)out.selected.size() >= out.effective_width) break;
                    if (gi < 0 || gi >= (int)tasks.size() || taken[gi]) continue;
                    if (picked_nets.count(tasks[gi].net)) continue;
                    out.selected.push_back(gi);
                    taken[gi] = 1;
                    picked_nets.insert(tasks[gi].net);
                }
                break;
            }
            threshold *= relax;
            out.relax_steps++;
            for (int gi : ordered_global_idx) {
                if ((int)out.selected.size() >= out.effective_width) break;
                if (gi < 0 || gi >= (int)tasks.size() || taken[gi]) continue;
                if (picked_nets.count(tasks[gi].net)) continue;
                if (max_vs_selected(gi) < threshold) {
                    out.selected.push_back(gi);
                    taken[gi] = 1;
                    picked_nets.insert(tasks[gi].net);
                }
            }
            if ((int)out.selected.size() >= 2 &&
                (int)out.selected.size() >= out.effective_width)
                break;
            if ((int)out.selected.size() >= 2) break;
        }
        out.threshold_used = threshold;
    }
    // Bounded pairwise diagnostics among selected only (<= width^2).
    for (std::size_t i = 0; i < out.selected.size(); ++i) {
        for (std::size_t j = i + 1; j < out.selected.size(); ++j) {
            int a = out.selected[i], b = out.selected[j];
            double w = 0.0;
            if (a < (int)corridors.size() && b < (int)corridors.size())
                w = interference_weight(corridors[a], corridors[b]);
            if (out.pair_scores.size() < kMaxStoredInterferencePairs) {
                out.pair_scores.push_back({{a, b}, w});
            } else {
                out.pairs_capped++;
            }
            if (w > out.max_interference) out.max_interference = w;
        }
    }
    out.pairs_stored = out.pair_scores.size();
    if (!out.pair_scores.empty()) {
        double sum = 0;
        for (auto& p : out.pair_scores) sum += p.second;
        out.mean_interference = sum / out.pair_scores.size();
    }
    return out;
}

std::vector<std::vector<int>> schedule_batches(const std::vector<ConnectionTask>& tasks,
                                                int batch_size) {
    if (batch_size < 1) batch_size = 1;
    std::vector<int> order(tasks.size());
    for (std::size_t i = 0; i < tasks.size(); ++i) order[i] = static_cast<int>(i);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        if (tasks[a].difficulty != tasks[b].difficulty)
            return tasks[a].difficulty > tasks[b].difficulty;
        if (tasks[a].net != tasks[b].net) return tasks[a].net < tasks[b].net;
        if (tasks[a].a != tasks[b].a) return tasks[a].a < tasks[b].a;
        if (tasks[a].b != tasks[b].b) return tasks[a].b < tasks[b].b;
        // Issue #12: deterministic pair-corridor tie-break.
        if (tasks[a].is_pair_corridor != tasks[b].is_pair_corridor)
            return tasks[a].is_pair_corridor < tasks[b].is_pair_corridor;
        return tasks[a].pair_id < tasks[b].pair_id;
    });
    std::vector<std::vector<int>> batches;
    for (std::size_t i = 0; i < order.size(); i += batch_size) {
        batches.push_back({});
        for (std::size_t j = i; j < order.size() && j < i + (std::size_t)batch_size; ++j)
            batches.back().push_back(order[j]);
    }
    return batches;
}

// ---- Congestion ----

void CongestionMap::init(const Board& board, int cells_per_side) {
    n_ = std::max(4, cells_per_side);
    bounds_ = board.bounds();
    Coord w = std::max<Coord>(1, bounds_.width());
    Coord h = std::max<Coord>(1, bounds_.height());
    cell_w_ = std::max<Coord>(1, (w + n_ - 1) / n_);
    cell_h_ = std::max<Coord>(1, (h + n_ - 1) / n_);
    present_.assign(n_ * n_, 0.0);
    history_.assign(n_ * n_, 0.0);
}

void CongestionMap::reset_present() { std::fill(present_.begin(), present_.end(), 0.0); }

int CongestionMap::cell_of_x(Coord x) const {
    int c = static_cast<int>((x - bounds_.x1) / cell_w_);
    return std::clamp(c, 0, n_ - 1);
}
int CongestionMap::cell_of_y(Coord y) const {
    int c = static_cast<int>((y - bounds_.y1) / cell_h_);
    return std::clamp(c, 0, n_ - 1);
}

void CongestionMap::add_present_corridor(const Rect& rect, double weight) {
    int x0 = cell_of_x(rect.x1), x1 = cell_of_x(rect.x2);
    int y0 = cell_of_y(rect.y1), y1 = cell_of_y(rect.y2);
    for (int y = y0; y <= y1; ++y)
        for (int x = x0; x <= x1; ++x) present_[y * n_ + x] += weight;
}

void CongestionMap::add_history_rect(const Rect& rect, double amount) {
    int x0 = cell_of_x(rect.x1), x1 = cell_of_x(rect.x2);
    int y0 = cell_of_y(rect.y1), y1 = cell_of_y(rect.y2);
    for (int y = y0; y <= y1; ++y)
        for (int x = x0; x <= x1; ++x) history_[y * n_ + x] += amount;
}

void CongestionMap::add_history_segment(const Segment& seg, double amount) {
    // Issue #12: traverse only cells intersected by the segment capsule,
    // not the entire bbox (a diagonal's bbox covers mostly empty cells).
    if (history_.empty() || n_ <= 0) return;
    Rect r = seg.bounds();
    int x0 = cell_of_x(r.x1), x1 = cell_of_x(r.x2);
    int y0 = cell_of_y(r.y1), y1 = cell_of_y(r.y2);
    for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
            Rect cell{bounds_.x1 + (Coord)x * cell_w_, bounds_.y1 + (Coord)y * cell_h_,
                      bounds_.x1 + (Coord)(x + 1) * cell_w_,
                      bounds_.y1 + (Coord)(y + 1) * cell_h_};
            if (!seg_intersects_rect(seg, cell)) continue;
            history_[y * n_ + x] += amount;
        }
    }
}

Coord CongestionMap::penalty_for_segment(const Segment& seg) const {
    // Issue #12: same capsule traversal as add_history_segment: only cells
    // intersected by the centerline segment contribute, not the whole bbox.
    if (present_.empty() || history_.empty() || n_ <= 0) return 0;
    Rect r = seg.bounds();
    int x0 = cell_of_x(r.x1), x1 = cell_of_x(r.x2);
    int y0 = cell_of_y(r.y1), y1 = cell_of_y(r.y2);
    // Perf (exact): clip each row to the x-range the segment can reach in that
    // row's y-band (plus one cell of slack), instead of sweeping the whole
    // bbox. The seg_intersects_rect gate below is unchanged, so the accumulated
    // cost -- and every routing decision -- is bit-identical; only the wasted
    // bbox-corner cells are skipped (quadratic in edge length for long diagonal
    // edges, which dominate the graph).
    const bool horizontal = (seg.a.y == seg.b.y);
    const long double ax = seg.a.x, ay = seg.a.y, bx = seg.b.x, by = seg.b.y;
    const long double dx = bx - ax, dy = by - ay;
    double cost = 0.0;
    for (int y = y0; y <= y1; ++y) {
        int rx0 = x0, rx1 = x1;
        if (!horizontal) {
            const long double band0 = bounds_.y1 + (Coord)y * cell_h_;
            const long double band1 = band0 + cell_h_;
            long double ta = (band0 - ay) / dy, tb = (band1 - ay) / dy;
            if (ta > tb) std::swap(ta, tb);
            if (ta < 0.0L) ta = 0.0L;
            if (tb > 1.0L) tb = 1.0L;
            if (ta <= tb) {
                long double xa = ax + ta * dx, xb = ax + tb * dx;
                if (xa > xb) std::swap(xa, xb);
                rx0 = std::max(x0, cell_of_x(static_cast<Coord>(xa)) - 1);
                rx1 = std::min(x1, cell_of_x(static_cast<Coord>(xb)) + 1);
            }
        }
        for (int x = rx0; x <= rx1; ++x) {
            Rect cell{bounds_.x1 + (Coord)x * cell_w_, bounds_.y1 + (Coord)y * cell_h_,
                      bounds_.x1 + (Coord)(x + 1) * cell_w_,
                      bounds_.y1 + (Coord)(y + 1) * cell_h_};
            if (!seg_intersects_rect(seg, cell)) continue;
            cost += present_[y * n_ + x] + 3.0 * history_[y * n_ + x];
        }
    }
    // 0.025mm per cost unit, capped: bounded distortion, never a hard lock.
    Coord p = static_cast<Coord>(cost * 25000.0);
    return std::min<Coord>(p, 1500000);
}

double CongestionMap::present_at(Point p) const {
    return present_[cell_of_y(p.y) * n_ + cell_of_x(p.x)];
}
double CongestionMap::history_at(Point p) const {
    return history_[cell_of_y(p.y) * n_ + cell_of_x(p.x)];
}

std::vector<Hotspot> CongestionMap::hotspots(int top_k) const {
    struct Cell {
        double score;
        int idx;
    };
    std::vector<Cell> cells;
    cells.reserve(n_ * n_);
    for (int i = 0; i < n_ * n_; ++i) {
        double s = present_[i] + 3.0 * history_[i];
        if (s > 0) cells.push_back({s, i});
    }
    std::sort(cells.begin(), cells.end(),
              [](const Cell& a, const Cell& b) { return a.score > b.score; });
    std::vector<Hotspot> out;
    for (int i = 0; i < (int)cells.size() && i < top_k; ++i) {
        int cx = cells[i].idx % n_, cy = cells[i].idx / n_;
        Hotspot h;
        h.x_mm = nm_to_mm(bounds_.x1 + cx * cell_w_ + cell_w_ / 2);
        h.y_mm = nm_to_mm(bounds_.y1 + cy * cell_h_ + cell_h_ / 2);
        h.present = present_[cells[i].idx];
        h.history = history_[cells[i].idx];
        out.push_back(h);
    }
    return out;
}

// ---- Reservations ----

int ReservationSet::cell_of_x(Coord x) const {
    if (nx_ <= 0) return 0;
    long long i = static_cast<long long>(x - bounds_.x1) / (cell_w_ > 0 ? cell_w_ : 1);
    if (i < 0) i = 0;
    if (i >= nx_) i = nx_ - 1;
    return static_cast<int>(i);
}
int ReservationSet::cell_of_y(Coord y) const {
    if (ny_ <= 0) return 0;
    long long i = static_cast<long long>(y - bounds_.y1) / (cell_h_ > 0 ? cell_h_ : 1);
    if (i < 0) i = 0;
    if (i >= ny_) i = ny_ - 1;
    return static_cast<int>(i);
}

void ReservationSet::build(const std::vector<ConnectionTask>& tasks,
                           const std::vector<Corridor>& corridors,
                           const std::vector<double>& difficulties) {
    corridors_ = corridors;
    weights_.resize(tasks.size());
    for (std::size_t i = 0; i < tasks.size(); ++i)
        weights_[i] = 0.5 + (i < difficulties.size() ? difficulties[i] / 8.0 : 0.0);
    cells_.clear();
    nx_ = ny_ = 0;
    Rect b{};
    bool have = false;
    for (const auto& c : corridors_) {
        const Rect& r = c.rect;
        if (r.x2 < r.x1 || r.y2 < r.y1) continue;
        if (!have) {
            b = r;
            have = true;
        } else {
            b.x1 = std::min(b.x1, r.x1);
            b.y1 = std::min(b.y1, r.y1);
            b.x2 = std::max(b.x2, r.x2);
            b.y2 = std::max(b.y2, r.y2);
        }
    }
    if (!have) return;
    bounds_ = b;
    // ~one cell per corridor, bounded to [64, 4096] cells: fine enough that a
    // short query segment touches few corridors, small enough to stay cheap.
    int target = static_cast<int>(corridors_.size());
    if (target < 64) target = 64;
    if (target > 4096) target = 4096;
    int side = static_cast<int>(std::sqrt(static_cast<double>(target)));
    if (side < 1) side = 1;
    nx_ = side;
    ny_ = side;
    cell_w_ = std::max<Coord>(1, (bounds_.x2 - bounds_.x1) / nx_);
    cell_h_ = std::max<Coord>(1, (bounds_.y2 - bounds_.y1) / ny_);
    cells_.assign(static_cast<std::size_t>(nx_) * static_cast<std::size_t>(ny_), {});
    for (std::size_t i = 0; i < corridors_.size(); ++i) {
        const Rect& r = corridors_[i].rect;
        if (r.x2 < r.x1 || r.y2 < r.y1) continue;
        int x0 = cell_of_x(r.x1), x1 = cell_of_x(r.x2);
        int y0 = cell_of_y(r.y1), y1 = cell_of_y(r.y2);
        for (int y = y0; y <= y1; ++y)
            for (int x = x0; x <= x1; ++x)
                cells_[static_cast<std::size_t>(y) * nx_ + x].push_back(
                    static_cast<int>(i));
    }
}

Coord ReservationSet::penalty_for_segment(std::size_t self_task, const Segment& seg) const {
    Rect r = seg.bounds();
    double acc = 0.0;
    if (nx_ <= 0 || ny_ <= 0 || cells_.empty()) {
        // No grid: legacy linear scan with the exact saturation early-out.
        for (std::size_t i = 0; i < corridors_.size(); ++i) {
            if (i == self_task) continue;
            if (corridors_[i].rect.intersects(r)) {
                acc += weights_[i];
                if (acc * 20000.0 * strength_ >= 800000.0) return 800000;
            }
        }
        Coord p = static_cast<Coord>(acc * 20000.0 * strength_);
        return std::min<Coord>(p, 800000);
    }
    // Gather the corridors near this segment (superset), then visit them in
    // ascending index order == the legacy accumulation order. Exact.
    static thread_local std::vector<int> cand;
    cand.clear();
    int x0 = cell_of_x(r.x1), x1 = cell_of_x(r.x2);
    int y0 = cell_of_y(r.y1), y1 = cell_of_y(r.y2);
    // Perf (exact): row-clipped gather (see CongestionMap::penalty_for_segment).
    // The candidate set is a superset of the old bbox sweep, so after the sort
    // + unique + per-corridor intersects() gate the accumulated cost is
    // bit-identical; only bbox-corner cells are skipped.
    const bool horizontal = (seg.a.y == seg.b.y);
    const long double ax = seg.a.x, ay = seg.a.y, bx = seg.b.x, by = seg.b.y;
    const long double dx = bx - ax, dy = by - ay;
    for (int y = y0; y <= y1; ++y) {
        int rx0 = x0, rx1 = x1;
        if (!horizontal) {
            const long double band0 = bounds_.y1 + (Coord)y * cell_h_;
            const long double band1 = band0 + cell_h_;
            long double ta = (band0 - ay) / dy, tb = (band1 - ay) / dy;
            if (ta > tb) std::swap(ta, tb);
            if (ta < 0.0L) ta = 0.0L;
            if (tb > 1.0L) tb = 1.0L;
            if (ta <= tb) {
                long double xa = ax + ta * dx, xb = ax + tb * dx;
                if (xa > xb) std::swap(xa, xb);
                rx0 = std::max(x0, cell_of_x(static_cast<Coord>(xa)) - 1);
                rx1 = std::min(x1, cell_of_x(static_cast<Coord>(xb)) + 1);
            }
        }
        for (int x = rx0; x <= rx1; ++x) {
            const std::vector<int>& v = cells_[static_cast<std::size_t>(y) * nx_ + x];
            cand.insert(cand.end(), v.begin(), v.end());
        }
    }
    std::sort(cand.begin(), cand.end());
    cand.erase(std::unique(cand.begin(), cand.end()), cand.end());
    for (int i : cand) {
        if (static_cast<std::size_t>(i) == self_task) continue;
        if (corridors_[static_cast<std::size_t>(i)].rect.intersects(r)) {
            acc += weights_[static_cast<std::size_t>(i)];
            if (acc * 20000.0 * strength_ >= 800000.0) return 800000;
        }
    }
    Coord p = static_cast<Coord>(acc * 20000.0 * strength_);
    return std::min<Coord>(p, 800000);
}

// ---- Candidate generation (worker body) ----

CandidateRoute route_candidate_task(const Board& snapshot, const RuleResolver& resolver,
                                    const ConnectionTask& task, std::size_t task_index,
                                    double difficulty, const ElectricalContext& ctx,
                                    const std::vector<double>& layer_mult,
                                    const AStarConfig& astar_cfg, const CongestionMap& congestion,
                                    const ReservationSet& reservations,
                                    const HierarchyConfig& hier_cfg,
                                    const HierarchyCache* hier_cache,
                                    const SparseGraphBudget* graph_budget,
                                    RoutePhaseTiming* timing) {
    CandidateRoute cand;
    // --time-stages phase clocks (no-op when timing == nullptr).
    auto now_tp = []() { return std::chrono::steady_clock::now(); };
    auto add_phase = [&](std::atomic<std::int64_t>& acc,
                         std::chrono::steady_clock::time_point t0) {
        if (timing == nullptr) return;
        acc.fetch_add(std::chrono::duration_cast<std::chrono::nanoseconds>(now_tp() - t0)
                          .count(),
                      std::memory_order_relaxed);
    };
    if (timing != nullptr) timing->calls.fetch_add(1, std::memory_order_relaxed);
    cand.task = task;
    cand.task_index = task_index;
    cand.difficulty = difficulty;
    const Terminal* ta = snapshot.find_terminal(task.a);
    const Terminal* tb = snapshot.find_terminal(task.b);
    if (!ta || !tb) {
        cand.fail_reason = "bad_task";
        return cand;
    }
    // Issue #12: pair corridors route midpoint-to-midpoint with the
    // occupied envelope. Both member pad sets must exist.
    const DiffPair* pair = nullptr;
    if (task.is_pair_corridor) {
        for (const auto& pr : snapshot.diffpairs) {
            if (pr.id == task.pair_id) {
                pair = &pr;
                break;
            }
        }
        if (!pair) {
            cand.fail_reason = "bad_task";
            return cand;
        }
        if (!snapshot.find_terminal(task.pair_a_other) ||
            !snapshot.find_terminal(task.pair_b_other)) {
            cand.fail_reason = "bad_task";
            return cand;
        }
        cand.gate_a = task_src_point(snapshot, task);
        cand.gate_b = task_dst_point(snapshot, task);
    } else {
        cand.gate_a = ta->pos;
        cand.gate_b = task_dst_point(snapshot, task);
    }
    // Issue #11: explicit impedance/current conflicts never route silently.
    // A net whose ampacity floor breaks the impedance tolerance on every
    // feasible layer (or that solves on no layer at all) fails here with a
    // machine-readable reason instead of committing wrong-width copper.
    const NetInfo* imp_net = snapshot.find_net(task.net);
    bool imp_active = imp_net && resolver.impedance().has_target(*imp_net);
    ImpedanceResolution imp_res;
    if (imp_active) {
        imp_res = resolver.impedanceResolution(task.net, ctx);
        if (imp_res.has_target && !imp_res.feasible) {
            cand.fail_reason =
                imp_res.conflict ? "impedance_current_conflict" : "impedance_infeasible";
            cand.via_reason = cand.fail_reason;
            const NetInfo* ninfo0 = snapshot.find_net(task.net);
            if (ninfo0) {
                bool dummy = false;
                cand.required_current_a = resolver.current().effective_current(
                    *ninfo0, snapshot.defaults, dummy);
                auto ordered = ViaBundlePlanner::ordered_styles(
                    resolver, task.net,
                    LayerSpan{snapshot.layers.front().id, snapshot.layers.back().id});
                if (!ordered.empty()) {
                    cand.via_style = ordered.front().name;
                    cand.vias_required = ViaBundlePlanner::required_count(
                        resolver, ordered.front(), task.net);
                }
            }
            return cand;
        }
    }
    TraceRule rule = resolver.traceRule(task.net, ta->layer, kAnyRegion);
    Coord width = rule.pref_width_nm;
    if (imp_active) {
        // Conservative sizing: the graph is built with the max reconciled
        // width so per-layer narrowing below stays legal.
        width = resolver.maxRequiredWidth(task.net, ctx);
    }
    // Issue #12: the corridor reserves the full occupied envelope; both
    // members must also be individually impedance-feasible (never route a
    // corridor the materializer cannot fill).
    if (pair) {
        width = diffpair_occupied_width(snapshot, resolver, *pair, ctx);
        for (NetId member : {pair->net_p, pair->net_n}) {
            const NetInfo* mn = snapshot.find_net(member);
            if (mn && resolver.impedance().has_target(*mn)) {
                ImpedanceResolution mr = resolver.impedanceResolution(member, ctx);
                if (mr.has_target && !mr.feasible) {
                    cand.fail_reason = mr.conflict ? "impedance_current_conflict"
                                                   : "impedance_infeasible";
                    cand.via_reason = cand.fail_reason;
                    return cand;
                }
            }
        }
    }

    // Issue #4: resolve the live multi-target set from the snapshot. Stored
    // copper points are hints; recomputing from current same-net components
    // keeps rip-up/reroute branches correct when copper moved. Deterministic:
    // nearest-first (manhattan, x, y, layer), capped.
    // Issue #5: pair corridors run portal-midpoint to portal-midpoint when
    // both members carry escape stubs (fallback to pad midpoints).
    std::vector<SparseTarget> dsts;
    LayerId primary_dst_layer = task_dst_layer(snapshot, task);
    Point src_pt = ta->pos;
    LayerId src_layer = ta->layer;
    if (pair) {
        src_pt = task_src_point(snapshot, task);
        src_layer = task_src_layer(snapshot, task);
        dsts.push_back({task_dst_point(snapshot, task), primary_dst_layer});
    } else if (task.has_copper_target) {
        std::vector<TermId> target_comp;
        for (const auto& grp : net_terminal_components(snapshot, task.net)) {
            if (std::find(grp.begin(), grp.end(), task.b) != grp.end()) {
                target_comp = grp;
                break;
            }
        }
        if (target_comp.empty()) target_comp = {task.b};
        std::vector<CopperTarget> contacts =
            copper_contacts_for(snapshot, task.net, task.a, target_comp, 8);
        for (const auto& ct : contacts) dsts.push_back({ct.p, ct.layer});
        if (dsts.empty()) dsts.push_back({tb->pos, tb->layer});
        primary_dst_layer = dsts.front().layer;
    } else {
        dsts.push_back({tb->pos, tb->layer});
    }

    if (task.has_plane_target) {
        // Issue #16: live plane entry from the snapshot. Planes are static,
        // but recomputing keeps rip-up/reroute branches correct and the
        // selection deterministic (nearest, same-layer preference, id).
        int pid = -1, island = 0;
        Point entry{};
        LayerId elayer = task.plane_layer;
        if (nearest_plane_target(snapshot, task.net, ta->pos, ta->layer, pid, entry,
                                 elayer, island)) {
            dsts.clear();
            dsts.push_back({entry, elayer});
            primary_dst_layer = elayer;
        }
    }

    // Effective layer multipliers first: hierarchical guidance needs the
    // same impedance/pair-adjusted costs as the exact search.
    std::vector<double> eff_mult =
        effective_layer_mult(snapshot, resolver, task, layer_mult);
    if (pair && !pair->preferred_layers.empty()) {
        // Issue #12: soft preference for the pair's corridor layers (2x off
        // preferred layers). Never a hard lock: the only legal route still
        // routes, and the materializer re-gates the layer set exactly.
        std::set<LayerId> pref(pair->preferred_layers.begin(),
                               pair->preferred_layers.end());
        for (const auto& l : snapshot.layers) {
            if (!pref.count(l.id) && l.id >= 0 &&
                l.id < static_cast<int>(eff_mult.size()))
                eff_mult[l.id] *= 2.0;
        }
    }

    Board filt;
    const Board* graph_board = &snapshot;
    NetId pair_exempt = -1;
    if (pair) {
        filt = snapshot;
        // Issue #12/#6: sibling copper lives inside the reserved envelope;
        // it is same-resource copper for corridor planning (gap-governed at
        // materialization, not voltage-governed here). Terminals are
        // relabeled so own-net connectivity holds; traces/vias/planes stay
        // under their sibling net but are skipped as obstacles via exempt_net
        // (sparse graph + via planner). Relabeling traces/vias as well keeps
        // the #17 simplifier (which has no exempt param) consistent.
        pair_exempt = pair->net_n;
        for (auto& t : filt.terminals) {
            if (t.net == pair->net_n) t.net = pair->net_p;
        }
        for (auto& t : filt.traces) {
            if (t.net == pair->net_n) t.net = pair->net_p;
        }
        for (auto& v : filt.vias) {
            if (v.net == pair->net_n) v.net = pair->net_p;
        }
        for (auto& z : filt.planes) {
            if (z.net == pair->net_n) z.net = pair->net_p;
        }
        graph_board = &filt;
    }

    // Issue #10: hierarchical coarse-to-fine guidance. The corridor is
    // computed BEFORE graph construction so the sparse graph itself can be
    // clipped to the tube (fewer bases, edges and bundle plans); the exact
    // search then runs biased inside the corridor. Exact integer geometry
    // stays the sole legality source: clipped graphs are strict subsets of
    // the full legal graph, and any clipped miss rebuilds wider, ending in
    // the unrestricted exact search. Guidance never removes a solution.
    HierarchyRequest req;
    req.net = task.net;
    req.src = src_pt;
    req.src_layer = src_layer;
    req.dsts = dsts;
    req.width_nm = width;
    req.clearance_nm = max_clear_for(*graph_board, resolver, task.net, ctx);
    req.layer_mult = eff_mult;
    req.astar_cfg = astar_cfg;
    req.soft_cost = [&](const Segment& s) -> Coord {
        return congestion.penalty_for_segment(s) +
               reservations.penalty_for_segment(task_index, s);
    };
    GuidanceResult guide;
    bool guidance_on = hier_cfg.enabled && hier_cache != nullptr;
    auto t_guide0 = now_tp();
    if (guidance_on) {
        guide = hier_cache->build_guidance(*graph_board, resolver, ctx, req,
                                           hier_cfg);
        cand.hierarchy.attempted = true;
        cand.hierarchy.levels_used_mm = guide.levels_used_mm;
        cand.hierarchy.coarse_expansions = guide.coarse_expansions;
    }
    if (timing != nullptr) add_phase(timing->guidance_ns, t_guide0);

    // Issue #4: effective sparse-graph budget for this task. Null input =
    // legacy defaults (384 bases / K=16) for speed; the engine passes the
    // maturity EffectiveSearchBudget here so dense phases build wider graphs.
    const SparseGraphBudget eff_budget =
        graph_budget ? *graph_budget : SparseGraphBudget::defaults();
    auto apply_soft = [&](SparseRoutingGraph& g) {
        // Soft costs only: congestion + reservations bias the search,
        // legality is structural (illegal edges were never built).
        g.add_penalties([&](const SparseNode& n, const SparseEdge& e) -> Coord {
            Coord p = 0;
            if (e.is_via) {
                p += congestion.penalty_for_segment(Segment{n.p, n.p});
                p += reservations.penalty_for_segment(task_index, Segment{n.p, n.p});
            } else if (e.dir2 >= 0) {
                Segment s1{n.p, e.elbow}, s2{e.elbow, g.nodes()[e.to].p};
                p += congestion.penalty_for_segment(s1) + congestion.penalty_for_segment(s2);
                p += reservations.penalty_for_segment(task_index, s1) +
                     reservations.penalty_for_segment(task_index, s2);
            } else {
                Segment s{n.p, g.nodes()[e.to].p};
                p += congestion.penalty_for_segment(s);
                p += reservations.penalty_for_segment(task_index, s);
            }
            return p;
        });
    };
    auto build_graph_with = [&](const std::vector<Point>* clip, Coord half,
                                const SparseGraphBudget& b) {
        auto t0 = now_tp();
        SparseRoutingGraph g =
            pair ? SparseRoutingGraph::build_multi(filt, resolver, task.net, src_pt,
                                                   src_layer, dsts, width, ctx, clip,
                                                   half, b, pair_exempt)
                 : SparseRoutingGraph::build_multi(snapshot, resolver, task.net, src_pt,
                                                   ta->layer, dsts, width, ctx, clip,
                                                   half, b);
        if (timing != nullptr) add_phase(timing->build_ns, t0);
        auto t1 = now_tp();
        apply_soft(g);
        if (timing != nullptr) add_phase(timing->soft_ns, t1);
        return g;
    };
    auto build_graph = [&](const std::vector<Point>* clip, Coord half) {
        return build_graph_with(clip, half, eff_budget);
    };
    auto pull_bias = [&](const SparseRoutingGraph& g,
                         const std::vector<Point>& path) {
        // Weak pull-to-path ordering assist (never legality): quartered
        // distance, capped, so the Manhattan heuristic stays
        // near-consistent. Zero on the corridor; deterministic.
        std::vector<Coord> bias(g.nodes().size(), 0);
        for (std::size_t i = 0; i < g.nodes().size(); ++i) {
            Coord best = std::numeric_limits<Coord>::max();
            for (const auto& p : path) {
                Coord d = manhattan(g.nodes()[i].p, p);
                if (d < best) best = d;
            }
            if (best == std::numeric_limits<Coord>::max()) best = 0;
            bias[i] = std::min(hier_cfg.max_bias_nm, best / 4);
        }
        return bias;
    };

    SparseRoutingGraph graph;
    AStarResult res;
    if (guidance_on && guide.found && !guide.level_paths.empty()) {
        const std::vector<Point>& path = guide.level_paths.back();
        // Clipped attempts around the corridor (widening), then the
        // unrestricted exact fallback on the full graph.
        int attempts = std::max(1, hier_cfg.max_window_attempts);
        Coord tube = hier_cfg.tube_half_nm;
        for (int attempt = 0; attempt < attempts; ++attempt) {
            graph = build_graph(&path, tube);
            auto t_a0 = now_tp();
            res = astar_route_masked(graph, eff_mult, astar_cfg, {},
                                     pull_bias(graph, path));
            if (timing != nullptr) add_phase(timing->astar_ns, t_a0);
            cand.hierarchy.window_attempts = attempt + 1;
            if (res.found) {
                cand.hierarchy.guided = true;
                cand.hierarchy.fallback_reason =
                    attempt == 0 ? "guided" : "guided_expanded";
                break;
            }
            tube *= 4;
        }
        if (!res.found) {
            // Unrestricted exact fallback on the full graph.
            graph = build_graph(nullptr, 0);
            auto t_a0 = now_tp();
            res = astar_route(graph, eff_mult, astar_cfg);
            if (timing != nullptr) add_phase(timing->astar_ns, t_a0);
            cand.hierarchy.guided = false;
            cand.hierarchy.fallback = true;
            cand.hierarchy.fallback_reason = "window_miss";
        }
        cand.hierarchy.exact_expansions = res.expansions;
    } else {
        graph = build_graph(nullptr, 0);
        auto t_a0 = now_tp();
        res = astar_route(graph, eff_mult, astar_cfg);
        if (timing != nullptr) add_phase(timing->astar_ns, t_a0);
        cand.hierarchy.exact_expansions = res.expansions;
        if (guidance_on)
            cand.hierarchy.fallback_reason = "coarse_fail";
        else
            cand.hierarchy.fallback_reason = "disabled";
        if (guidance_on && !guide.found) cand.hierarchy.fallback = true;
    }
    // Issue #4: genuinely expanded last-resort mode. When the exact search
    // misses for reachability (not A* budget exhaustion), rebuild the graph
    // with uncapped bases + K=64 and re-run the unrestricted exact search
    // before declaring the task unreachable. The culled graph is a strict
    // subset of this one, so a miss here is real evidence of blockage.
    // Skipped when the task already ran at last-resort scale.
    if (!res.found && res.fail_reason != "budget_exhausted" &&
        !eff_budget.is_last_resort()) {
        const SparseGraphBudget lr = SparseGraphBudget::last_resort();
        SparseRoutingGraph lr_graph = build_graph_with(nullptr, 0, lr);
        auto t_a0 = now_tp();
        AStarResult lr_res = astar_route(lr_graph, eff_mult, astar_cfg);
        if (timing != nullptr) add_phase(timing->astar_ns, t_a0);
        if (lr_res.found) {
            graph = std::move(lr_graph);
            res = std::move(lr_res);
            cand.hierarchy.guided = false;
            cand.hierarchy.fallback = true;
            cand.hierarchy.fallback_reason = "last_resort";
            cand.hierarchy.exact_expansions = res.expansions;
        } else if (lr_res.fail_reason != "budget_exhausted") {
            // Fullest-search evidence wins for attribution.
            graph = std::move(lr_graph);
            res = std::move(lr_res);
            cand.hierarchy.guided = false;
            cand.hierarchy.fallback = true;
            cand.hierarchy.fallback_reason = "last_resort_miss";
            cand.hierarchy.exact_expansions = res.expansions;
        }
    }
    cand.expansions = res.expansions;
    cand.closest_node = res.closest_node;
    cand.closest_goal_dist_nm = res.closest_goal_dist_nm;
    // Issue #23: forward A*-frontier-only rejection evidence from the search
    // (`res`), not construction-wide graph stats: only transitions out of
    // nodes A* actually expanded count. Both success and failure carry it;
    // attribution uses it on failure. Bounded top-N, deterministic.
    cand.frontier_blockers.reserve(res.frontier_stats.size());
    for (const auto& s : res.frontier_stats) {
        FrontierBlockerStat f;
        f.blocker_net = s.blocker_net;
        f.kind = s.kind;
        f.desc = s.desc;
        f.layer = s.layer;
        f.pos = s.pos;
        f.count = s.count;
        cand.frontier_blockers.push_back(f);
    }
    // Baseline via diagnostics from the ordered selection (no geometry).
    {
        const NetInfo* ninfo = snapshot.find_net(task.net);
        bool dummy = false;
        cand.required_current_a =
            ninfo ? resolver.current().effective_current(*ninfo, snapshot.defaults, dummy)
                  : 0.0;
        auto ordered = ViaBundlePlanner::ordered_styles(
            resolver, task.net,
            LayerSpan{snapshot.layers.front().id, snapshot.layers.back().id});
        if (!ordered.empty()) {
            cand.via_style = ordered.front().name;
            cand.vias_required = ViaBundlePlanner::required_count(resolver, ordered.front(),
                                                                  task.net);
        } else {
            cand.via_style.clear();
            cand.vias_required = 1;
            cand.via_reason = "no_via_class";
        }
    }
    if (!res.found) {
        cand.fail_reason = res.fail_reason == "budget_exhausted" ? "budget_exhausted" : "unreachable";
        // Explicit bundle infeasibility (issue #5): a layer-changing task on
        // a graph with zero via edges can never transition. Distinguish a
        // blocked parallel bundle from a plain maze failure so agents get a
        // stable reason category.
        if (src_layer != primary_dst_layer) {
            int via_edges = 0;
            for (std::size_t ni = 0; ni < graph.nodes().size(); ++ni)
                for (const auto& e : graph.edges(static_cast<int>(ni)))
                    if (e.is_via) ++via_edges;
            if (via_edges == 0) {
                LayerSpan span{src_layer, primary_dst_layer};
                NetId probe_exempt = pair ? pair->net_n : -1;
                ViaBundle probe = ViaBundlePlanner::plan(snapshot, resolver, task.net,
                                                         src_pt, span, width, ctx,
                                                         probe_exempt);
                cand.via_style = probe.via_class;
                cand.vias_required = std::max(1, probe.count);
                cand.required_current_a = probe.required_current_a;
                if (!probe.feasible) {
                    cand.via_reason = probe.reason;
                    cand.fail_reason = probe.reason == "no_via_class"
                                           ? "no_via"
                                           : "via_bundle_infeasible";
                }
            }
        }
        return cand;
    }
    cand.found = true;
    auto t_mat0 = now_tp();
    // Issue #8: search cost is debug-only from here on; ordering uses the
    // recomputed materialized cost below (after simplification).
    cand.search_cost_nm = res.cost_nm;
    cand.cost_nm = res.cost_nm;
    // Issue #12: pair corridors carry a single center via per transition
    // (no bundle stubs). The DiffPairMaterializer rebuilds symmetric paired
    // vias + stubs post-route; corridor stubs would corrupt the centerline.
    if (pair) {
        for (std::size_t i = 0; i < res.edge_path.size(); ++i) {
            int u = res.node_path[i];
            const SparseEdge& e = graph.edges(u)[res.edge_path[i]];
            const SparseNode& nu = graph.nodes()[u];
            const SparseNode& nv = graph.nodes()[e.to];
            if (e.is_via) {
                LayerSpan span{std::min(nu.layer, nv.layer),
                               std::max(nu.layer, nv.layer)};
                ViaStyle style;
                if (!resolver.select_via(task.net, span, style)) {
                    cand.found = false;
                    cand.traces.clear();
                    cand.vias.clear();
                    cand.via_reason = "no_via_class";
                    cand.fail_reason = "no_via";
                    return cand;
                }
                // Issue #7: pair-specific via-transition footprint. The
                // materializer rebuilds symmetric P+N barrels + stubs around
                // the center, so gate each corridor transition on the
                // occupied envelope (width == wp+wn+gap+2*ext), not a single
                // via: both members must plan an occupied-width bundle at the
                // center (sibling-exempt, gap-governed). Otherwise the
                // corridor would commit a transition the materializer cannot
                // realize as paired vias.
                {
                    ViaBundle bP = ViaBundlePlanner::plan(
                        snapshot, resolver, task.net, nu.p, span, width, ctx,
                        pair_exempt);
                    ViaBundle bN = ViaBundlePlanner::plan(
                        snapshot, resolver, pair->net_n, nu.p, span, width, ctx,
                        task.net);
                    if (!bP.feasible || !bN.feasible) {
                        cand.found = false;
                        cand.traces.clear();
                        cand.vias.clear();
                        cand.via_style = bP.feasible ? bN.via_class : bP.via_class;
                        cand.vias_required =
                            std::max(bP.feasible ? 1 : bP.count, bN.feasible ? 1 : bN.count);
                        cand.required_current_a = bP.required_current_a;
                        cand.via_reason = "bundle_blocked";
                        cand.fail_reason = "via_bundle_infeasible";
                        return cand;
                    }
                }
                Via v;
                v.net = task.net;
                v.pos = nu.p;
                v.top_layer = span.top;
                v.bottom_layer = span.bottom;
                v.outer_d_nm = style.outer_nm;
                v.hole_d_nm = style.hole_nm;
                v.via_class = style.name;
                cand.vias.push_back(v);
                cand.via_style = style.name;
                cand.via_reason = "ok";
            } else if (e.dir2 >= 0) {
                cand.traces.push_back({task.net, nu.layer, nu.p, e.elbow, width});
                if (!(e.elbow == nv.p))
                    cand.traces.push_back({task.net, nu.layer, e.elbow, nv.p, width});
            } else {
                if (!(nu.p == nv.p))
                    cand.traces.push_back({task.net, nu.layer, nu.p, nv.p, width});
            }
        }
        // Corridor traces stay at the occupied envelope width (never
        // narrowed per layer: the envelope must fit everywhere).
        // Simplify against the sibling-exempt view (same board the graph
        // was built on) so the envelope keeps #17 minimum-bend geometry.
        std::vector<char> no_stub(cand.traces.size(), 0);
        simplify_candidate_traces(filt, resolver, ctx, task.net, cand.traces,
                                  no_stub, simplify_exempt_task(task));
        // Issue #8: recompute from final copper (simplification shortens it).
        cand.materialized_cost_nm =
            materialized_route_cost(cand.traces, cand.vias, eff_mult, astar_cfg);
        cand.cost_nm = cand.materialized_cost_nm;
        if (timing != nullptr) add_phase(timing->materialize_ns, t_mat0);
        return cand;
    }
    // Materialize every A* layer transition as one atomic bundle (issue #5):
    // all barrels plus both layers' star stubs, or the whole candidate
    // fails with an explicit reason. Never half-build a transition.
    // Route segments and bundle stubs are collected separately: issue #17
    // simplifies route runs to arbitrary-angle minimum-bend geometry while
    // stub traces stay verbatim (they were planned atomically).
    bool saw_via = false;
    std::vector<TraceSeg> route_segs;
    std::vector<TraceSeg> stub_segs;
    for (std::size_t i = 0; i < res.edge_path.size(); ++i) {
        int u = res.node_path[i];
        const SparseEdge& e = graph.edges(u)[res.edge_path[i]];
        const SparseNode& nu = graph.nodes()[u];
        const SparseNode& nv = graph.nodes()[e.to];
        if (e.is_via) {
            LayerSpan span{nu.layer, nv.layer};
            ViaBundle bundle = ViaBundlePlanner::plan(snapshot, resolver, task.net, nu.p,
                                                      span, width, ctx);
            if (!bundle.feasible) {
                cand.found = false;
                cand.traces.clear();
                cand.vias.clear();
                cand.via_style = bundle.via_class;
                cand.vias_required = std::max(1, bundle.count);
                cand.required_current_a = bundle.required_current_a;
                cand.via_reason = bundle.reason;
                cand.fail_reason =
                    bundle.reason == "no_via_class" ? "no_via" : "via_bundle_infeasible";
                return cand;
            }
            if (!saw_via) {
                cand.via_style = bundle.style.name;
                cand.vias_required = bundle.count;
                cand.via_reason = "ok";
                cand.required_current_a = bundle.required_current_a;
                saw_via = true;
            } else {
                cand.vias_required = std::max(cand.vias_required, bundle.count);
            }
            for (auto p : bundle.positions) {
                Via v;
                v.net = task.net;
                v.pos = p;
                v.top_layer = std::min(nu.layer, nv.layer);
                v.bottom_layer = std::max(nu.layer, nv.layer);
                v.outer_d_nm = bundle.style.outer_nm;
                v.hole_d_nm = bundle.style.hole_nm;
                v.via_class = bundle.style.name;
                cand.vias.push_back(v);
            }
            for (const auto& s : bundle.stubs) stub_segs.push_back(s);
        } else if (e.dir2 >= 0) {
            route_segs.push_back({task.net, nu.layer, nu.p, e.elbow, width});
            if (!(e.elbow == nv.p)) route_segs.push_back({task.net, nu.layer, e.elbow, nv.p, width});
        } else {
            if (!(nu.p == nv.p)) route_segs.push_back({task.net, nu.layer, nu.p, nv.p, width});
        }
    }
    if (!saw_via) cand.via_reason = "ok";
    if (imp_active) {
        // Issue #11: assign each same-layer run its reconciled layer width.
        // Shrink-only versus the max-width graph, so legality is preserved.
        // Via-bundle star stubs keep their planned width (atomic bundles).
        for (auto& s : route_segs)
            s.width_nm = resolver.requiredTraceWidth(task.net, s.layer, ctx);
    }
    // Issue #17: arbitrary-angle minimum-bend simplification of each
    // same-layer route run (endpoints fixed, exact legality gate, prior
    // retained on failure). Tuning-exempt tasks (#15 hook) keep guidance
    // geometry verbatim. Covers ordinary routes, plane access and reroutes:
    // all flow through this worker body.
    cand.traces = route_segs;
    {
        std::vector<char> stub_mask(route_segs.size(), 0);
        cand.traces.insert(cand.traces.end(), stub_segs.begin(), stub_segs.end());
        stub_mask.resize(cand.traces.size(), 0);
        for (std::size_t i = route_segs.size(); i < cand.traces.size(); ++i)
            stub_mask[i] = 1;
        simplify_candidate_traces(snapshot, resolver, ctx, task.net, cand.traces,
                                  stub_mask, simplify_exempt_task(task));
    }
    // Issue #8: recompute from final copper (simplification + stubs change it).
    cand.materialized_cost_nm =
        materialized_route_cost(cand.traces, cand.vias, eff_mult, astar_cfg);
    cand.cost_nm = cand.materialized_cost_nm;
    if (timing != nullptr) add_phase(timing->materialize_ns, t_mat0);
    return cand;
}

// ---- Legality + conflicts (arbiter) ----

namespace {

// All committed copper plus already-accepted candidates, read-only.
struct LegalView {
    const Board& board;
    const std::vector<TraceSeg>* extra_traces = nullptr;
    const std::vector<Via>* extra_vias = nullptr;
    // S4: per-net clearance memos shared across every probe of an arbitration.
    // Thread-local per arbitrate() call (the arbiter itself is single-threaded;
    // workers never see LegalView).
    mutable std::map<NetId, ClearanceCache> caches;
    const ClearanceCache& cache_for(NetId net, const RuleResolver& resolver,
                                    const ElectricalContext& ctx) const {
        auto it = caches.find(net);
        if (it != caches.end()) return it->second;
        return caches.emplace(net, ClearanceCache(board, resolver, ctx, net))
            .first->second;
    }
};

bool trace_legal(const TraceSeg& s, const LegalView& v, const RuleResolver& resolver,
                 const ElectricalContext& ctx, std::string& why) {
    // Issue #12: corridor copper exempts its coupled sibling (gap-governed
    // at materialization, not voltage-governed here).
    NetId sibling = pair_partner_of(v.board, s.net);
    // D3: the board probe is the shared verifier-exact predicate (exact
    // 4*d2 >= rhs^2, bbox prechecks, {own, sibling} exemption). Verdicts and
    // reason labels match the old inline clone for even-nm widths (all
    // production widths); odd-nm razor cases follow the verifier exactly.
    SegLegalityCtx leg;
    leg.cc = v.cache_for(s.net, resolver, ctx);  // shares the memo table
    leg.exempt_net = sibling;
    leg.layer = s.layer;
    leg.width_nm = s.width_nm;
    if (!leg.segment_legal(s.segment(), &why)) return false;
    // Already-accepted candidates use the same verifier-exact math pairwise.
    Coord hw = s.width_nm / 2;
    auto check_trace = [&](const TraceSeg& t) -> bool {
        if (t.net == s.net || t.net == sibling || t.layer != s.layer) return true;
        Coord c = leg.cc.get(t.net, s.layer);
        Segment a = s.segment(), b = t.segment();
        __int128 rhs = (__int128)2 * c + s.width_nm + t.width_nm;
        if (!a.bounds().expanded(c + hw + t.width_nm / 2).intersects(b.bounds()))
            return true;
        return (__int128)4 * seg_seg_dist2(a, b) >= rhs * rhs;
    };
    auto check_via = [&](const Via& vv) -> bool {
        if (vv.net == s.net || vv.net == sibling) return true;
        if (s.layer < std::min(vv.top_layer, vv.bottom_layer) ||
            s.layer > std::max(vv.top_layer, vv.bottom_layer))
            return true;
        Coord c = leg.cc.get(vv.net, s.layer);
        __int128 rhs = (__int128)2 * c + s.width_nm;
        Rect vr = Rect::from_center_size(vv.pos, vv.outer_d_nm, vv.outer_d_nm);
        if (!s.segment().bounds().expanded(c + hw).intersects(vr)) return true;
        return (__int128)4 * seg_rect_dist2(s.segment(), vr) >= rhs * rhs;
    };
    if (v.extra_traces)
        for (const auto& t : *v.extra_traces)
            if (!check_trace(t)) {
                why = "clearance:trace";
                return false;
            }
    if (v.extra_vias)
        for (const auto& vv : *v.extra_vias)
            if (!check_via(vv)) {
                why = "clearance:via";
                return false;
            }
    return true;
}

bool via_legal(const Via& vv, const LegalView& v, const RuleResolver& resolver,
               const ElectricalContext& ctx, std::string& why) {
    Rect vr = Rect::from_center_size(vv.pos, vv.outer_d_nm, vv.outer_d_nm);
    if (!v.board.bounds().contains(vr)) {
        why = "off_board";
        return false;
    }
    // S4: shared per-net memo instead of a resolver scan per obstacle.
    const ClearanceCache& cc = v.cache_for(vv.net, resolver, ctx);
    Coord max_clear = cc.max_clear();
    // Issue #12: corridor vias exempt the coupled sibling (paired at
    // materialization under the gap rule).
    NetId vsibling = pair_partner_of(v.board, vv.net);
    auto check_vs_net = [&](NetId other, const Rect& raw) -> bool {
        if (other == vsibling) return true;
        Coord c = cc.get(other, vv.top_layer);
        return gap_ok_rect(vr, raw, c);
    };
    for (const auto& ko : v.board.keepouts) {
        bool span_hit = ko.layer == kAllLayers ||
                        (ko.layer >= std::min(vv.top_layer, vv.bottom_layer) &&
                         ko.layer <= std::max(vv.top_layer, vv.bottom_layer));
        if (!span_hit) continue;
        if (!gap_ok_rect(vr, ko.rect, max_clear)) {
            why = "keepout:" + ko.reason;
            return false;
        }
    }
    // Issue #16: via barrels keep clearance from foreign pours on spanned layers.
    for (const auto& z : v.board.planes) {
        if (z.net == vv.net) continue;
        if (z.layer < std::min(vv.top_layer, vv.bottom_layer) ||
            z.layer > std::max(vv.top_layer, vv.bottom_layer))
            continue;
        Coord c = cc.get(z.net, z.layer);
        if (vr.expanded(c).intersects(z.bounds())) {
            if (plane_rect_poly_dist2(vr, z.poly) < (__int128)c * c) {
                why = "clearance:plane";
                return false;
            }
        }
    }
    for (const auto& t : v.board.terminals) {
        if (t.net == vv.net || t.net == vsibling) continue;
        bool span_hit =
            t.layer >= std::min(vv.top_layer, vv.bottom_layer) &&
            t.layer <= std::max(vv.top_layer, vv.bottom_layer);
        if (!span_hit) continue;
        if (!check_vs_net(t.net, t.pad_rect())) {
            why = "clearance:pad";
            return false;
        }
    }
    auto check_trace = [&](const TraceSeg& t) -> bool {
        if (t.net == vv.net || t.net == vsibling) return true;
        if (t.layer < std::min(vv.top_layer, vv.bottom_layer) ||
            t.layer > std::max(vv.top_layer, vv.bottom_layer))
            return true;
        Coord c = cc.get(t.net, t.layer);
        __int128 d2 = seg_rect_dist2(t.segment(), vr);
        Coord need = c + t.width_nm / 2;
        return d2 >= (__int128)need * need;
    };
    for (const auto& t : v.board.traces)
        if (!check_trace(t)) {
            why = "clearance:trace";
            return false;
        }
    if (v.extra_traces)
        for (const auto& t : *v.extra_traces)
            if (!check_trace(t)) {
                why = "clearance:trace";
                return false;
            }
    auto check_via = [&](const Via& o) -> bool {
        if (o.net == vv.net || o.net == vsibling) return true;
        bool overlap = !(o.bottom_layer < std::min(vv.top_layer, vv.bottom_layer) ||
                         o.top_layer > std::max(vv.top_layer, vv.bottom_layer));
        if (!overlap) return true;
        Rect orr = Rect::from_center_size(o.pos, o.outer_d_nm, o.outer_d_nm);
        return check_vs_net(o.net, orr);
    };
    for (const auto& o : v.board.vias)
        if (!check_via(o)) {
            why = "clearance:via";
            return false;
        }
    if (v.extra_vias)
        for (const auto& o : *v.extra_vias)
            if (!check_via(o)) {
                why = "clearance:via";
                return false;
            }
    return true;
}

Rect candidate_bounds(const CandidateRoute& c) {
    if (c.traces.empty() && c.vias.empty()) return {c.gate_a.x, c.gate_a.y, c.gate_a.x, c.gate_a.y};
    Rect r{};
    bool first = true;
    auto grow = [&](const Rect& q) {
        if (first) {
            r = q;
            first = false;
        } else {
            r = {std::min(r.x1, q.x1), std::min(r.y1, q.y1), std::max(r.x2, q.x2),
                 std::max(r.y2, q.y2)};
        }
    };
    for (const auto& t : c.traces) grow(t.segment().bounds().expanded(t.width_nm / 2));
    for (const auto& vv : c.vias)
        grow(Rect::from_center_size(vv.pos, vv.outer_d_nm, vv.outer_d_nm));
    return r;
}

}  // namespace

bool candidate_legal_vs_board(const CandidateRoute& cand, const Board& committed,
                              const RuleResolver& resolver, const ElectricalContext& ctx,
                              std::string& reason_out) {
    LegalView v{committed, nullptr, nullptr};
    for (const auto& t : cand.traces)
        if (!trace_legal(t, v, resolver, ctx, reason_out)) return false;
    for (const auto& vv : cand.vias)
        if (!via_legal(vv, v, resolver, ctx, reason_out)) return false;
    return true;
}

bool candidates_conflict(const CandidateRoute& a, const CandidateRoute& b,
                         const RuleResolver& resolver, const ElectricalContext& ctx) {
    if (!a.found || !b.found || a.task.net == b.task.net) return false;
    // Bounds pre-check with the pair clearance.
    std::string cs;
    Coord c = resolver.requiredClearance(a.task.net, b.task.net, 0, ctx, &cs);
    Coord wa = 0, wb = 0;
    for (const auto& t : a.traces) wa = std::max(wa, t.width_nm / 2);
    for (const auto& t : b.traces) wb = std::max(wb, t.width_nm / 2);
    if (!candidate_bounds(a).expanded(c + wa + wb).intersects(candidate_bounds(b))) return false;
    for (const auto& ta : a.traces) {
        for (const auto& tb : b.traces) {
            if (ta.layer != tb.layer) continue;
            Segment sa = ta.segment(), sb = tb.segment();
            Coord need = c + ta.width_nm / 2 + tb.width_nm / 2;
            if (!sa.bounds().expanded(need).intersects(sb.bounds())) continue;
            if (seg_seg_dist2(sa, sb) < (__int128)need * need) return true;
        }
        for (const auto& vb : b.vias) {
            if (ta.layer < std::min(vb.top_layer, vb.bottom_layer) ||
                ta.layer > std::max(vb.top_layer, vb.bottom_layer))
                continue;
            Rect vr = Rect::from_center_size(vb.pos, vb.outer_d_nm, vb.outer_d_nm);
            if (!seg_ok_rect(ta.segment(), vr, c + ta.width_nm / 2)) return true;
        }
    }
    for (const auto& va : a.vias) {
        Rect vra = Rect::from_center_size(va.pos, va.outer_d_nm, va.outer_d_nm);
        for (const auto& tb : b.traces) {
            if (tb.layer < std::min(va.top_layer, va.bottom_layer) ||
                tb.layer > std::max(va.top_layer, va.bottom_layer))
                continue;
            if (!seg_ok_rect(tb.segment(), vra, c + tb.width_nm / 2)) return true;
        }
        for (const auto& vb : b.vias) {
            bool overlap = !(vb.bottom_layer < std::min(va.top_layer, va.bottom_layer) ||
                             vb.top_layer > std::max(va.top_layer, va.bottom_layer));
            if (!overlap) continue;
            Rect vr = Rect::from_center_size(vb.pos, vb.outer_d_nm, vb.outer_d_nm);
            if (!gap_ok_rect(vra, vr, c)) return true;
        }
    }
    return false;
}

ArbiterResult arbitrate(const std::vector<CandidateRoute>& candidates, const Board& committed,
                        const RuleResolver& resolver, const ElectricalContext& ctx) {
    ArbiterResult out;
    std::vector<std::size_t> order(candidates.size());
    for (std::size_t i = 0; i < order.size(); ++i) order[i] = i;
    // Deterministic selection order: difficulty desc, then (net, a, b).
    // Completion order of workers can never change this.
    std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
        if (candidates[a].difficulty != candidates[b].difficulty)
            return candidates[a].difficulty > candidates[b].difficulty;
        const auto& ta = candidates[a].task;
        const auto& tb = candidates[b].task;
        if (ta.net != tb.net) return ta.net < tb.net;
        if (ta.a != tb.a) return ta.a < tb.a;
        return ta.b < tb.b;
    });
    std::vector<TraceSeg> acc_traces;
    std::vector<Via> acc_vias;
    std::vector<std::size_t> acc_idx;
    LegalView v{committed, &acc_traces, &acc_vias};
    for (std::size_t i : order) {
        const CandidateRoute& c = candidates[i];
        if (!c.found) {
            out.rejected.push_back(i);
            out.reject_reason.push_back(c.fail_reason.empty() ? "unreachable" : c.fail_reason);
            continue;
        }
        bool conflict = false;
        for (std::size_t j : acc_idx) {
            if (candidates_conflict(c, candidates[j], resolver, ctx)) {
                conflict = true;
                break;
            }
        }
        if (conflict) {
            out.rejected.push_back(i);
            out.reject_reason.push_back("conflict");
            continue;
        }
        std::string why;
        bool ok = true;
        for (const auto& t : c.traces)
            if (!trace_legal(t, v, resolver, ctx, why)) {
                ok = false;
                break;
            }
        if (ok)
            for (const auto& vv : c.vias)
                if (!via_legal(vv, v, resolver, ctx, why)) {
                    ok = false;
                    break;
                }
        if (!ok) {
            out.rejected.push_back(i);
            out.reject_reason.push_back("illegal_overlap:" + why);
            continue;
        }
        out.accepted.push_back(i);
        acc_idx.push_back(i);
        acc_traces.insert(acc_traces.end(), c.traces.begin(), c.traces.end());
        acc_vias.insert(acc_vias.end(), c.vias.begin(), c.vias.end());
    }
    return out;
}

void commit_candidates(Board& committed, const std::vector<CandidateRoute>& candidates,
                       const ArbiterResult& arb, RouteStats& stats) {
    // Issue #10: hierarchy aggregates cover every candidate (accepted or
    // rejected); exact expansions keep their existing accounting.
    for (const auto& c : candidates) {
        if (c.hierarchy.attempted && c.hierarchy.guided) stats.hierarchy_guided_tasks++;
        if (c.hierarchy.attempted && c.hierarchy.fallback)
            stats.hierarchy_fallback_tasks++;
        stats.hierarchy_coarse_expansions += c.hierarchy.coarse_expansions;
    }
    for (std::size_t i : arb.accepted) {
        const CandidateRoute& c = candidates[i];
        for (const auto& t : c.traces) {
            stats.length_nm += euclid_len_nm(t.a, t.b);
            committed.traces.push_back(t);
        }
        for (const auto& vv : c.vias) {
            committed.vias.push_back(vv);
            stats.via_count++;
        }
        stats.tasks_routed++;
        stats.expansions_total += c.expansions;
    }
    // Rejected candidates still consumed search work: count it.
    for (std::size_t i : arb.rejected) stats.expansions_total += candidates[i].expansions;
}

JsonValue EpochInfo::to_json() const {
    JsonValue o = JsonValue::object();
    o["epoch"] = static_cast<double>(epoch);
    o["batch_size"] = static_cast<double>(batch_size);
    o["candidates"] = static_cast<double>(candidates);
    o["accepted"] = static_cast<double>(accepted);
    o["rejected"] = static_cast<double>(rejected);
    o["expansions"] = static_cast<double>(expansions);
    o["workers"] = static_cast<double>(workers);
    o["time_ms"] = static_cast<double>(time_ms);
    // Issue #3: batch IDs + pairwise interference + width/memory stats.
    JsonValue bids = JsonValue::array();
    for (int id : batch_task_ids) bids.as_array().push_back(JsonValue(static_cast<double>(id)));
    o["batch_task_ids"] = bids;
    JsonValue pairs = JsonValue::array();
    for (const auto& p : interference_pairs) {
        JsonValue e = JsonValue::object();
        e["a"] = static_cast<double>(p.first.first);
        e["b"] = static_cast<double>(p.first.second);
        e["weight"] = p.second;
        pairs.as_array().push_back(e);
    }
    o["interference_pairs"] = pairs;
    o["interference_threshold_used"] = interference_threshold_used;
    o["interference_relax_steps"] = static_cast<double>(interference_relax_steps);
    o["interference_max"] = interference_max;
    o["interference_mean"] = interference_mean;
    o["effective_batch_width"] = static_cast<double>(effective_batch_width);
    // Issue #14: maturity phase + effective budget snapshot for this epoch.
    o["maturity_phase"] = maturity_phase;
    o["maturity"] = maturity;
    o["budget"] = budget;
    return o;
}

std::string geometry_hash(const Board& board) {
    // FNV-1a/64 over committed copper in commit order (deterministic).
    std::uint64_t h = 1469598103934665603ULL;
    auto mix = [&](const void* p, std::size_t n) {
        const auto* b = static_cast<const unsigned char*>(p);
        for (std::size_t i = 0; i < n; ++i) {
            h ^= b[i];
            h *= 1099511628211ULL;
        }
    };
    auto mix64 = [&](std::uint64_t v) {
        for (int i = 0; i < 8; ++i) {
            h ^= static_cast<unsigned char>(v & 0xff);
            h *= 1099511628211ULL;
            v >>= 8;
        }
    };
    for (const auto& t : board.traces) {
        mix64(static_cast<std::uint64_t>(t.net));
        mix64(static_cast<std::uint64_t>(t.layer));
        mix64(static_cast<std::uint64_t>(t.a.x));
        mix64(static_cast<std::uint64_t>(t.a.y));
        mix64(static_cast<std::uint64_t>(t.b.x));
        mix64(static_cast<std::uint64_t>(t.b.y));
        mix64(static_cast<std::uint64_t>(t.width_nm));
    }
    mix("V", 1);
    for (const auto& v : board.vias) {
        mix64(static_cast<std::uint64_t>(v.net));
        mix64(static_cast<std::uint64_t>(v.pos.x));
        mix64(static_cast<std::uint64_t>(v.pos.y));
        mix64(static_cast<std::uint64_t>(v.top_layer));
        mix64(static_cast<std::uint64_t>(v.bottom_layer));
        mix64(static_cast<std::uint64_t>(v.outer_d_nm));
        mix64(static_cast<std::uint64_t>(v.hole_d_nm));
    }
    char buf[17];
    std::snprintf(buf, sizeof(buf), "%016llx", (unsigned long long)h);
    return std::string(buf);
}

}  // namespace copperline
