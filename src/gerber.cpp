// Copperline: Gerber RS-274X import (minimal but real; see gerber.h).
//
// Line-streaming parser: the file is read in 64KB chunks and complete
// `*`-terminated blocks are handled incrementally, so large Gerber jobs
// never hold a whole-file DOM. Buffered state is bounded (apertures <=
// 64k, region vertices <= 32k each, primitives <= 2M, SR repeat <= 10k).
#include "router/gerber.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <map>
#include <numbers>
#include <sstream>

namespace copperline {
namespace {

// Memory caps (router <= 2048MB total).
constexpr std::size_t kMaxApertures = 65536;
constexpr std::size_t kMaxRegionVertices = 32768;
constexpr std::size_t kMaxGerberPrimitives = 2000000;
constexpr std::size_t kMaxSrRepeat = 10000;
constexpr std::size_t kStreamChunk = 65536;

struct Aperture {
    bool valid = false;
    char kind = 'C';  // C circle, R rect, O obround, P polygon, M macro-approx
    double w_mm = 0;  // effective copper size (circle: w==h==dia)
    double h_mm = 0;
};

// One parsed operation. Coordinates are FILE UNITS (scaled by %MO% only
// at board-build time); x1/y1 = flash point or draw start, x2/y2 = draw end.
struct Op {
    char kind = 'F';  // F flash, D draw, R region-vertex, M region-move
    double x1 = 0, y1 = 0, x2 = 0, y2 = 0;
    int ap = -1;      // aperture D-code at op time
    bool dark = true;  // polarity at op time
};

bool ends_with_ci(const std::string& s, const std::string& suffix) {
    if (suffix.size() > s.size()) return false;
    for (std::size_t i = 0; i < suffix.size(); ++i) {
        if (std::tolower(s[s.size() - suffix.size() + i]) != std::tolower(suffix[i]))
            return false;
    }
    return true;
}

std::string lower_str(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(c));
    return s;
}

std::string trim(const std::string& s) {
    std::size_t a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    return s.substr(a, s.find_last_not_of(" \t\r\n") + 1 - a);
}

// Numeric coordinate field: Gerber omits the decimal point; int_n+frac_n
// come from %FS%. omit_t = trailing-zero omission (T), else leading (L).
double parse_coord_value(const std::string& digits, int int_n, int frac_n, bool omit_t) {
    if (digits.empty()) return 0;
    bool neg = digits[0] == '-';
    std::string d = (neg || digits[0] == '+') ? digits.substr(1) : digits;
    auto dot = d.find('.');
    if (dot != std::string::npos) {
        try {
            return std::stod(digits);  // explicit decimals tolerated
        } catch (...) {
            return 0;
        }
    }
    int total = int_n + frac_n;
    std::string p = d;
    if (omit_t) {
        while (static_cast<int>(p.size()) < total) p += '0';  // left-justify
    } else {
        while (static_cast<int>(p.size()) < total) p = '0' + p;  // right-justify
    }
    if (static_cast<int>(p.size()) > total) p = p.substr(p.size() - total);
    double v = 0;
    try {
        v = std::stod(p);
    } catch (...) {
        v = 0;
    }
    v /= std::pow(10.0, frac_n);
    return neg ? -v : v;
}

std::vector<double> split_nums(const std::string& s) {
    std::vector<double> out;
    std::string cur;
    auto flush = [&]() {
        if (!cur.empty() && cur != "-" && cur != "+" && cur != ".") {
            try {
                out.push_back(std::stod(cur));
            } catch (...) {
            }
        }
        cur.clear();
    };
    for (char c : s) {
        if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+') cur += c;
        else flush();
    }
    flush();
    return out;
}

}  // namespace

class GerberImport {
  public:
    ImportResult run_stream(std::istream& in, const std::string& path) {
        path_ = path;
        detect_layer_from_name(path);
        // Incremental block scan: only one block + one chunk buffered.
        std::string pending;
        bool in_ext = false;  // inside a %...% extended block
        char buf[kStreamChunk];
        bool eof = false;
        while (!eof) {
            in.read(buf, sizeof(buf));
            std::streamsize got = in.gcount();
            eof = got <= 0 || static_cast<std::size_t>(got) < sizeof(buf);
            if (got > 0) pending.append(buf, static_cast<std::size_t>(got));
            // Extract complete blocks.
            while (true) {
                if (pending.empty()) break;
                // Skip line breaks between blocks.
                std::size_t s = 0;
                while (s < pending.size() && (pending[s] == '\n' || pending[s] == '\r'))
                    ++s;
                if (s > 0) pending.erase(0, s);
                if (pending.empty()) break;
                in_ext = !pending.empty() && pending[0] == '%';
                std::size_t term = std::string::npos;
                if (in_ext) {
                    std::size_t star = pending.find('*');
                    if (star != std::string::npos)
                        term = pending.find('%', star + 1);
                } else {
                    term = pending.find('*');
                }
                if (term == std::string::npos) break;  // need more input
                std::string blk = pending.substr(0, term + 1);
                pending.erase(0, term + 1);
                handle_block(blk);
            }
            if (static_cast<std::size_t>(pending.size()) > 4 * kStreamChunk)
                throw BoardError(InputKind::kInvalid,
                                 "gerber: single block over 256KB; file malformed");
        }
        if (!trim(pending).empty()) handle_block(pending);  // trailing M02 w/o '*'
        if (!saw_m02_) warn("no M02/M00 end-of-file marker; parsed anyway");
        return build();
    }

    ImportResult run_text(const std::string& text, const std::string& path) {
        std::istringstream in(text);
        return run_stream(in, path);
    }

  private:
    std::string path_;
    ImportResult result_;
    // Format state.
    bool inch_ = true;  // RS-274X default until %MO%
    bool saw_mo_ = false, saw_fs_ = false;
    int x_int_ = 4, x_frac_ = 6, y_int_ = 4, y_frac_ = 6;
    bool zero_t_ = true;  // true = trailing-zero omission
    bool absolute_ = true;
    // Draw state.
    int cur_ap_ = -1;
    int interp_ = 1;  // 1 linear, 2 cw, 3 ccw
    bool multi_quad_ = false;
    bool dark_ = true;
    bool in_region_ = false;
    bool saw_m02_ = false;
    bool warned_arc_ = false, warned_macro_ = false, warned_sr_ = false;
    bool warned_attr_ = false, warned_hole_ = false, warned_inner_ = false;
    bool warned_noap_ = false;
    bool profile_mode_ = false;
    bool sr_active_ = false;
    int sr_x_ = 1, sr_y_ = 1;
    double sr_dx_ = 0, sr_dy_ = 0;
    std::vector<Op> sr_buf_;
    double cur_x_ = 0, cur_y_ = 0;  // file units
    std::map<int, Aperture> apertures_;
    std::map<std::string, std::string> macros_;  // name -> body (subset honored)
    std::vector<Op> ops_;                        // flashes + draws
    std::vector<std::vector<Op>> regions_;       // region contours (vertex ops)
    std::vector<Op> cur_region_;
    std::vector<Point> profile_pts_;  // file-unit points (1e6 scale ints)
    LayerId copper_layer_ = 0;

    void warn(const std::string& w) { result_.warnings.push_back(w); }

    double to_mm(double v) const { return inch_ ? v * 25.4 : v; }

    void detect_layer_from_name(const std::string& path) {
        std::string l = lower_str(path);
        if (l.find("profile") != std::string::npos || l.find("outline") != std::string::npos ||
            l.find("edgecut") != std::string::npos || l.find("edge_cut") != std::string::npos ||
            ends_with_ci(l, ".gko") || ends_with_ci(l, ".gm1") || ends_with_ci(l, ".gml") ||
            ends_with_ci(l, ".oln")) {
            profile_mode_ = true;
            warn("gerber: profile/outline file detected by name; geometry used as "
                 "board outline, not copper");
        }
        if (l.find("bottom") != std::string::npos || ends_with_ci(l, ".gbl") ||
            ends_with_ci(l, ".bot")) {
            copper_layer_ = 1;
        }
    }

    void handle_block(const std::string& raw) {
        std::string b = trim(raw);
        if (b.empty()) return;
        if (!b.empty() && b[0] == '%') {
            handle_extended(b);
            return;
        }
        if (b.size() >= 3 && b.compare(0, 3, "G04") == 0) {
            if (lower_str(b).find("copperline:profile") != std::string::npos &&
                !profile_mode_) {
                profile_mode_ = true;
                warn("gerber: G04 COPPERLINE:PROFILE marker; geometry used as outline");
            }
            return;
        }
        if (b == "M02*" || b == "M02" || b == "M00*" || b == "M00") {
            saw_m02_ = true;
            return;
        }
        if (b == "G36*" || b == "G36") {
            in_region_ = true;
            cur_region_.clear();
            return;
        }
        if (b == "G37*" || b == "G37") {
            in_region_ = false;
            if (!cur_region_.empty()) {
                if (sr_active_) {
                    for (const auto& o : cur_region_) sr_buf_.push_back(o);
                } else {
                    regions_.push_back(cur_region_);
                }
            }
            cur_region_.clear();
            return;
        }
        if (!b.empty() && b[0] == 'G') {
            if (b.find("G01") != std::string::npos) interp_ = 1;
            if (b.find("G02") != std::string::npos) interp_ = 2;
            if (b.find("G03") != std::string::npos) interp_ = 3;
            if (b.find("G75") != std::string::npos) multi_quad_ = true;
            // A block may carry coordinates too: fall through.
        }
        parse_operation(b);
    }

    void handle_extended(const std::string& b) {
        if (b.compare(0, 3, "%FS") == 0) {
            saw_fs_ = true;
            zero_t_ = (b.find('T') != std::string::npos);
            absolute_ = (b.find('A') != std::string::npos);
            auto px = b.find('X');
            if (px != std::string::npos && px + 2 < b.size() && std::isdigit(b[px + 1]) &&
                std::isdigit(b[px + 2])) {
                x_int_ = b[px + 1] - '0';
                x_frac_ = b[px + 2] - '0';
            }
            auto py = b.find('Y');
            if (py != std::string::npos && py + 2 < b.size() && std::isdigit(b[py + 1]) &&
                std::isdigit(b[py + 2])) {
                y_int_ = b[py + 1] - '0';
                y_frac_ = b[py + 2] - '0';
            }
            return;
        }
        if (b.compare(0, 3, "%MO") == 0) {
            saw_mo_ = true;
            inch_ = (b.find("IN") != std::string::npos);
            return;
        }
        if (b.compare(0, 3, "%AD") == 0) {
            // %ADD10C,0.5*% %ADD11R,1X2*% %ADD12O,..*% %ADD13P,...*% %ADD14NAME*%
            // (find the D-code after the "%AD" prefix, not the D in "AD").
            std::size_t d = b.find('D', 3);
            if (d == std::string::npos) return;
            int code = 0;
            try {
                code = std::stoi(b.substr(d + 1));
            } catch (...) {
                return;
            }
            if (apertures_.size() > kMaxApertures) {
                warn("gerber: aperture cap (65536) exceeded; further apertures ignored");
                return;
            }
            std::size_t cm = b.find(',', d);
            if (cm == std::string::npos) return;
            // Shape letter (or macro name) sits between the D-code and ','.
            std::size_t ks = d + 1;
            while (ks < cm && std::isdigit(b[ks])) ++ks;
            std::string shape = b.substr(ks, cm - ks);
            std::string body = b.substr(cm + 1);
            while (!body.empty() && (body.back() == '*' || body.back() == '%')) body.pop_back();
            if (shape.empty()) return;
            Aperture ap;
            ap.valid = true;
            char kind = shape[0];
            auto nums = split_nums(body);
            if (kind == 'C') {
                ap.kind = 'C';
                double dia = nums.empty() ? 0 : nums[0];
                ap.w_mm = ap.h_mm = to_mm(dia);
                if (nums.size() > 1 && nums[1] > 0 && !warned_hole_) {
                    warned_hole_ = true;
                    warn("gerber: aperture holes ignored (copper extent only)");
                }
            } else if (kind == 'R' || kind == 'O') {
                ap.kind = kind;
                ap.w_mm = to_mm(nums.size() > 0 ? nums[0] : 0);
                ap.h_mm = to_mm(nums.size() > 1 ? nums[1] : (nums.empty() ? 0 : nums[0]));
            } else if (kind == 'P') {
                ap.kind = 'P';
                double dia = nums.empty() ? 0 : nums[0];
                ap.w_mm = ap.h_mm = to_mm(dia);
                warn("gerber: polygon aperture D" + std::to_string(code) +
                     " approximated by bounding box");
            } else {
                // Macro aperture: shape names the macro; honor primitives 1
                // (circle) / 21 (rect) as a bbox, everything else warns +
                // circle-approx.
                std::size_t e = 0;
                while (e < shape.size() &&
                       (std::isalnum(shape[e]) || shape[e] == '_' || shape[e] == '$' ||
                        shape[e] == '.'))
                    ++e;
                auto mit = macros_.find(shape.substr(0, e));
                ap.kind = 'M';
                ap.w_mm = ap.h_mm = to_mm(0.5);
                if (mit != macros_.end()) macro_bbox(mit->second, ap.w_mm, ap.h_mm);
                if (!warned_macro_) {
                    warned_macro_ = true;
                    warn("gerber: macro apertures limited to circle/rect primitives "
                         "(bbox approximation)");
                }
            }
            apertures_[code] = ap;
            return;
        }
        if (b.compare(0, 3, "%AM") == 0) {
            std::string inner = b.substr(3);
            while (!inner.empty() && inner.back() == '%') inner.pop_back();
            std::size_t st = inner.find('*');
            macros_[st == std::string::npos ? inner : inner.substr(0, st)] =
                st == std::string::npos ? "" : inner.substr(st + 1);
            return;
        }
        if (b.compare(0, 3, "%LP") == 0) {
            dark_ = (b.find('D') != std::string::npos);
            return;
        }
        if (b.compare(0, 3, "%SR") == 0) {
            if (b == "%SR*%") {
                sr_active_ = false;
                replicate_sr();
                sr_buf_.clear();
                return;
            }
            auto get = [&](char k) -> double {
                auto p = b.find(k);
                if (p == std::string::npos) return 0;
                try {
                    return std::stod(b.substr(p + 1));
                } catch (...) {
                    return 0;
                }
            };
            sr_x_ = std::max(1, static_cast<int>(get('X')));
            sr_y_ = std::max(1, static_cast<int>(get('Y')));
            sr_dx_ = get('I');
            sr_dy_ = get('J');
            if (static_cast<long long>(sr_x_) * sr_y_ > static_cast<long long>(kMaxSrRepeat)) {
                if (!warned_sr_) {
                    warned_sr_ = true;
                    warn("gerber: step-repeat over cap (10000); clamped to 100x100");
                }
                sr_x_ = std::min(sr_x_, 100);
                sr_y_ = std::min(sr_y_, 100);
            }
            sr_active_ = true;
            sr_buf_.clear();
            return;
        }
        if (b.compare(0, 3, "%TF") == 0) {
            std::string l = lower_str(b);
            if (l.find("profile") != std::string::npos && !profile_mode_) {
                profile_mode_ = true;
                warn("gerber: FileFunction Profile; geometry used as outline, not copper");
            } else if (l.find("soldermask") != std::string::npos ||
                       l.find("paste") != std::string::npos ||
                       l.find("silk") != std::string::npos ||
                       l.find("legend") != std::string::npos ||
                       l.find("mask") != std::string::npos) {
                if (!warned_attr_) {
                    warned_attr_ = true;
                    warn("gerber: non-copper FileFunction parsed as copper (no "
                         "soldermask/paste semantics)");
                }
            }
            auto cp = l.find("copper,l");
            if (cp != std::string::npos) {
                int lay = 0;
                try {
                    lay = std::stoi(l.substr(cp + 8));
                } catch (...) {
                    lay = 0;
                }
                // 2-layer board model: L1 -> Top, L2+ -> Bottom.
                copper_layer_ = (lay <= 1) ? 0 : 1;
                if (lay > 2 && !warned_inner_) {
                    warned_inner_ = true;
                    warn("gerber: inner-layer FileFunction collapsed onto Bottom "
                         "(2-layer board model)");
                }
            }
            return;
        }
        if (!warned_attr_) {
            warned_attr_ = true;
            warn("gerber: attribute/load commands beyond FileFunction/Profile are not "
                 "modeled (parsed for copper only)");
        }
    }

    // Bounding box (mm) of macro primitives 1 (circle) and 21 (rect);
    // any other primitive folds into a circle of its first parameter.
    static void macro_bbox(const std::string& body, double& w, double& h) {
        w = h = 0;
        std::string cur;
        auto flush = [&](const std::string& prim) {
            if (prim.empty()) return;
            auto nums = split_nums(prim);
            if (nums.empty()) return;
            int code = static_cast<int>(nums[0]);
            if (code == 1 && nums.size() >= 4) {
                double dia = nums[2], cx = nums[3], cy = nums.size() > 4 ? nums[4] : 0;
                w = std::max(w, std::fabs(cx) + dia / 2);
                h = std::max(h, std::fabs(cy) + dia / 2);
            } else if (code == 21 && nums.size() >= 5) {
                double pw = nums[2], ph = nums[3], cx = nums[4],
                       cy = nums.size() > 5 ? nums[5] : 0;
                w = std::max(w, std::fabs(cx) + pw / 2);
                h = std::max(h, std::fabs(cy) + ph / 2);
            } else if (nums.size() > 1) {
                w = std::max(w, nums[1]);
                h = std::max(h, nums[1]);
            }
        };
        for (char c : body) {
            if (c == '*') {
                flush(cur);
                cur.clear();
            } else {
                cur += c;
            }
        }
        flush(cur);
        w *= 2;
        h *= 2;
        if (w <= 0) w = 0.5;
        if (h <= 0) h = 0.5;
    }

    void replicate_sr() {
        std::vector<Op> base = sr_buf_;
        for (int ix = 0; ix < sr_x_; ++ix) {
            for (int iy = 0; iy < sr_y_; ++iy) {
                for (auto o : base) {
                    double dx = ix * sr_dx_, dy = iy * sr_dy_;
                    o.x1 += dx;
                    o.y1 += dy;
                    o.x2 += dx;
                    o.y2 += dy;
                    emit_op(o);
                }
            }
        }
    }

    void emit_op(const Op& o) {
        if (o.kind == 'R' || o.kind == 'M') {
            cur_region_.push_back(o);
            if (cur_region_.size() > kMaxRegionVertices) {
                warn("gerber: region over vertex cap (32768); truncated with warning");
                in_region_ = false;
                regions_.push_back(cur_region_);
                cur_region_.clear();
            }
            return;
        }
        if (profile_mode_) {
            if (o.kind == 'F') profile_pts_.push_back(file_pt(o.x1, o.y1));
            else profile_pts_.push_back(file_pt(o.x2, o.y2));
            return;
        }
        if (ops_.size() > kMaxGerberPrimitives)
            throw BoardError(InputKind::kInvalid,
                             "gerber: primitive cap (2000000) exceeded; file too large");
        ops_.push_back(o);
    }

    static Point file_pt(double x, double y) {
        return {static_cast<Coord>(std::llround(x * 1e6)),
                static_cast<Coord>(std::llround(y * 1e6))};
    }

    // Parse a data block: optional X/Y/I/J + D-code.
    void parse_operation(const std::string& b) {
        std::string s = b;
        if (!s.empty() && s.back() == '*') s.pop_back();
        bool has_xy = (s.find('X') != std::string::npos) || (s.find('Y') != std::string::npos);
        bool has_ij = (s.find('I') != std::string::npos) || (s.find('J') != std::string::npos);
        int dcode = -1;
        auto dpos = s.find('D');
        if (dpos != std::string::npos) {
            try {
                dcode = std::stoi(s.substr(dpos + 1));
            } catch (...) {
                dcode = -1;
            }
        }
        if (!has_xy && !has_ij) {
            if (dcode >= 10) cur_ap_ = dcode;  // aperture select
            return;
        }
        auto field = [&](char k, int in, int fr, double cur) -> double {
            auto p = s.find(k);
            if (p == std::string::npos) return cur;
            std::size_t e = p + 1;
            if (e < s.size() && (s[e] == '+' || s[e] == '-')) ++e;
            while (e < s.size() && (std::isdigit(s[e]) || s[e] == '.')) ++e;
            double v = parse_coord_value(s.substr(p + 1, e - p - 1), in, fr, zero_t_);
            return absolute_ ? v : cur + v;
        };
        double nx = field('X', x_int_, x_frac_, cur_x_);
        double ny = field('Y', y_int_, y_frac_, cur_y_);
        int op = (dcode >= 1 && dcode <= 3) ? dcode : 1;  // modal D01
        if (op == 3 && has_ij && !has_xy) op = 1;  // I/J without X/Y is a draw, not flash
        if (op == 2) {
            cur_x_ = nx;
            cur_y_ = ny;
            if (in_region_) {
                Op o;
                o.kind = 'M';
                o.x1 = nx;
                o.y1 = ny;
                o.dark = dark_;
                if (sr_active_) sr_buf_.push_back(o);
                else emit_op(o);
            } else if (profile_mode_) {
                profile_pts_.push_back(file_pt(nx, ny));
            }
            return;
        }
        if (op == 3) {
            Op o;
            o.kind = 'F';
            o.x1 = nx;
            o.y1 = ny;
            o.ap = cur_ap_;
            o.dark = dark_;
            cur_x_ = nx;
            cur_y_ = ny;
            if (in_region_) {
                // Flash inside a region contour: close the contour, keep the
                // flash as a pad (warned, rare in practice).
                if (!cur_region_.empty()) {
                    regions_.push_back(cur_region_);
                    cur_region_.clear();
                }
                warn("gerber: flash inside region contour treated as pad, contour split");
            }
            if (sr_active_) sr_buf_.push_back(o);
            else emit_op(o);
            return;
        }
        // D01 draw (linear, or arc when G02/G03 modal).
        if ((interp_ == 2 || interp_ == 3) && has_ij) {
            double ioff = field('I', x_int_, x_frac_, 0);
            double joff = field('J', y_int_, y_frac_, 0);
            linearize_arc(cur_x_, cur_y_, nx, ny, ioff, joff, interp_ == 2);
            cur_x_ = nx;
            cur_y_ = ny;
            return;
        }
        Op o;
        o.kind = 'D';
        o.x1 = cur_x_;
        o.y1 = cur_y_;
        o.x2 = nx;
        o.y2 = ny;
        o.ap = cur_ap_;
        o.dark = dark_;
        cur_x_ = nx;
        cur_y_ = ny;
        if (in_region_) {
            Op v;
            v.kind = 'R';
            v.x1 = nx;
            v.y1 = ny;
            v.dark = dark_;
            v.ap = cur_ap_;
            if (sr_active_) sr_buf_.push_back(v);
            else emit_op(v);
            return;
        }
        if (sr_active_) sr_buf_.push_back(o);
        else emit_op(o);
    }

    // Linearize a circular-arc draw into straight draw ops (<= 256).
    void linearize_arc(double x0, double y0, double x1, double y1, double ioff, double joff,
                       bool cw) {
        if (!warned_arc_) {
            warned_arc_ = true;
            warn("gerber: arc draws (G02/G03) linearized into chords (<= 256)");
        }
        double cx = x0 + ioff, cy = y0 + joff;  // I/J are always start-relative
        double r = std::hypot(x0 - cx, y0 - cy);
        auto emit_draw = [&](double ax, double ay, double bx, double by) {
            Op o;
            o.kind = 'D';
            o.x1 = ax;
            o.y1 = ay;
            o.x2 = bx;
            o.y2 = by;
            o.ap = cur_ap_;
            o.dark = dark_;
            if (in_region_) {
                Op v;
                v.kind = 'R';
                v.x1 = bx;
                v.y1 = by;
                v.dark = dark_;
                v.ap = cur_ap_;
                if (sr_active_) sr_buf_.push_back(v);
                else emit_op(v);
            } else if (sr_active_) {
                sr_buf_.push_back(o);
            } else {
                emit_op(o);
            }
        };
        if (!(r > 0)) {
            emit_draw(x0, y0, x1, y1);
            return;
        }
        double a0 = std::atan2(y0 - cy, x0 - cx), a1 = std::atan2(y1 - cy, x1 - cx);
        double sweep = 0;
        if (cw) {
            sweep = a0 - a1;
            while (sweep <= 0) sweep += 2 * std::numbers::pi;
        } else {
            sweep = a1 - a0;
            while (sweep <= 0) sweep += 2 * std::numbers::pi;
        }
        int n = std::max(2, std::min(256, static_cast<int>(std::ceil(sweep / 0.05)) + 1));
        double px = x0, py = y0;
        for (int i = 1; i <= n; ++i) {
            double a = cw ? (a0 - sweep * i / n) : (a0 + sweep * i / n);
            double qx = cx + r * std::cos(a), qy = cy + r * std::sin(a);
            if (i == n) {
                qx = x1;
                qy = y1;
            }
            emit_draw(px, py, qx, qy);
            px = qx;
            py = qy;
        }
    }

    ImportResult build() {
        ImportResult& out = result_;
        Board& b = out.board;
        b.source_format = "gerber";
        b.source_file = path_;
        if (!saw_fs_) warn("gerber: no %FS% format block; assumed X4.6/Y4.6 trailing-zero");
        if (!saw_mo_) warn("gerber: no %MO% units block; assumed inches (RS-274X default)");

        b.layers.push_back({0, "Top", false, 1.0});
        b.layers.push_back({1, "Bottom", true, 1.0});

        NetInfo net;
        net.id = 0;
        net.name = "COPPER";
        b.nets.push_back(net);

        double ox = 0, oy = 0, ex = 0, ey = 0;
        bool have_box = false;
        auto note = [&](double x, double y) {
            if (!have_box) {
                ox = ex = x;
                oy = ey = y;
                have_box = true;
            } else {
                ox = std::min(ox, x);
                oy = std::min(oy, y);
                ex = std::max(ex, x);
                ey = std::max(ey, y);
            }
        };
        if (profile_mode_) {
            for (const auto& p : profile_pts_) note(p.x / 1e6, p.y / 1e6);
            for (const auto& r : regions_)
                for (const auto& v : r) note(v.x1, v.y1);
            if (!have_box) throw BoardError(InputKind::kInvalid, "gerber: empty profile file");
            b.width_nm = mm_to_nm(to_mm(ex - ox));
            b.height_nm = mm_to_nm(to_mm(ey - oy));
            if (b.width_nm <= 0 || b.height_nm <= 0)
                throw BoardError(InputKind::kInvalid, "gerber: degenerate profile outline");
            warn("gerber: profile mode: geometry used as outline only (no copper imported)");
            return std::move(out);
        }
        // Copper bbox (flashes, draws incl. aperture extent, regions).
        auto ap_ext = [&](int ap) -> double {
            auto it = apertures_.find(ap);
            if (it != apertures_.end() && it->second.valid)
                return std::max(it->second.w_mm, it->second.h_mm) / 2;
            return 0;
        };
        for (const auto& o : ops_) {
            double ext_mm = ap_ext(o.ap);
            double ext_fu = ext_mm / (inch_ ? 25.4 : 1.0);
            if (o.kind == 'F') {
                note(o.x1 - ext_fu, o.y1 - ext_fu);
                note(o.x1 + ext_fu, o.y1 + ext_fu);
            } else {
                note(std::min(o.x1, o.x2) - ext_fu, std::min(o.y1, o.y2) - ext_fu);
                note(std::max(o.x1, o.x2) + ext_fu, std::max(o.y1, o.y2) + ext_fu);
            }
        }
        for (const auto& r : regions_)
            for (const auto& v : r) note(v.x1, v.y1);
        if (!have_box) throw BoardError(InputKind::kInvalid, "gerber: no copper geometry found");
        double margin_fu = 1.0 / (inch_ ? 25.4 : 1.0);  // 1mm margin
        ox -= margin_fu;
        oy -= margin_fu;
        ex += margin_fu;
        ey += margin_fu;
        warn("gerber: no profile layer; outline = copper bbox + 1mm margin");
        b.width_nm = mm_to_nm(to_mm(ex - ox));
        b.height_nm = mm_to_nm(to_mm(ey - oy));
        double origin_x = to_mm(ox), origin_y = to_mm(oy);
        auto px = [&](double v) { return mm_to_nm(to_mm(v) - origin_x); };
        auto py = [&](double v) { return mm_to_nm(to_mm(v) - origin_y); };

        TermId next_term = 0;
        int plane_seq = 0;
        auto aperture_size = [&](int ap, double& w, double& h) {
            w = h = 0.2;
            auto it = apertures_.find(ap);
            if (it != apertures_.end() && it->second.valid) {
                w = it->second.w_mm;
                h = it->second.h_mm;
            } else if (ap >= 10 && !warned_noap_) {
                warned_noap_ = true;
                warn("gerber: aperture used without definition; assumed 0.2mm");
            }
        };
        for (const auto& o : ops_) {
            double aw = 0.2, ah = 0.2;
            aperture_size(o.ap, aw, ah);
            if (o.kind == 'F') {
                if (!o.dark) {
                    Keepout k;
                    Coord w = mm_to_nm(aw), h = mm_to_nm(ah);
                    k.rect = {px(o.x1) - w / 2, py(o.y1) - h / 2, px(o.x1) + w / 2,
                              py(o.y1) + h / 2};
                    k.layer = copper_layer_;
                    k.reason = "gerber-clear-flash";
                    b.keepouts.push_back(k);
                    continue;
                }
                Terminal t;
                t.id = next_term++;
                t.net = 0;
                t.pos = {px(o.x1), py(o.y1)};
                t.layer = copper_layer_;
                t.pad_w_nm = mm_to_nm(aw);
                t.pad_h_nm = mm_to_nm(ah);
                t.component = "G" + std::to_string(t.id);
                t.pin = "1";
                b.terminals.push_back(t);
                b.nets[0].terminals.push_back(t.id);
                continue;
            }
            Coord w = mm_to_nm(std::min(aw, ah));
            if (w <= 0) w = mm_to_nm(0.2);
            if (!o.dark) {
                Keepout k;
                k.rect = {std::min(px(o.x1), px(o.x2)) - w / 2,
                          std::min(py(o.y1), py(o.y2)) - w / 2,
                          std::max(px(o.x1), px(o.x2)) + w / 2,
                          std::max(py(o.y1), py(o.y2)) + w / 2};
                k.layer = copper_layer_;
                k.reason = "gerber-clear-draw";
                b.keepouts.push_back(k);
                continue;
            }
            if (px(o.x1) == px(o.x2) && py(o.y1) == py(o.y2)) {
                warn("gerber: zero-length draw treated as pad");
                Terminal t;
                t.id = next_term++;
                t.net = 0;
                t.pos = {px(o.x1), py(o.y1)};
                t.layer = copper_layer_;
                t.pad_w_nm = w;
                t.pad_h_nm = w;
                t.component = "G" + std::to_string(t.id);
                t.pin = "1";
                b.terminals.push_back(t);
                b.nets[0].terminals.push_back(t.id);
                continue;
            }
            TraceSeg s;
            s.net = 0;
            s.layer = copper_layer_;
            s.a = {px(o.x1), py(o.y1)};
            s.b = {px(o.x2), py(o.y2)};
            s.width_nm = w;
            b.traces.push_back(s);
        }
        for (const auto& r : regions_) {
            std::vector<Point> poly;
            bool dark = true;
            for (const auto& v : r) {
                if (v.kind == 'M' && !poly.empty()) break;  // multi-contour: first wins
                dark = v.dark;
                poly.push_back({px(v.x1), py(v.y1)});
            }
            // Drop an explicit closing vertex duplicating the start.
            if (poly.size() >= 2 && poly.back() == poly.front()) poly.pop_back();
            if (poly.size() < 3) {
                warn("gerber: degenerate region skipped with warning");
                continue;
            }
            if (!dark) {
                Coord x1 = poly[0].x, y1 = poly[0].y, x2 = poly[0].x, y2 = poly[0].y;
                for (const auto& p : poly) {
                    x1 = std::min(x1, p.x);
                    y1 = std::min(y1, p.y);
                    x2 = std::max(x2, p.x);
                    y2 = std::max(y2, p.y);
                }
                Keepout k;
                k.rect = {x1, y1, x2, y2};
                k.layer = copper_layer_;
                k.reason = "gerber-clear-region";
                b.keepouts.push_back(k);
                warn("gerber: clear (LPC) region approximated by bounding-box keepout");
                continue;
            }
            PlaneZone z;
            z.id = plane_seq;
            z.net = 0;
            z.layer = copper_layer_;
            z.poly = std::move(poly);
            z.island = plane_seq++;
            z.routable = true;
            b.planes.push_back(std::move(z));
        }
        if (b.terminals.empty() && b.traces.empty() && b.planes.empty())
            throw BoardError(InputKind::kInvalid, "gerber: no dark copper geometry found");
        warn("gerber: no embedded netlist; all copper is single-net (COPPER) + obstacles");
        return std::move(out);
    }
};

bool GerberImporter::claims(const std::string& path, const std::string& head_bytes) const {
    std::string l = lower_str(path);
    bool ext = ends_with_ci(l, ".gbr") || ends_with_ci(l, ".ger") || ends_with_ci(l, ".pho") ||
               ends_with_ci(l, ".gtl") || ends_with_ci(l, ".gbl") || ends_with_ci(l, ".gko") ||
               ends_with_ci(l, ".gm1") || ends_with_ci(l, ".gml") || ends_with_ci(l, ".gtp") ||
               ends_with_ci(l, ".gbp") || ends_with_ci(l, ".gts") || ends_with_ci(l, ".gbs") ||
               ends_with_ci(l, ".top") || ends_with_ci(l, ".bot") || ends_with_ci(l, ".oln");
    bool marker = head_bytes.find("%FS") != std::string::npos ||
                  head_bytes.find("%MO") != std::string::npos ||
                  head_bytes.find("%AD") != std::string::npos;
    if (ext) return marker || head_bytes.find("D03") != std::string::npos;
    return marker;
}

ImportResult GerberImporter::import_file(const std::string& path) const {
    std::ifstream f(path, std::ios::binary);
    if (!f) throw BoardError(InputKind::kInvalid, "cannot open file: " + path);
    return GerberImport().run_stream(f, path);
}

ImportResult GerberImporter::import_text(const std::string& text, const std::string& path) const {
    std::istringstream in(text);
    return GerberImport().run_stream(in, path);
}

const BoardImporter& gerber_importer_singleton() {
    static const GerberImporter kInstance;
    return kInstance;
}

}  // namespace copperline
