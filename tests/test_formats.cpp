// Format-completion tests: DSN/KiCad copper pours, KiCad export
// round-trip, Gerber RS-274X import, IPC-2581C import. Every new format
// honors determinism, integer-nm geometry and never-silent warnings.
#include "helpers.h"

#include "router/board.h"
#include "router/dsn.h"
#include "router/engine.h"
#include "router/gerber.h"
#include "router/ipc2581.h"
#include "router/verifier.h"

#include <set>

using namespace copperline;
using namespace copperline::test;

#ifndef FIXTURE_DIR
#define FIXTURE_DIR "fixtures"
#endif

namespace {

std::string fixture(const std::string& name) { return std::string(FIXTURE_DIR) + "/" + name; }

bool has_warning(const std::vector<std::string>& w, const std::string& needle) {
    for (const auto& s : w)
        if (s.find(needle) != std::string::npos) return true;
    return false;
}

struct TermKey {
    Point pos{};
    LayerId layer = 0;
    Coord w = 0, h = 0;
    NetId net = -1;
    bool operator<(const TermKey& o) const {
        if (pos < o.pos || o.pos < pos) return pos < o.pos;
        if (layer != o.layer) return layer < o.layer;
        if (w != o.w) return w < o.w;
        if (h != o.h) return h < o.h;
        return net < o.net;
    }
};

}  // namespace

// ---- issue: DSN (resolution <unit> <digits>) scaling -----------------------
// One file count = unit_size * 10^-digits (docs/reverse DSN dialect notes).
// um-10 => 1e-13 mm per count => 1 count per 0.0001 nm, so a 10 mm pin span is
// written as 1e14 counts and must land on exactly 10,000,000 nm.
CT_TEST(dsn_resolution_scaling) {
    static const char* kDsn =
        "(pcb \"res_demo\"\n"
        "  (parser (string_quote \") (space_in_quoted_tokens on))\n"
        "  (resolution um 10)\n"
        "  (unit mm)\n"                      // both present: resolution wins
        "  (structure\n"
        "    (layer Top (type signal))\n"
        "    (layer Bottom (type signal))\n"
        "    (boundary (rect pcb 0 0 200000000000000 200000000000000))\n"  // 20 mm
        "    (via STD 6000000000000 3000000000000))\n"
        "  (placement\n"
        "    (component U1 (place P 0 100000000000000 front 0))\n"          // 10 mm
        "    (component U2 (place P 100000000000000 100000000000000 front 0)))\n"
        "  (library (image P (pin 1 0 0)))\n"
        "  (network (net N1 (pins U1-1 U2-1))))\n";
    DsnImporter dsn;
    ImportResult ir = dsn.import_text(kDsn, "res_um10.dsn");
    CT_CHECK(ir.board.width_nm == mm_to_nm(20.0));
    CT_CHECK(ir.board.height_nm == mm_to_nm(20.0));
    CT_CHECK(ir.board.terminals.size() == 2);
    Coord span = std::llabs(ir.board.terminals[0].pos.x - ir.board.terminals[1].pos.x);
    CT_CHECK(span == mm_to_nm(10.0));  // 10,000,000 nm
    CT_CHECK(has_warning(ir.warnings, "resolution scale"));

    // Resolution decade that cannot describe the file's own coordinates falls
    // back to the unit scale with an explicit warning (never a silent
    // sub-nanometre board).
    static const char* kFallback =
        "(pcb \"res_fallback\"\n"
        "  (resolution um 10)\n"
        "  (structure\n"
        "    (layer Top (type signal))\n"
        "    (layer Bottom (type signal))\n"
        "    (boundary (rect 0 0 20 20)))\n"
        "  (placement (component U1 (place P 1 1 front 0))\n"
        "             (component U2 (place P 19 1 front 0)))\n"
        "  (library (image P (pin 1 0 0)))\n"
        "  (network (net N1 (pins U1-1 U2-1))))\n";
    ImportResult fb = dsn.import_text(kFallback, "res_fallback.dsn");
    CT_CHECK(fb.board.width_nm == mm_to_nm(20.0));
    CT_CHECK(has_warning(fb.warnings, "used the unit scale"));
}

// Padstack shapes: `(padstack <id> (shape (rect|circle|octagon|polygon ...)))`
// drives Terminal pad extents instead of the fixed 0.3 mm square, so the
// verifier sees the copper the file declares.
CT_TEST(dsn_padstack_shapes) {
    static const char* kDsn =
        "(pcb \"pads\" (unit mm)\n"
        "  (structure (layer Top (type signal)) (layer Bottom (type signal))\n"
        "    (boundary (rect 0 0 20 20)))\n"
        "  (placement (component U1 (place PKG 2 2 front 0))\n"
        "             (component U2 (place PKG 16 2 front 90)))\n"
        "  (library\n"
        "    (image PKG\n"
        "      (pin 1 SMD 0 0)\n"
        "      (pin 2 SMD 0 6)\n"
        "      (padstack SMD (shape (rect Top -0.6 -0.4 0.6 0.4)\n"
        "                           (circle Bottom 0.5)))))\n"
        "  (network (net N1 (pins U1-1 U2-1)) (net N2 (pins U1-2 U2-2))))\n";
    DsnImporter dsn;
    ImportResult ir = dsn.import_text(kDsn, "padstacks.dsn");
    CT_CHECK(ir.board.terminals.size() == 4);
    const Terminal* p = nullptr;
    for (const auto& t : ir.board.terminals)
        if (t.component == "U1" && t.pin == "1") p = &t;
    CT_CHECK(p != nullptr);
    CT_CHECK(p->pad_w_nm == mm_to_nm(1.2));  // rect -0.6..0.6
    CT_CHECK(p->pad_h_nm == mm_to_nm(0.8));  // rect -0.4..0.4
    CT_CHECK(p->pos == Point({mm_to_nm(2), mm_to_nm(2)}));  // symmetric shape
    // The verifier must now flag a foreign trace that clears a 0.3 mm square
    // pad (0.25 mm gap) but penetrates the declared 1.2 mm rect copper.
    TraceSeg s;
    s.net = ir.board.find_net_by_name("N2")->id;
    s.layer = 0;
    s.a = {mm_to_nm(2.4), mm_to_nm(0.5)};
    s.b = {mm_to_nm(2.4), mm_to_nm(3.5)};
    s.width_nm = mm_to_nm(0.2);
    ir.board.traces.push_back(s);
    RuleResolver res = RuleResolver::defaults_for(ir.board);
    BoardVerifier v;
    VerifyResult vr = v.verify(ir.board, res, res.defaultContext());
    CT_CHECK(!vr.legal);
}

// Octagon / polygon padstacks are approximated by their bounding box, with an
// explicit warning (never silent, never fatal).
CT_TEST(dsn_padstack_approximation_warns) {
    static const char* kDsn =
        "(pcb \"pads_approx\" (unit mm)\n"
        "  (structure (layer Top (type signal)) (layer Bottom (type signal))\n"
        "    (boundary (rect 0 0 20 20)))\n"
        "  (placement (component U1 (place PKG 2 2 front 0))\n"
        "             (component U2 (place PKG 16 2 front 0)))\n"
        "  (library\n"
        "    (image PKG\n"
        "      (pin 1 OCT 0 0)\n"
        "      (pin 2 POLY 0 6)\n"
        "      (padstack OCT (shape (octagon Top 1.0)))\n"
        "      (padstack POLY (shape (polygon Top 0.1 -0.5 -0.5 0.5 -0.5 0.5 0.5)))))\n"
        "  (network (net N1 (pins U1-1 U2-1)) (net N2 (pins U1-2 U2-2))))\n";
    DsnImporter dsn;
    ImportResult ir = dsn.import_text(kDsn, "pads_approx.dsn");
    CT_CHECK(has_warning(ir.warnings, "(octagon ...)"));
    CT_CHECK(has_warning(ir.warnings, "(polygon ...)"));
    for (const auto& t : ir.board.terminals)
        CT_CHECK(t.pad_w_nm == mm_to_nm(1.0));
}

// Quoted names: Altium writes `(layer "Top Layer")` while references may use
// `TopLayer` or `TOP`. Layer/padstack identifiers compare separator- and
// case-insensitively, quoted tokens (including their spaces) survive, and net
// names stay verbatim so nothing is silently merged.
CT_TEST(dsn_quoted_name_normalization) {
    static const char* kDsn =
        "(pcb \"quoted\"\n"
        "  (parser (string_quote \") (space_in_quoted_tokens on))\n"
        "  (unit mm)\n"
        "  (structure\n"
        "    (layer \"Top Layer\" (type signal))\n"
        "    (layer \"Bottom Layer\" (type signal))\n"
        "    (boundary (rect pcb 0 0 20 20))\n"
        "    (copper_pour PWR (polygon bottomlayer 1 8 19 8 19 12 1 12))\n"
        "    (via_keepout (rect toplayer 5 5 6 6)))\n"
        "  (placement (component U1 (place \"SO 8\" 2 10 front 0))\n"
        "             (component U2 (place \"SO 8\" 18 10 front 0)))\n"
        "  (library (image \"SO 8\" (pin 1 0 0) (pin 2 0 0)))\n"
        "  (network (class \"SIG CLASS\" (rule (width 0.2) (clearance 0.15)))\n"
        "           (net \"SIG A\" (pins U1-1 U2-1) (class \"SIG CLASS\"))\n"
        "           (net PWR (pins U1-2 U2-2) (class \"SIG CLASS\"))))\n";
    DsnImporter dsn;
    ImportResult ir = dsn.import_text(kDsn, "quoted.dsn");
    CT_CHECK(ir.board.layers.size() == 2);
    CT_CHECK(ir.board.layers[0].name == "Top Layer");  // verbatim name kept
    // Pour resolved `bottomlayer` -> "Bottom Layer"; keepout resolved
    // `toplayer` -> "Top Layer".
    CT_CHECK(ir.board.planes.size() == 1);
    CT_CHECK(ir.board.planes[0].layer == ir.board.layers[1].id);
    CT_CHECK(ir.board.keepouts.size() == 1);
    CT_CHECK(ir.board.keepouts[0].layer == ir.board.layers[0].id);
    // Quoted net names keep their spaces (no normalization for nets).
    CT_CHECK(ir.board.find_net_by_name("SIG A") != nullptr);
    CT_CHECK(ir.board.find_net_by_name("SIGA") == nullptr);
    // Pin/pad names with spaces still resolve through the quoted image name.
    CT_CHECK(ir.board.terminals.size() == 4);
}

// (boundary (polygon pcb ...)): a real outline becomes the board extents. The
// rect board model can only keep the bounding box, so a non-rectangular
// outline must say so. A polygon that *is* a rectangle is exact and stays
// silent.
CT_TEST(dsn_polygon_boundary) {
    static const char* kPentagon =
        "(pcb \"poly_demo\"\n"
        "  (unit mm)\n"
        "  (structure\n"
        "    (layer Top (type signal))\n"
        "    (layer Bottom (type signal))\n"
        "    (boundary (polygon pcb 0 0 20 0 20 10 10 14 0 10)))\n"
        "  (placement (component U1 (place SO8 0 0 front 0))\n"
        "             (component U2 (place SO8 18 4 front 0)))\n"
        "  (library (image SO8 (pin 1 0 0)))\n"
        "  (network (net N (pins U1-1 U2-1))))\n";
    DsnImporter dsn;
    ImportResult ir = dsn.import_text(kPentagon, "pentagon.dsn");
    // bbox of the pentagon: 20 mm x 14 mm.
    CT_CHECK(ir.board.width_nm == mm_to_nm(20.0));
    CT_CHECK(ir.board.height_nm == mm_to_nm(14.0));
    CT_CHECK(has_warning(ir.warnings, "non-rectangular board outline approximated by its bounding box"));
    // A pad placed on the polygon's min corner lands on the board origin.
    CT_CHECK(ir.board.terminals.size() == 2);
    CT_CHECK(ir.board.terminals[0].pos.x == 0);
    CT_CHECK(ir.board.terminals[0].pos.y == 0);
    CT_CHECK(ir.board.terminals[1].pos.x == mm_to_nm(18.0));

    // A four-vertex polygon that is axis-aligned is an exact rectangle.
    static const char* kRectPoly =
        "(pcb \"rect_poly\"\n"
        "  (unit mm)\n"
        "  (structure\n"
        "    (layer Top (type signal))\n"
        "    (boundary (polygon pcb 0 0 30 0 30 12 0 12)))\n"
        "  (placement (component U1 (place SO8 0 0 front 0))\n"
        "             (component U2 (place SO8 10 6 front 0)))\n"
        "  (library (image SO8 (pin 1 0 0)))\n"
        "  (network (net N (pins U1-1 U2-1))))\n";
    ImportResult ir2 = dsn.import_text(kRectPoly, "rect_poly.dsn");
    CT_CHECK(ir2.board.width_nm == mm_to_nm(30.0));
    CT_CHECK(ir2.board.height_nm == mm_to_nm(12.0));
    CT_CHECK(!has_warning(ir2.warnings, "non-rectangular board outline"));
}

// ---- DSN keepout kinds ------------------------------------------------
// (wire|via|place|bend|elongate)_keepout name WHAT is forbidden inside the
// region; the imported Keepout must carry that enforcement kind so a via
// keepout is only a via wall (wires stay legal) and vice versa. The generic
// (keepout ...) blocks everything. Polygons become bounding boxes with an
// explicit warning (never silent).
CT_TEST(dsn_keepout_kinds) {
    static const char* kDsn =
        "(pcb \"keepout_kinds\"\n"
        "  (unit mm)\n"
        "  (structure\n"
        "    (layer Top (type signal))\n"
        "    (layer Bottom (type signal))\n"
        "    (boundary (rect 0 0 20 20))\n"
        "    (wire_keepout (rect Top 1 1 2 2))\n"
        "    (via_keepout (rect Top 5 5 6 6))\n"
        "    (place_keepout (rect Top 10 10 11 11))\n"
        "    (bend_keepout (polygon Top 0.1 14 14 16 14 16 16 14 16))\n"
        "    (elongate_keepout (rect Bottom 1 14 2 16))\n"
        "    (keepout (rect Bottom 18 18 19 19)))\n"
        "  (placement (component U1 (place P 2 8 front 0))\n"
        "             (component U2 (place P 18 8 front 0)))\n"
        "  (library (image P (pin 1 0 0)))\n"
        "  (network (net N (pins U1-1 U2-1))))\n";
    DsnImporter dsn;
    ImportResult ir = dsn.import_text(kDsn, "keepout_kinds.dsn");
    CT_CHECK(ir.board.keepouts.size() == 6);
    // Document order is preserved (import order == declaration order).
    CT_CHECK(ir.board.keepouts[0].kind == KeepoutKind::kWire);
    CT_CHECK(ir.board.keepouts[1].kind == KeepoutKind::kVia);
    CT_CHECK(ir.board.keepouts[2].kind == KeepoutKind::kPlace);
    CT_CHECK(ir.board.keepouts[3].kind == KeepoutKind::kBend);
    CT_CHECK(ir.board.keepouts[4].kind == KeepoutKind::kElongate);
    CT_CHECK(ir.board.keepouts[5].kind == KeepoutKind::kAll);
    CT_CHECK(has_warning(ir.warnings, "keepout polygons approximated by bounding box"));
}

// Enforcement is per kind: a via_keepout flags vias but not wires, a
// wire_keepout flags wires but not vias, a place_keepout flags pads only,
// and the generic keepout blocks everything (traces + vias + pads).
CT_TEST(keepout_kind_enforcement) {
    Board b = base_2layer();
    NetInfo n0 = make_net(0, "N0");
    b.nets.push_back(n0);
    NetInfo n1 = make_net(1, "N1");
    b.nets.push_back(n1);
    reset_term_ids();
    add_terminal(b, 0, 2.0, 2.0, 0);
    add_terminal(b, 0, 18.0, 2.0, 0);
    add_terminal(b, 1, 2.0, 16.0, 0);
    add_terminal(b, 1, 18.0, 16.0, 0);

    // One region of every kind, all the same 1x1 mm square at (10,10).
    auto add_ko = [&](KeepoutKind k, const char* reason) {
        Keepout ko;
        ko.rect = {mm_to_nm(10), mm_to_nm(10), mm_to_nm(11), mm_to_nm(11)};
        ko.layer = 0;
        ko.kind = k;
        ko.reason = reason;
        b.keepouts.push_back(ko);
    };
    // Generic first: blocks every object type.
    add_ko(KeepoutKind::kAll, "all");
    // Wire keepout: a trace through it is illegal, a via through it is not.
    b.keepouts.push_back({mm_to_nm(5), mm_to_nm(5), mm_to_nm(6), mm_to_nm(6), 0,
                          "wire", KeepoutKind::kWire});
    // Via keepout: vias illegal, wires legal.
    b.keepouts.push_back({mm_to_nm(14), mm_to_nm(5), mm_to_nm(15), mm_to_nm(6), 0,
                          "via", KeepoutKind::kVia});
    RuleResolver r = RuleResolver::defaults_for(b);
    BoardVerifier v;

    // Trace through the generic keepout: violation.
    {
        Board t = b;
        TraceSeg s;
        s.net = 0;
        s.layer = 0;
        s.a = {mm_to_nm(9.5), mm_to_nm(10.5)};
        s.b = {mm_to_nm(11.5), mm_to_nm(10.5)};
        s.width_nm = mm_to_nm(0.2);
        t.traces.push_back(s);
        CT_CHECK(!v.verify(t, r, r.defaultContext()).legal);
    }
    // Trace through the via keepout: legal (wires are not blocked).
    {
        Board t = b;
        TraceSeg s;
        s.net = 0;
        s.layer = 0;
        s.a = {mm_to_nm(13.5), mm_to_nm(5.5)};
        s.b = {mm_to_nm(15.5), mm_to_nm(5.5)};
        s.width_nm = mm_to_nm(0.2);
        t.traces.push_back(s);
        CT_CHECK(v.verify(t, r, r.defaultContext()).legal);
    }
    // Via through the via keepout: violation (new per-kind via check).
    {
        Board t = b;
        Via vv;
        vv.net = 0;
        vv.pos = {mm_to_nm(14.5), mm_to_nm(5.5)};
        vv.top_layer = 0;
        vv.bottom_layer = 1;
        vv.outer_d_nm = mm_to_nm(0.6);
        vv.hole_d_nm = mm_to_nm(0.3);
        t.vias.push_back(vv);
        VerifyResult vr = v.verify(t, r, r.defaultContext());
        CT_CHECK(!vr.legal);
        bool saw_via_ko = false;
        for (const auto& vi : vr.violations)
            if (vi.rule == "keepout" && vi.detail.find("via enters") != std::string::npos)
                saw_via_ko = true;
        CT_CHECK(saw_via_ko);
    }
    // Via through the wire keepout: legal.
    {
        Board t = b;
        Via vv;
        vv.net = 0;
        vv.pos = {mm_to_nm(5.5), mm_to_nm(5.5)};
        vv.top_layer = 0;
        vv.bottom_layer = 1;
        vv.outer_d_nm = mm_to_nm(0.6);
        vv.hole_d_nm = mm_to_nm(0.3);
        t.vias.push_back(vv);
        CT_CHECK(v.verify(t, r, r.defaultContext()).legal);
    }
    // Pad center inside the place keepout: violation; a wire through a
    // place keepout is legal (placement rules do not block copper).
    {
        Board t = b;
        Keepout pk;
        pk.rect = {mm_to_nm(7), mm_to_nm(7), mm_to_nm(8), mm_to_nm(8)};
        pk.layer = 0;
        pk.kind = KeepoutKind::kPlace;
        pk.reason = "place";
        t.keepouts.push_back(pk);
        // Pad centered inside the region -> pad-keepout violation.
        Keepout bogus;
        bogus.rect = {mm_to_nm(7), mm_to_nm(7), mm_to_nm(8), mm_to_nm(8)};
        bogus.layer = 0;
        bogus.kind = KeepoutKind::kPlace;
        bogus.reason = "place";
        {
            Board p = t;
            Terminal pad;
            pad.id = static_cast<TermId>(p.terminals.size());
            pad.net = 0;
            pad.pos = {mm_to_nm(7.5), mm_to_nm(7.5)};
            pad.layer = 0;
            pad.pad_w_nm = mm_to_nm(0.5);
            pad.pad_h_nm = mm_to_nm(0.5);
            pad.component = "U9";
            pad.pin = "1";
            p.terminals.push_back(pad);
            CT_CHECK(!v.verify(p, r, r.defaultContext()).legal);
        }
        // Wire through the place keepout: legal (placement rules do not
        // block copper; the region is not a wire wall).
        TraceSeg s;
        s.net = 0;
        s.layer = 0;
        s.a = {mm_to_nm(6.5), mm_to_nm(7.5)};
        s.b = {mm_to_nm(8.5), mm_to_nm(7.5)};
        s.width_nm = mm_to_nm(0.2);
        t.traces.push_back(s);
        CT_CHECK(v.verify(t, r, r.defaultContext()).legal);
    }
    // Unknown keepout kind names are never fatal: separator-stripped names
    // resolve to the serialized form (round-trip via keepout_kind_name).
    CT_CHECK(std::string(keepout_kind_name(KeepoutKind::kVia)) == "via");
    CT_CHECK(std::string(keepout_kind_name(KeepoutKind::kAll)) == "all");
    CT_CHECK(keepout_kind_from_name("via") == KeepoutKind::kVia);
    CT_CHECK(keepout_kind_from_name("") == KeepoutKind::kAll);
}

// (microvia ...) and (via_rule ...) declarations are parsed and recorded
// (their names appear in the import warnings) but never silently converted
// into through-via copper: blind/buried/micro spans are out of the Board
// model, so importing them as through vias would fabricate spans the file
// did not declare. Never fatal.
CT_TEST(dsn_microvia_via_rule_recorded) {
    static const char* kDsn =
        "(pcb \"mv_demo\"\n"
        "  (unit mm)\n"
        "  (structure\n"
        "    (layer Top (type signal))\n"
        "    (layer Mid (type signal))\n"
        "    (layer Bottom (type signal))\n"
        "    (boundary (rect 0 0 20 20))\n"
        "    (microvia MV_1_2 Top Mid 0.3 0.15)\n"
        "    (via_rule STD (rotate 45) (offset 0 0))\n"
        "    (via_rule (via MV_1_2 (rotate 0) (offset 0 0))))\n"
        "  (placement (component U1 (place P 2 2 front 0))\n"
        "             (component U2 (place P 18 2 front 0)))\n"
        "  (library (image P (pin 1 0 0)))\n"
        "  (network (net N (pins U1-1 U2-1))))\n";
    DsnImporter dsn;
    ImportResult ir = dsn.import_text(kDsn, "mv_demo.dsn");
    // The import succeeds, the board is usable, and the declarations are
    // recorded by name in the warnings (never silent, never fatal).
    CT_CHECK(ir.board.layers.size() == 3);
    CT_CHECK(ir.board.nets.size() == 1);
    CT_CHECK(has_warning(ir.warnings, "recorded"));
    CT_CHECK(has_warning(ir.warnings, "microvia"));
    CT_CHECK(has_warning(ir.warnings, "MV_1_2"));
    CT_CHECK(has_warning(ir.warnings, "via_rule"));
    CT_CHECK(has_warning(ir.warnings, "STD"));
    // No fabricated via copper for the micro span.
    CT_CHECK(ir.board.vias.empty());
}

// ---- width min/pref/max triplets -------------------------------------
// The Specctra router routes at the Preferred width with Min as the floor
// (M9). All three DSN spellings — single value, positional min-pref-max,
// and the labelled (min ...)/(pref ...) fragment — must land on the net's
// min AND preferred width, and the chosen source must be reported.
CT_TEST(dsn_width_triplets) {
    static const char* kDsn =
        "(pcb \"w_triplets\"\n"
        "  (unit mm)\n"
        "  (structure (layer Top (type signal)) (layer Bottom (type signal))\n"
        "    (boundary (rect 0 0 20 20)))\n"
        "  (placement (component U1 (place P 2 2 front 0))\n"
        "             (component U2 (place P 18 2 front 0)))\n"
        "  (library (image P (pin 1 0 0)))\n"
        "  (network\n"
        "    (class SINGLE (rule (width 0.2) (clearance 0.15)))\n"
        "    (class POS (rule (width 0.15 0.25 0.4) (clearance 0.15)))\n"
        "    (class LAB (rule (width (min 0.1) (pref 0.3)) (clearance 0.15)))\n"
        "    (net A (pins U1-1 U2-1) (class SINGLE))\n"
        "    (net B (pins U1-1 U2-1) (class POS))\n"
        "    (net C (pins U1-1 U2-1) (class LAB))))\n";
    DsnImporter dsn;
    ImportResult ir = dsn.import_text(kDsn, "w_triplets.dsn");
    // Single value: min == pref == value, source reported as the plain value.
    const NetInfo* a = ir.board.find_net_by_name("A");
    CT_CHECK(a != nullptr && a->has_min_width);
    CT_CHECK(a->min_width_nm == mm_to_nm(0.2));
    CT_CHECK(!a->has_pref_width || a->pref_width_nm == mm_to_nm(0.2));
    // Positional triplet: min = 0.15, pref = 0.25 (max 0.4 recorded, unused).
    const NetInfo* b = ir.board.find_net_by_name("B");
    CT_CHECK(b != nullptr && b->has_min_width);
    CT_CHECK(b->min_width_nm == mm_to_nm(0.15));
    CT_CHECK(b->has_pref_width);
    CT_CHECK(b->pref_width_nm == mm_to_nm(0.25));
    CT_CHECK(has_warning(ir.warnings, "uses the preferred value"));
    // Labelled fragment: min = 0.1, pref = 0.3.
    const NetInfo* c = ir.board.find_net_by_name("C");
    CT_CHECK(c != nullptr && c->has_min_width);
    CT_CHECK(c->min_width_nm == mm_to_nm(0.1));
    CT_CHECK(c->has_pref_width);
    CT_CHECK(c->pref_width_nm == mm_to_nm(0.3));
    // The resolver routes at the preferred width (floored by min/current).
    RuleResolver r = RuleResolver::defaults_for(ir.board);
    ElectricalContext ctx;
    TraceRule tb = r.traceRule(b->id, 0, 0);
    CT_CHECK(tb.pref_width_nm == mm_to_nm(0.25));
    CT_CHECK(tb.pref_width_nm >= tb.min_width_nm);
    // Preferred below min: still routes at min and says so.
    static const char* kBadPref =
        "(pcb \"w_badpref\"\n"
        "  (unit mm)\n"
        "  (structure (layer Top (type signal)) (layer Bottom (type signal))\n"
        "    (boundary (rect 0 0 20 20)))\n"
        "  (placement (component U1 (place P 2 2 front 0))\n"
        "             (component U2 (place P 18 2 front 0)))\n"
        "  (library (image P (pin 1 0 0)))\n"
        "  (network\n"
        "    (class BAD (rule (width 0.3 0.2) (clearance 0.15)))\n"
        "    (net X (pins U1-1 U2-1) (class BAD))))\n";
    ImportResult ib = dsn.import_text(kBadPref, "w_badpref.dsn");
    const NetInfo* x = ib.board.find_net_by_name("X");
    CT_CHECK(x != nullptr && x->has_min_width);
    CT_CHECK(x->min_width_nm == mm_to_nm(0.3));
    CT_CHECK(x->has_pref_width);  // recorded, but the router floors at min
    CT_CHECK(has_warning(ib.warnings, "preferred below min"));
    RuleResolver rb = RuleResolver::defaults_for(ib.board);
    TraceRule tx = rb.traceRule(x->id, 0, 0);
    CT_CHECK(tx.pref_width_nm == mm_to_nm(0.3));  // max(pref, min) = min
}

// ---- FreeRouting / classic SES dialects ------------------------------------
// The session payload (wires + vias per net) is the same everywhere; only the
// nesting and the coordinate declaration differ. These tests pin both.

namespace {

// Minimal routed-pad board: two pads of net N at (2,2) and (18,2) mm.
Board ses_base_board() {
    static const char* kDsn =
        "(pcb \"ses_base\"\n"
        "  (unit mm)\n"
        "  (structure\n"
        "    (layer Top (type signal))\n"
        "    (layer Bottom (type signal))\n"
        "    (boundary (rect 0 0 20 20)))\n"
        "  (placement (component U1 (place P 2 2 front 0))\n"
        "             (component U2 (place P 18 2 front 0)))\n"
        "  (library (image P (pin 1 0 0)))\n"
        "  (network (net N (pins U1-1 U2-1))))\n";
    DsnImporter dsn;
    return std::move(dsn.import_text(kDsn, "ses_base.dsn").board);
}

}  // namespace

// FreeRouting nesting: (resolution um 10) + library_out/network_out, with the
// wire width in a sibling node and padstacks wrapped in an image. F.Cu/B.Cu
// resolve onto the board's outer layers rather than being dropped.
CT_TEST(ses_freerouting_dialect) {
    static const char* kSes =
        "(session \"free.ses\"\n"
        "  (base_design \"ses_base.dsn\")\n"
        "  (resolution um 10)\n"
        "  (library_out\n"
        "    (image \"Via[0-1]_800:400_um\"\n"
        "      (padstack \"Via[0-1]_800:400_um\"\n"
        "        (shape (circle F.Cu 800))\n"
        "        (shape (circle B.Cu 800))\n"
        "        (hole 400))))\n"
        "  (network_out\n"
        "    (net \"N\"\n"
        "      (wire (path F.Cu 2000 2000 18000 2000) (width 200) (type route))\n"
        "      (via \"Via[0-1]_800:400_um\" 18000 2000))))\n";
    Board b = ses_base_board();
    std::vector<std::string> warns = ses_merge_into(b, kSes, "free.ses");
    CT_CHECK(b.traces.size() == 1);
    // Integer micrometres next to `(resolution um 10)`: the declared decade is
    // a precision, not a quantum, and the merge must say it switched.
    CT_CHECK(has_warning(warns, "used the unit scale"));
    CT_CHECK(b.traces[0].a.x == mm_to_nm(2.0));
    CT_CHECK(b.traces[0].a.y == mm_to_nm(2.0));
    CT_CHECK(b.traces[0].b.x == mm_to_nm(18.0));
    CT_CHECK(b.traces[0].width_nm == mm_to_nm(0.2));
    CT_CHECK(b.traces[0].layer == b.layers.front().id);
    CT_CHECK(has_warning(warns, "matched outer layer 'Top'"));
    // Via geometry comes from the library_out shape + hole, not from defaults.
    CT_CHECK(b.vias.size() == 1);
    CT_CHECK(b.vias[0].pos.x == mm_to_nm(18.0));
    CT_CHECK(b.vias[0].outer_d_nm == mm_to_nm(0.8));
    CT_CHECK(b.vias[0].hole_d_nm == mm_to_nm(0.4));
    CT_CHECK(b.vias[0].via_class == "Via[0-1]_800:400_um");
    CT_CHECK(!has_warning(warns, "unknown padstack"));
}

// Classic nesting: (unit mm), route/library/network, (wires ...) and
// (vias ...) containers, width as the last path number, numeric padstack
// form. A dotted/lower-case layer name resolves through normalization.
CT_TEST(ses_classic_dialect) {
    static const char* kSes =
        "(session \"classic.ses\"\n"
        "  (base_design \"ses_base.dsn\")\n"
        "  (unit mm)\n"
        "  (route\n"
        "    (library (padstack \"STD\" 0.7 0.35))\n"
        "    (network\n"
        "      (net \"N\"\n"
        "        (wires (wire (path \"TOP\" 2 2 18 2 0.25)))\n"
        "        (vias (via \"STD\" 18 2))))))\n";
    Board b = ses_base_board();
    std::vector<std::string> warns = ses_merge_into(b, kSes, "classic.ses");
    CT_CHECK(b.traces.size() == 1);
    CT_CHECK(b.traces[0].width_nm == mm_to_nm(0.25));
    CT_CHECK(b.traces[0].a.x == mm_to_nm(2.0));
    CT_CHECK(b.traces[0].b.y == mm_to_nm(2.0));
    CT_CHECK(b.traces[0].layer == b.layers.front().id);
    CT_CHECK(b.vias.size() == 1);
    CT_CHECK(b.vias[0].outer_d_nm == mm_to_nm(0.7));
    CT_CHECK(b.vias[0].hole_d_nm == mm_to_nm(0.35));
    CT_CHECK(has_warning(warns, "coordinates scaled by 1 mm per file count"));
    // No aliasing happened: the layer name normalized exactly.
    CT_CHECK(!has_warning(warns, "matched outer layer"));
}

// A session that names a padstack it never defined keeps the via but says so;
// nothing is silently invented.
CT_TEST(ses_unknown_padstack_warns) {
    static const char* kSes =
        "(session \"missing.ses\"\n"
        "  (unit mm)\n"
        "  (library (padstack \"STD\" 0.7 0.35))\n"
        "  (network (net \"N\" (via \"GHOST\" 10 10))))\n";
    Board b = ses_base_board();
    std::vector<std::string> warns = ses_merge_into(b, kSes, "missing.ses");
    CT_CHECK(b.vias.size() == 1);
    CT_CHECK(b.vias[0].outer_d_nm == b.defaults.via_outer_nm);
    CT_CHECK(b.vias[0].hole_d_nm == b.defaults.via_hole_nm);
    CT_CHECK(has_warning(warns, "unknown padstack 'GHOST'"));
}

// Padstack shape with no (hole) fallback: half the outer diameter, reported.
CT_TEST(ses_padstack_without_hole_warns) {
    static const char* kSes =
        "(session \"nohole.ses\"\n"
        "  (unit mm)\n"
        "  (library_out (padstack \"P1\" (shape (circle Top 0.9))))\n"
        "  (network_out (net \"N\" (via \"P1\" 10 10))))\n";
    Board b = ses_base_board();
    std::vector<std::string> warns = ses_merge_into(b, kSes, "nohole.ses");
    CT_CHECK(b.vias.size() == 1);
    CT_CHECK(b.vias[0].outer_d_nm == mm_to_nm(0.9));
    CT_CHECK(b.vias[0].hole_d_nm == mm_to_nm(0.45));
    CT_CHECK(has_warning(warns, "has no (hole ...); assumed half the outer diameter"));
}

// (direction ...) / (grid on|off) are soft layer hints: recorded verbatim,
// folded into preferred_horizontal, round-tripped through JSON, and reported
// as not-yet-honoured instead of pretending the router applied them.
CT_TEST(dsn_layer_direction_and_grid) {
    static const char* kDsn =
        "(pcb \"dir_demo\"\n"
        "  (unit mm)\n"
        "  (structure\n"
        "    (layer Top (type signal) (direction horizontal) (grid on))\n"
        "    (layer Bottom (type signal) (direction 45) (grid off))\n"
        "    (layer Weird (type signal) (direction sideways))\n"
        "    (boundary (rect 0 0 20 20)))\n"
        "  (placement (component U1 (place P 2 2 front 0))\n"
        "             (component U2 (place P 18 2 front 0)))\n"
        "  (library (image P (pin 1 0 0)))\n"
        "  (network (net N (pins U1-1 U2-1))))\n";
    DsnImporter dsn;
    ImportResult ir = dsn.import_text(kDsn, "dir.dsn");
    CT_CHECK(ir.board.layers.size() == 3);
    CT_CHECK(ir.board.layers[0].direction == "horizontal");
    CT_CHECK(ir.board.layers[0].preferred_horizontal);
    CT_CHECK(ir.board.layers[0].grid_on);
    CT_CHECK(ir.board.layers[1].direction == "45");
    CT_CHECK(!ir.board.layers[1].preferred_horizontal);
    CT_CHECK(!ir.board.layers[1].grid_on);
    // Unknown tokens are kept verbatim and named, never fatal.
    CT_CHECK(ir.board.layers[2].direction == "sideways");
    CT_CHECK(has_warning(ir.warnings, "unknown (direction sideways)"));
    CT_CHECK(has_warning(ir.warnings, "the router does not bias step cost by axis yet"));

    // Native JSON keeps the hints so `route --output` cannot silently drop them.
    JsonBoardImporter imp;
    ImportResult rt = imp.import_value(board_to_json(ir.board), "dir_roundtrip");
    CT_CHECK(rt.board.layers.size() == 3);
    CT_CHECK(rt.board.layers[0].direction == "horizontal");
    CT_CHECK(rt.board.layers[0].preferred_horizontal);
    CT_CHECK(rt.board.layers[0].grid_on);
    CT_CHECK(rt.board.layers[1].direction == "45");
    CT_CHECK(rt.board.layers[2].direction == "sideways");
}

CT_TEST(dsn_pour_imports_as_plane) {
    DsnImporter dsn;
    ImportResult ir = dsn.import_file(fixture("pour_test.dsn"));
    CT_CHECK(ir.board.planes.size() == 1);
    const PlaneZone& z = ir.board.planes[0];
    const NetInfo* pwr = ir.board.find_net_by_name("PWR");
    CT_CHECK(pwr != nullptr);
    CT_CHECK(z.net == pwr->id);
    CT_CHECK(ir.board.layers[z.layer].name == "Bottom");
    CT_CHECK(z.poly.size() == 4);
    CT_CHECK(z.routable);
    // Rectangular pour corners in integer nm.
    CT_CHECK(z.poly[0] == Point({mm_to_nm(1), mm_to_nm(8)}));
    CT_CHECK(z.poly[2] == Point({mm_to_nm(19), mm_to_nm(12)}));
    // The GND pour (unknown net AND unknown layer) warns explicitly.
    CT_CHECK(has_warning(ir.warnings, "copper_pour"));
    CT_CHECK(has_warning(ir.warnings, "warning"));
}

CT_TEST(dsn_pour_routes_power_into_pour) {
    ImportResult ir = import_board_auto(fixture("pour_test.dsn"));
    Board b = std::move(ir.board);
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    opt.threads = 1;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status == "COMPLETE");
    CT_CHECK(rep.verification.ok);
    // Both PWR terminals entered the pour (plane-access records).
    const NetInfo* pwr = engine.committed().find_net_by_name("PWR");
    CT_CHECK(pwr != nullptr);
    int pwr_entries = 0;
    for (const auto& p : rep.plane_access)
        if (p.net == pwr->id) ++pwr_entries;
    CT_CHECK(pwr_entries == 2);
    // No PWR stub needed to run pad-to-pad: committed PWR copper is short.
    Coord pwr_len = 0;
    for (const auto& t : engine.committed().traces)
        if (t.net == pwr->id) pwr_len += manhattan(t.a, t.b);
    CT_CHECK(pwr_len < mm_to_nm(8.0));
}

CT_TEST(foreign_pour_blocks_and_clearance_enforced) {
    // Full-width GND pours on both layers: SIG cannot cross legally.
    Board b = base_2layer();
    NetInfo gnd = make_net(0, "GND");
    b.nets.push_back(gnd);
    NetInfo sig = make_net(1, "SIG");
    b.nets.push_back(sig);
    add_terminal(b, 1, 10.0, 2.0, 0);
    add_terminal(b, 1, 10.0, 18.0, 0);
    for (LayerId l : {0, 1}) {
        PlaneZone z;
        z.id = l;
        z.net = 0;
        z.layer = l;
        z.island = l;
        z.routable = true;
        z.poly = {{0, mm_to_nm(9)}, {mm_to_nm(20), mm_to_nm(9)},
                  {mm_to_nm(20), mm_to_nm(11)}, {0, mm_to_nm(11)}};
        b.planes.push_back(z);
    }
    // A hand-placed SIG trace straight through the foreign pour violates.
    Board crossed = b;
    TraceSeg s;
    s.net = 1;
    s.layer = 0;
    s.a = {mm_to_nm(10), mm_to_nm(2)};
    s.b = {mm_to_nm(10), mm_to_nm(18)};
    s.width_nm = mm_to_nm(0.2);
    crossed.traces.push_back(s);
    BoardVerifier v;
    RuleResolver res = RuleResolver::defaults_for(crossed);
    VerifyResult vr = v.verify(crossed, res, res.defaultContext());
    CT_CHECK(!vr.legal);
    CT_CHECK(!vr.violations.empty());
    // Near-miss inside the clearance band violates too (0.1 < 0.15mm).
    Board near = b;
    TraceSeg n;
    n.net = 1;
    n.layer = 0;
    n.a = {mm_to_nm(1), mm_to_nm(11.2)};
    n.b = {mm_to_nm(19), mm_to_nm(11.2)};
    n.width_nm = mm_to_nm(0.2);
    near.traces.push_back(n);
    RuleResolver res2 = RuleResolver::defaults_for(near);
    VerifyResult vr2 = v.verify(near, res2, res2.defaultContext());
    CT_CHECK(!vr2.legal);
    // The router must not falsely succeed through the wall.
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    opt.threads = 1;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status != "COMPLETE" || rep.verification.ok);
    CT_CHECK(rep.status == "INCOMPLETE");
}

CT_TEST(kicad_zone_imports_as_plane) {
    KicadPcbImporter kicad;
    ImportResult ir = kicad.import_file(fixture("pour_zone.kicad_pcb"));
    CT_CHECK(ir.board.planes.size() == 1);
    const PlaneZone& z = ir.board.planes[0];
    const NetInfo* gnd = ir.board.find_net_by_name("GND");
    CT_CHECK(gnd != nullptr);
    CT_CHECK(z.net == gnd->id);
    CT_CHECK(ir.board.layers[z.layer].name == "B.Cu");
    CT_CHECK(z.routable);
    // Filled polygon wins over the outline polygon (96.5, translated by
    // the 95mm Edge.Cuts origin => 1.5mm).
    CT_CHECK(z.poly.size() == 4);
    CT_CHECK(z.poly[0] == Point({mm_to_nm(1.5), mm_to_nm(1.5)}));
    CT_CHECK(z.poly[2] == Point({mm_to_nm(8.5), mm_to_nm(8.5)}));
    // The net-0 zone warns explicitly (never silent).
    CT_CHECK(has_warning(ir.warnings, "zone"));
    CT_CHECK(has_warning(ir.warnings, "warning"));
}

CT_TEST(kicad_property_reference_keeps_instances_distinct) {
    // KiCad 7+ writes the reference as a footprint property, not fp_text.
    // Missing it collapses every instance of a shared footprint name into one
    // synthetic component, which explodes fine-pitch grouping downstream.
    static const char* kPcb =
        "(kicad_pcb (version 20241229) (generator pcbnew)\n"
        "  (general)\n"
        "  (paper \"A4\")\n"
        "  (layers (0 \"F.Cu\" signal) (31 \"B.Cu\" signal))\n"
        "  (setup)\n"
        "  (net 0 \"\")\n"
        "  (net 1 \"GND\")\n"
        "  (footprint \"Test:R_0805\" (layer \"F.Cu\") (at 100 100)\n"
        "    (property \"Reference\" \"R1\" (at 0 -1.5 0))\n"
        "    (pad \"1\" smd rect (at -0.95 0) (size 1 1.2) (layers \"F.Cu\") (net 1 \"GND\"))\n"
        "    (pad \"2\" smd rect (at 0.95 0) (size 1 1.2) (layers \"F.Cu\") (net 1 \"GND\"))\n"
        "  )\n"
        "  (footprint \"Test:R_0805\" (layer \"F.Cu\") (at 105 100)\n"
        "    (property \"Reference\" \"R2\" (at 0 -1.5 0))\n"
        "    (pad \"1\" smd rect (at -0.95 0) (size 1 1.2) (layers \"F.Cu\") (net 1 \"GND\"))\n"
        "    (pad \"2\" smd rect (at 0.95 0) (size 1 1.2) (layers \"F.Cu\") (net 1 \"GND\"))\n"
        "  )\n"
        ")\n";
    KicadPcbImporter kicad;
    ImportResult ir = kicad.import_text(kPcb, "prop_ref.kicad_pcb");
    CT_CHECK(ir.board.terminals.size() == 4);
    std::set<std::string> comps;
    for (const auto& t : ir.board.terminals) comps.insert(t.component);
    // Distinct references -> distinct instances, not "Test:R_0805" twice.
    CT_CHECK(comps.count("R1") == 1);
    CT_CHECK(comps.count("R2") == 1);
    CT_CHECK(comps.count("Test:R_0805") == 0);
}

// A circle pad is rotation invariant. Applying the rotated-rectangle bbox
// formula inflated round pads by up to sqrt(2) on rotated footprints, which
// turned real boards (KiCad demos) into false clearance violations.
CT_TEST(kicad_rotated_circle_pad_not_inflated) {
    static const char* kPcb =
        "(kicad_pcb (version 20241229) (generator pcbnew)\n"
        "  (general)\n"
        "  (paper \"A4\")\n"
        "  (layers (0 \"F.Cu\" signal) (31 \"B.Cu\" signal))\n"
        "  (setup)\n"
        "  (net 0 \"\")\n"
        "  (net 1 \"GND\")\n"
        "  (gr_line (start 90 90) (end 110 90) (layer \"Edge.Cuts\") (width 0.05))\n"
        "  (gr_line (start 110 90) (end 110 110) (layer \"Edge.Cuts\") (width 0.05))\n"
        "  (gr_line (start 110 110) (end 90 110) (layer \"Edge.Cuts\") (width 0.05))\n"
        "  (gr_line (start 90 110) (end 90 90) (layer \"Edge.Cuts\") (width 0.05))\n"
        "  (footprint \"Test:Round\" (layer \"F.Cu\") (at 100 100 45)\n"
        "    (property \"Reference\" \"J1\" (at 0 -1.5 0))\n"
        "    (pad \"1\" thru_hole circle (at 0 0) (size 1.6 1.6)\n"
        "      (drill 0.8) (layers \"*.Cu\") (net 1 \"GND\"))\n"
        "  )\n"
        ")\n";
    KicadPcbImporter kicad;
    ImportResult ir = kicad.import_text(kPcb, "round_pad.kicad_pcb");
    // A thru-hole pad fans out to one terminal per copper layer.
    CT_CHECK(ir.board.terminals.size() == 2);
    for (const auto& t : ir.board.terminals) {
        CT_CHECK(t.pad_w_nm == mm_to_nm(1.6));
        CT_CHECK(t.pad_h_nm == mm_to_nm(1.6));
    }
    // A rotated RECT pad still gets its bounding box (conservative).
    static const char* kRect =
        "(kicad_pcb (version 20241229) (generator pcbnew)\n"
        "  (general)\n"
        "  (paper \"A4\")\n"
        "  (layers (0 \"F.Cu\" signal) (31 \"B.Cu\" signal))\n"
        "  (setup)\n"
        "  (net 1 \"GND\")\n"
        "  (gr_line (start 90 90) (end 110 90) (layer \"Edge.Cuts\") (width 0.05))\n"
        "  (gr_line (start 110 90) (end 110 110) (layer \"Edge.Cuts\") (width 0.05))\n"
        "  (gr_line (start 110 110) (end 90 110) (layer \"Edge.Cuts\") (width 0.05))\n"
        "  (gr_line (start 90 110) (end 90 90) (layer \"Edge.Cuts\") (width 0.05))\n"
        "  (footprint \"Test:Rect\" (layer \"F.Cu\") (at 100 100 45)\n"
        "    (property \"Reference\" \"J2\" (at 0 -1.5 0))\n"
        "    (pad \"1\" smd rect (at 0 0) (size 1 2)\n"
        "      (layers \"F.Cu\") (net 1 \"GND\"))\n"
        "  )\n"
        ")\n";
    ImportResult ir2 = kicad.import_text(kRect, "rect_pad.kicad_pcb");
    const Terminal& r = ir2.board.terminals.front();
    // 1x2 at 45 deg -> ~2.12 x 2.12 AABB.
    CT_CHECK(r.pad_w_nm > mm_to_nm(2.0));
    CT_CHECK(r.pad_w_nm < mm_to_nm(2.2));
}

CT_TEST(kicad_export_roundtrip) {
    KicadPcbImporter kicad;
    ImportResult ir = kicad.import_file(fixture("minimal.kicad_pcb"));
    // Net-class rules survived import.
    const NetInfo* vcc = ir.board.find_net_by_name("VCC");
    CT_CHECK(vcc != nullptr && vcc->has_min_width);
    CT_CHECK(vcc->min_width_nm == mm_to_nm(0.8));
    std::string text = board_to_kicad_pcb(ir.board);
    CT_CHECK(text.find("(kicad_pcb") != std::string::npos);
    CT_CHECK(text.find("(segment") == std::string::npos);  // unrouted: no copper yet
    ImportResult re = kicad.import_text(text, "roundtrip.kicad_pcb");
    // Identical nets.
    CT_CHECK(re.board.nets.size() == ir.board.nets.size());
    for (const auto& n : ir.board.nets) {
        const NetInfo* m = re.board.find_net_by_name(n.name);
        CT_CHECK(m != nullptr);
        CT_CHECK(m->id == n.id);
    }
    // Identical pads (geometry + net + layer; component refs preserved).
    CT_CHECK(re.board.terminals.size() == ir.board.terminals.size());
    std::vector<TermKey> a, e;
    std::map<TermId, std::string> acomp, ecomp;
    for (const auto& t : ir.board.terminals) {
        a.push_back({t.pos, t.layer, t.pad_w_nm, t.pad_h_nm, t.net});
        acomp[t.id] = t.component + "." + t.pin;
    }
    for (const auto& t : re.board.terminals) {
        e.push_back({t.pos, t.layer, t.pad_w_nm, t.pad_h_nm, t.net});
        ecomp[t.id] = t.component + "." + t.pin;
    }
    std::sort(a.begin(), a.end(), [](const TermKey& x, const TermKey& y) { return x < y; });
    std::sort(e.begin(), e.end(), [](const TermKey& x, const TermKey& y) { return x < y; });
    CT_CHECK(a.size() == e.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
        CT_CHECK(a[i].pos == e[i].pos);
        CT_CHECK(a[i].layer == e[i].layer);
        CT_CHECK(a[i].w == e[i].w && a[i].h == e[i].h);
        CT_CHECK(a[i].net == e[i].net);
    }
    // Component/pin identity preserved for real footprints.
    {
        std::vector<std::string> ac, ec;
        for (const auto& kv : acomp) ac.push_back(kv.second);
        for (const auto& kv : ecomp) ec.push_back(kv.second);
        std::sort(ac.begin(), ac.end());
        std::sort(ec.begin(), ec.end());
        CT_CHECK(ac == ec);
    }
    // Identical rules.
    CT_CHECK(re.board.defaults.clearance_nm == ir.board.defaults.clearance_nm);
    CT_CHECK(re.board.defaults.trace_width_nm == ir.board.defaults.trace_width_nm);
    const NetInfo* rvcc = re.board.find_net_by_name("VCC");
    CT_CHECK(rvcc != nullptr && rvcc->has_min_width);
    CT_CHECK(rvcc->min_width_nm == mm_to_nm(0.8));
    CT_CHECK(rvcc->has_min_clearance && rvcc->min_clearance_nm == mm_to_nm(0.3));
    CT_CHECK(re.board.traces.size() == ir.board.traces.size());
    CT_CHECK(re.board.vias.size() == ir.board.vias.size());
}

CT_TEST(kicad_zone_roundtrip_preserves_pour) {
    KicadPcbImporter kicad;
    ImportResult ir = kicad.import_file(fixture("pour_zone.kicad_pcb"));
    CT_CHECK(ir.board.planes.size() == 1);
    std::string text = board_to_kicad_pcb(ir.board);
    CT_CHECK(text.find("(zone") != std::string::npos);
    ImportResult re = kicad.import_text(text, "roundtrip.kicad_pcb");
    CT_CHECK(re.board.planes.size() == 1);
    CT_CHECK(re.board.planes[0].poly == ir.board.planes[0].poly);
    CT_CHECK(re.board.planes[0].net == ir.board.planes[0].net);
    CT_CHECK(re.board.planes[0].layer == ir.board.planes[0].layer);
}

CT_TEST(kicad_route_export_verify) {
    // `route board.kicad_pcb --output routed.kicad_pcb` then verify it.
    ImportResult ir = import_board_auto(fixture("minimal.kicad_pcb"));
    Board b = std::move(ir.board);
    RuleResolver r = RuleResolver::defaults_for(b);
    EngineOptions opt;
    opt.threads = 1;
    RouterEngine engine(std::move(b), std::move(r), opt);
    RouteReport rep = engine.run();
    CT_CHECK(rep.status == "COMPLETE");
    std::string text = board_to_kicad_pcb(engine.committed());
    KicadPcbImporter kicad;
    ImportResult re = kicad.import_text(text, "routed.kicad_pcb");
    CT_CHECK(!re.board.traces.empty());
    CT_CHECK(re.board.traces.size() == engine.committed().traces.size());
    CT_CHECK(re.board.vias.size() == engine.committed().vias.size());
    BoardVerifier v;
    RuleResolver vr = RuleResolver::defaults_for(re.board);
    VerifyResult res = v.verify(re.board, vr, vr.defaultContext());
    CT_CHECK(res.ok);
    CT_CHECK(res.connected);
    CT_CHECK(res.legal);
}

CT_TEST(gerber_copper_geometry) {
    GerberImporter gerber;
    ImportResult ir = gerber.import_file(fixture("gerber_copper.gbr"));
    CT_CHECK(ir.board.nets.size() == 1);
    CT_CHECK(ir.board.nets[0].name == "COPPER");
    // Outline = copper bbox + 1mm: x 1.75..18.25 => 0.75..19.25 (18.5mm),
    // y 3.75..14.5 => 2.75..15.5 (12.75mm; rect flash counts max(w,h)/2).
    CT_CHECK(ir.board.width_nm == mm_to_nm(18.5));
    CT_CHECK(ir.board.height_nm == mm_to_nm(12.75));
    // Flashes: circle 0.5 at (5,4), rect 1.0x0.4 at (8,14).
    CT_CHECK(ir.board.terminals.size() == 2);
    const Terminal* rect_pad = nullptr;
    const Terminal* round_pad = nullptr;
    for (const auto& t : ir.board.terminals) {
        if (t.pad_w_nm == mm_to_nm(1.0)) rect_pad = &t;
        if (t.pad_w_nm == mm_to_nm(0.5)) round_pad = &t;
    }
    CT_CHECK(rect_pad != nullptr && round_pad != nullptr);
    CT_CHECK(rect_pad->pos == Point({mm_to_nm(7.25), mm_to_nm(11.25)}));
    CT_CHECK(rect_pad->pad_h_nm == mm_to_nm(0.4));
    CT_CHECK(round_pad->pos == Point({mm_to_nm(4.25), mm_to_nm(1.25)}));
    // Draw: (2,10)-(18,10) at 0.5mm width on Top.
    CT_CHECK(ir.board.traces.size() == 1);
    const TraceSeg& s = ir.board.traces[0];
    CT_CHECK(s.a == Point({mm_to_nm(1.25), mm_to_nm(7.25)}));
    CT_CHECK(s.b == Point({mm_to_nm(17.25), mm_to_nm(7.25)}));
    CT_CHECK(s.width_nm == mm_to_nm(0.5));
    CT_CHECK(s.layer == 0);
    // Region: rect (12,8)-(16,12) as a routable pour on the COPPER net.
    CT_CHECK(ir.board.planes.size() == 1);
    const PlaneZone& z = ir.board.planes[0];
    CT_CHECK(z.net == 0 && z.routable);
    CT_CHECK(z.poly.size() == 4);
    CT_CHECK(z.poly[0] == Point({mm_to_nm(11.25), mm_to_nm(5.25)}));
    CT_CHECK(z.poly[2] == Point({mm_to_nm(15.25), mm_to_nm(9.25)}));
    CT_CHECK(has_warning(ir.warnings, "single-net"));
}

CT_TEST(gerber_profile_outline) {
    GerberImporter gerber;
    ImportResult ir = gerber.import_file(fixture("gerber_profile.gko"));
    CT_CHECK(ir.board.width_nm == mm_to_nm(20.0));
    CT_CHECK(ir.board.height_nm == mm_to_nm(20.0));
    CT_CHECK(ir.board.terminals.empty());
    CT_CHECK(ir.board.traces.empty());
    CT_CHECK(has_warning(ir.warnings, "profile"));
}

CT_TEST(ipc2581_imports_nets_pads_traces) {
    Ipc2581Importer ipc;
    ImportResult ir = ipc.import_file(fixture("ipc2581_demo.xml"));
    CT_CHECK(ir.board.nets.size() == 2);
    CT_CHECK(ir.board.find_net_by_name("SIG1") != nullptr);
    const NetInfo* vbat = ir.board.find_net_by_name("VBAT");
    CT_CHECK(vbat != nullptr);
    // NetClass PWR mapped to width/clearance floors.
    CT_CHECK(vbat->has_min_width && vbat->min_width_nm == mm_to_nm(0.8));
    CT_CHECK(vbat->has_min_clearance && vbat->min_clearance_nm == mm_to_nm(0.2));
    // Pads with component identity and integer-nm positions.
    CT_CHECK(ir.board.terminals.size() == 4);
    bool saw_u1 = false;
    for (const auto& t : ir.board.terminals) {
        if (t.component == "U1" && t.net == vbat->id) {
            saw_u1 = true;
            CT_CHECK(t.pos == Point({mm_to_nm(2), mm_to_nm(5)}));
            CT_CHECK(t.pad_w_nm == mm_to_nm(0.6));
        }
    }
    CT_CHECK(saw_u1);
    // Trace + via with exact geometry.
    CT_CHECK(ir.board.traces.size() == 1);
    const NetInfo* sig1 = ir.board.find_net_by_name("SIG1");
    CT_CHECK(ir.board.traces[0].net == sig1->id);
    CT_CHECK(ir.board.traces[0].a == Point({mm_to_nm(2), mm_to_nm(10)}));
    CT_CHECK(ir.board.traces[0].b == Point({mm_to_nm(18), mm_to_nm(10)}));
    CT_CHECK(ir.board.traces[0].width_nm == mm_to_nm(0.2));
    CT_CHECK(ir.board.vias.size() == 1);
    CT_CHECK(ir.board.vias[0].pos == Point({mm_to_nm(10), mm_to_nm(5)}));
    CT_CHECK(ir.board.vias[0].outer_d_nm == mm_to_nm(0.6));
    CT_CHECK(ir.board.vias[0].hole_d_nm == mm_to_nm(0.3));
    CT_CHECK(ir.board.vias[0].net == vbat->id);
    // Datum outline.
    CT_CHECK(ir.board.width_nm == mm_to_nm(20.0));
    CT_CHECK(ir.board.height_nm == mm_to_nm(20.0));
    // Auto-claim by content: import_board_auto resolves the .xml path.
    ImportResult auto_ir = import_board_auto(fixture("ipc2581_demo.xml"));
    CT_CHECK(auto_ir.board.source_format == "ipc-2581");
    ImportResult auto_gbr = import_board_auto(fixture("gerber_copper.gbr"));
    CT_CHECK(auto_gbr.board.source_format == "gerber");
}

// Item 9: layer-scoped net-class rules are parsed and surfaced honestly.
CT_TEST(dsn_layer_scoped_class_rules) {
    static const char* kDsn =
        "(pcb \"layerscope\"\n"
        "  (unit mm)\n"
        "  (structure\n"
        "    (layer Top (type signal))\n"
        "    (layer Bottom (type signal))\n"
        "    (boundary (rect 0 0 20 20))\n"
        "    (via STD 0.6 0.3))\n"
        "  (placement\n"
        "    (component U1 (place P 1 1 front 0))\n"
        "    (component U2 (place P 19 1 front 0)))\n"
        "  (library (image P (pin 1 0 0)))\n"
        "  (network\n"
        "    (class PWR\n"
        "      (rule (width 0.5)\n"
        "            (layer \"Top Layer\" (width 1.0) (clearance 0.3))\n"
        "            (via_share on)\n"
        "            (via_at_smd on (grid on))\n"
        "            (smd_to_via_gap 0.2)\n"
        "            (poly_connect (style relief) (airgap 0.4)\n"
        "                          (spoke_w 0.25) (entries 4))))\n"
        "    (net V1 (pins U1-1 U2-1) (class PWR))))\n";
    DsnImporter dsn;
    ImportResult ir = dsn.import_text(kDsn, "layerscope.dsn");
    CT_CHECK(ir.board.nets.size() == 1);
    CT_CHECK(ir.board.nets[0].name == "V1");
    // Class-wide width still applies (layer-scoped is recorded, not enforced
    // by the class-wide model).
    CT_CHECK(ir.board.nets[0].has_min_width);
    CT_CHECK(ir.board.nets[0].min_width_nm == mm_to_nm(0.5));
    // Item 9 surfaced: layer-scoped rules warn, never silent.
    CT_CHECK(has_warning(ir.warnings, "layer-scoped"));
    // Item 12 surfaced: via policy knobs warn.
    CT_CHECK(has_warning(ir.warnings, "via-policy"));
    // Item 13 surfaced: PolygonConnect style recorded + warned, with the
    // pointer to the EM/thermal tier.
    CT_CHECK(has_warning(ir.warnings, "PolygonConnect"));
    CT_CHECK(has_warning(ir.warnings, "em/validate_thermal.py"));
}

// Items 9/12/13 must never change the committed geometry (deterministic
// import; the new fields are advisory).
CT_TEST(dsn_phase2_fields_do_not_affect_geometry) {
    static const char* kDsn =
        "(pcb \"phase2\"\n"
        "  (unit mm)\n"
        "  (structure\n"
        "    (layer Top (type signal))\n"
        "    (layer Bottom (type signal))\n"
        "    (boundary (rect 0 0 20 20))\n"
        "    (via STD 0.6 0.3))\n"
        "  (placement\n"
        "    (component U1 (place P 1 1 front 0))\n"
        "    (component U2 (place P 19 1 front 0)))\n"
        "  (library (image P (pin 1 0 0)))\n"
        "  (network\n"
        "    (class PWR (rule (width 0.5)\n"
        "           (via_share on) (smd_to_via_gap 0.2)\n"
        "           (poly_connect (style relief) (entries 4))))\n"
        "    (net V1 (pins U1-1 U2-1) (class PWR))))\n";
    DsnImporter dsn;
    ImportResult ir = dsn.import_text(kDsn, "phase2.dsn");
    // Same geometry as the same board without the advisory fields: two
    // terminals, correct span.
    CT_CHECK(ir.board.terminals.size() == 2);
    Coord span = std::llabs(ir.board.terminals[0].pos.x - ir.board.terminals[1].pos.x);
    CT_CHECK(span == mm_to_nm(18.0));
    CT_CHECK(ir.board.nets[0].min_width_nm == mm_to_nm(0.5));
}

int main() { return copperline::test::run_all_tests(); }
