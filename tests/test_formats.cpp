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

int main() { return copperline::test::run_all_tests(); }
