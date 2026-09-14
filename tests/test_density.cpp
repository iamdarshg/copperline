#include "helpers.h"

#include "router/analyze.h"
#include "router/density.h"

using namespace copperline;
using namespace copperline::test;

namespace {

Board mcu_board() {
    Board b = base_2layer(30.0, 30.0);
    // Dense 8x8 "MCU" grid, 0.5mm pitch, in the corner.
    NetId gid = 100;
    for (int ix = 0; ix < 8; ++ix) {
        for (int iy = 0; iy < 8; ++iy) {
            NetInfo n = make_net(gid, "G" + std::to_string(gid));
            b.nets.push_back(n);
            add_terminal(b, gid, 2.0 + ix * 0.5, 2.0 + iy * 0.5, 0, 0.25, "U1",
                         "P" + std::to_string(ix * 8 + iy));
            ++gid;
        }
    }
    // One lonely net in open space.
    NetInfo lone = make_net(200, "LONE");
    lone.has_current = true;
    lone.current_a = 0.1;
    b.nets.push_back(lone);
    add_terminal(b, 200, 25.0, 25.0, 0, 0.5, "J1", "1");
    add_terminal(b, 200, 27.0, 25.0, 0, 0.5, "J2", "1");
    return b;
}

}  // namespace

CT_TEST(dense_region_scores_higher) {
    Board b = mcu_board();
    DensityEstimator est;
    double dense = est.local_density(b, {mm_to_nm(3.5), mm_to_nm(3.5)});
    double open = est.local_density(b, {mm_to_nm(20.0), mm_to_nm(10.0)});
    CT_CHECK(dense > 3.0);   // ~50 neighbors in the 2mm kernel
    CT_CHECK(open < 0.25);   // empty space
    CT_CHECK(dense > 10 * (open + 1e-9));
}

CT_TEST(dense_footprint_detected) {
    Board b = mcu_board();
    DensityEstimator est;
    DensityResult r = est.analyze(b);
    CT_CHECK(!r.dense_footprints.empty());
    bool found_u1 = false;
    for (const auto& fp : r.dense_footprints) {
        if (fp.component == "U1") {
            found_u1 = true;
            CT_CHECK(fp.pin_count == 64);
        }
    }
    CT_CHECK(found_u1);
    CT_CHECK(r.board_avg_per_mm2 > 0);
}

CT_TEST(analyze_report_shape) {
    Board b = mcu_board();
    RuleResolver resolver = RuleResolver::defaults_for(b);
    ElectricalContext ctx;
    AnalysisResult a = analyze_board(b, resolver, ctx, {"demo warning"});
    const JsonValue& r = a.data;
    CT_CHECK(r.find("schema")->as_string() == std::string("copperline/analyze-report/1"));
    CT_CHECK(r.find("nets")->is_array());
    CT_CHECK(r.find("dense_footprints")->is_array());
    CT_CHECK(!r.find("dense_footprints")->as_array().empty());
    CT_CHECK(r.find("current_classes")->is_array());
    CT_CHECK(r.find("voltage_classes")->is_array());
    CT_CHECK(r.find("likely_bottlenecks")->is_array());
    CT_CHECK(r.find("defaults_used_for_current")->is_array());
    // Grid nets carry no current metadata -> board default reported.
    bool grid_defaulted = false;
    for (const auto& n : r.find("nets")->as_array()) {
        if (n.get_string("name") == "G100") {
            CT_CHECK(n.get_bool("default_current_used", false));
            grid_defaulted = true;
        }
        if (n.get_string("name") == "LONE") {
            // LONE states 0.1A explicitly -> no default involved.
            CT_CHECK(!n.get_bool("default_current_used", true));
        }
    }
    CT_CHECK(grid_defaulted);
    CT_CHECK(!r.find("import_warnings")->as_array().empty());
}

int main() { return copperline::test::run_all_tests(); }
