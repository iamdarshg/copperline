// Shared test helpers: tiny framework + synthetic board builders.
#pragma once

#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "router/board.h"
#include "router/rules.h"

#define CT_TEST(name)                                                                        \
    static void ctest_fn_##name();                                                           \
    static struct ctest_reg_##name {                                                         \
        ctest_reg_##name() {                                                                 \
            ::copperline::test::registry().push_back({#name, ctest_fn_##name});               \
        }                                                                                    \
    } ctest_reg_instance_##name;                                                             \
    static void ctest_fn_##name()

#define CT_CHECK(cond)                                                                \
    do {                                                                              \
        if (!(cond))                                                                    \
            throw std::runtime_error(std::string("CHECK failed: ") + #cond + " @" +   \
                                     std::to_string(__LINE__));                       \
    } while (0)

#define CT_CHECK_NEAR(a, b, eps)                                                              \
    do {                                                                                      \
        if (std::fabs((a) - (b)) > (eps))                                                     \
            throw std::runtime_error(std::string("NEAR failed: ") + #a " vs " #b " @" +       \
                                     std::to_string(__LINE__));                               \
    } while (0)

namespace copperline {
namespace test {

struct TestCase {
    std::string name;
    std::function<void()> fn;
};

inline std::vector<TestCase>& registry() {
    static std::vector<TestCase> r;
    return r;
}

inline int run_all_tests() {
    int fails = 0;
    for (auto& t : registry()) {
        auto t0 = std::chrono::steady_clock::now();
        try {
            t.fn();
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::steady_clock::now() - t0)
                          .count();
            std::cout << "ok - " << t.name << " (" << ms << "ms)\n";
        } catch (const std::exception& e) {
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::steady_clock::now() - t0)
                          .count();
            std::cout << "not ok - " << t.name << " (" << ms << "ms): " << e.what()
                      << "\n";
            ++fails;
        }
    }
    std::cout << (fails ? "FAIL" : "PASS") << ": " << (registry().size() - fails) << "/"
              << registry().size() << " passed\n";
    return fails ? 1 : 0;
}

// ---- Synthetic board builders (shared by unit + fixture tests) ----

inline Board base_2layer(double w_mm = 20.0, double h_mm = 20.0) {
    Board b;
    b.source_format = "test";
    b.width_nm = mm_to_nm(w_mm);
    b.height_nm = mm_to_nm(h_mm);
    b.layers.push_back({0, "Top"});
    b.layers.push_back({1, "Bottom"});
    return b;
}

inline NetInfo make_net(NetId id, const std::string& name) {
    NetInfo n;
    n.id = id;
    n.name = name;
    return n;
}

inline TermId& term_id_counter() {
    static TermId next = 0;
    return next;
}

inline TermId add_terminal(Board& board, NetId net, double x_mm, double y_mm, LayerId layer = 0,
                           double pad_mm = 0.5, const std::string& comp = "",
                           const std::string& pin = "") {
    TermId next = term_id_counter();
    Terminal t;
    t.id = next++;
    term_id_counter() = next;
    t.net = net;
    t.pos = {mm_to_nm(x_mm), mm_to_nm(y_mm)};
    t.layer = layer;
    t.pad_w_nm = mm_to_nm(pad_mm);
    t.pad_h_nm = mm_to_nm(pad_mm);
    t.component = comp;
    t.pin = pin;
    board.terminals.push_back(t);
    if (NetInfo* n = board.find_net(net)) n->terminals.push_back(t.id);
    return t.id;
}

// Reset the terminal-id counter so a freshly built board has dense ids
// (0..n-1). Board::find_terminal's O(1) fast path requires terminals[i].id
// == i, which large synthetic boards in one test binary otherwise break.
inline void reset_term_ids() { term_id_counter() = 0; }

}  // namespace test
}  // namespace copperline
