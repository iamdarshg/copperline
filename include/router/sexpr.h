// Copperline: minimal S-expression parser (KiCad s-expr, Specctra DSN, ...).
//
// Shared lexer/parser for the Lisp-like board formats used across PCB EDA.
// Produces a simple node tree; each format importer interprets it.
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace copperline {

struct SexprNode {
    bool is_atom = true;
    std::string atom;  // valid when is_atom
    std::vector<std::unique_ptr<SexprNode>> children;  // valid when !is_atom

    static std::unique_ptr<SexprNode> atom_node(const std::string& a) {
        auto n = std::make_unique<SexprNode>();
        n->is_atom = true;
        n->atom = a;
        return n;
    }
    static std::unique_ptr<SexprNode> list_node() {
        auto n = std::make_unique<SexprNode>();
        n->is_atom = false;
        return n;
    }

    // Child helpers: head atom of a list, first child with a given head, etc.
    std::string head() const;
    const SexprNode* find_child(const std::string& head) const;
    std::vector<const SexprNode*> find_all(const std::string& head) const;
};

// Throws std::runtime_error on malformed input.
std::unique_ptr<SexprNode> parse_sexpr(const std::string& text);

}  // namespace copperline
