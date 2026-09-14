#include "router/sexpr.h"

#include <stdexcept>

namespace copperline {

std::string SexprNode::head() const {
    if (is_atom) return atom;
    if (!children.empty() && children.front()->is_atom) return children.front()->atom;
    return "";
}

const SexprNode* SexprNode::find_child(const std::string& head) const {
    for (const auto& c : children) {
        if (!c->is_atom && c->head() == head) return c.get();
    }
    return nullptr;
}

std::vector<const SexprNode*> SexprNode::find_all(const std::string& head) const {
    std::vector<const SexprNode*> out;
    for (const auto& c : children) {
        if (!c->is_atom && c->head() == head) out.push_back(c.get());
    }
    return out;
}

namespace {

class Lexer {
  public:
    explicit Lexer(const std::string& text) : s_(text) {}

    // Returns false at end of input.
    bool next(std::string& token, bool& is_paren) {
        skip_space_and_comments();
        if (pos_ >= s_.size()) return false;
        char c = s_[pos_];
        if (c == '(' || c == ')') {
            token = std::string(1, c);
            is_paren = true;
            ++pos_;
            return true;
        }
        if (c == '"') {
            token = parse_quoted();
            is_paren = false;
            return true;
        }
        std::size_t start = pos_;
        while (pos_ < s_.size() && !is_delim(s_[pos_])) ++pos_;
        token = s_.substr(start, pos_ - start);
        is_paren = false;
        return true;
    }

  private:
    const std::string& s_;
    std::size_t pos_ = 0;

    static bool is_delim(char c) {
        return c == '(' || c == ')' || c == '"' || c == ' ' || c == '\t' || c == '\n' ||
               c == '\r';
    }

    void skip_space_and_comments() {
        while (pos_ < s_.size()) {
            char c = s_[pos_];
            if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
                ++pos_;
            } else if (c == '#') {
                // Comment to end of line.
                while (pos_ < s_.size() && s_[pos_] != '\n') ++pos_;
            } else {
                break;
            }
        }
    }

    std::string parse_quoted() {
        ++pos_;  // opening quote
        std::string out;
        while (true) {
            if (pos_ >= s_.size()) throw std::runtime_error("unterminated string in s-expr");
            char c = s_[pos_++];
            if (c == '"') break;
            if (c == '\\' && pos_ < s_.size()) {
                char e = s_[pos_++];
                switch (e) {
                    case 'n': out += '\n'; break;
                    case 't': out += '\t'; break;
                    case 'r': out += '\r'; break;
                    default: out += e; break;
                }
            } else {
                out += c;
            }
        }
        return out;
    }
};

class Parser {
  public:
    explicit Parser(const std::string& text) : lex_(text) {
        advance();
    }

    std::unique_ptr<SexprNode> run() {
        auto root = SexprNode::list_node();
        while (have_) {
            root->children.push_back(parse_node());
        }
        return root;
    }

  private:
    Lexer lex_;
    bool have_ = false;
    std::string token_;
    bool is_paren_ = false;

    void advance() { have_ = lex_.next(token_, is_paren_); }

    std::unique_ptr<SexprNode> parse_node() {
        if (!have_) throw std::runtime_error("unexpected end of s-expr");
        if (is_paren_) {
            if (token_ == "(") {
                advance();
                auto list = SexprNode::list_node();
                while (have_ && !(is_paren_ && token_ == ")")) {
                    list->children.push_back(parse_node());
                }
                if (!have_) throw std::runtime_error("unbalanced parens in s-expr");
                advance();  // consume ')'
                return list;
            }
            throw std::runtime_error("unexpected ')' in s-expr");
        }
        auto node = SexprNode::atom_node(token_);
        advance();
        return node;
    }
};

}  // namespace

std::unique_ptr<SexprNode> parse_sexpr(const std::string& text) { return Parser(text).run(); }

}  // namespace copperline
