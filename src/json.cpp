#include "router/json.h"

#include <cmath>
#include <cstdio>
#include <sstream>
#include <stdexcept>

namespace copperline {

bool JsonValue::as_bool(bool fallback) const {
    if (auto* p = std::get_if<bool>(&data)) return *p;
    return fallback;
}

double JsonValue::as_number(double fallback) const {
    if (auto* p = std::get_if<double>(&data)) return *p;
    return fallback;
}

const std::string& JsonValue::as_string() const {
    static const std::string kEmpty;
    if (auto* p = std::get_if<std::string>(&data)) return *p;
    return kEmpty;
}

const JsonArray& JsonValue::as_array() const {
    static const JsonArray kEmpty;
    if (auto* p = std::get_if<JsonArray>(&data)) return *p;
    return kEmpty;
}

const JsonObject& JsonValue::as_object() const {
    static const JsonObject kEmpty;
    if (auto* p = std::get_if<JsonObject>(&data)) return *p;
    return kEmpty;
}

JsonArray& JsonValue::as_array() {
    if (!is_array()) data = JsonArray{};
    return std::get<JsonArray>(data);
}

JsonObject& JsonValue::as_object() {
    if (!is_object()) data = JsonObject{};
    return std::get<JsonObject>(data);
}

const JsonValue* JsonValue::find(const std::string& key) const {
    if (!is_object()) return nullptr;
    const auto& o = std::get<JsonObject>(data);
    auto it = o.find(key);
    return it == o.end() ? nullptr : &it->second;
}

double JsonValue::get_number(const std::string& key, double fallback) const {
    const JsonValue* v = find(key);
    return v ? v->as_number(fallback) : fallback;
}

std::string JsonValue::get_string(const std::string& key, const std::string& fallback) const {
    const JsonValue* v = find(key);
    if (!v || !v->is_string()) return fallback;
    return v->as_string();
}

bool JsonValue::get_bool(const std::string& key, bool fallback) const {
    const JsonValue* v = find(key);
    return v ? v->as_bool(fallback) : fallback;
}

JsonValue& JsonValue::operator[](const std::string& key) { return as_object()[key]; }

namespace {

class Parser {
  public:
    explicit Parser(const std::string& text) : s_(text) {}

    JsonValue run() {
        skip_ws();
        JsonValue v = parse_value();
        skip_ws();
        if (pos_ != s_.size()) fail("trailing characters");
        return v;
    }

  private:
    const std::string& s_;
    std::size_t pos_ = 0;

    [[noreturn]] void fail(const std::string& msg) {
        throw std::runtime_error("JSON parse error at offset " + std::to_string(pos_) + ": " + msg);
    }

    void skip_ws() {
        while (pos_ < s_.size() &&
               (s_[pos_] == ' ' || s_[pos_] == '\t' || s_[pos_] == '\n' || s_[pos_] == '\r'))
            ++pos_;
    }

    char peek() {
        if (pos_ >= s_.size()) fail("unexpected end of input");
        return s_[pos_];
    }

    void expect(char c) {
        if (peek() != c) fail(std::string("expected '") + c + "'");
        ++pos_;
    }

    void expect_literal(const char* lit) {
        for (const char* p = lit; *p; ++p) {
            if (pos_ >= s_.size() || s_[pos_] != *p) fail("bad literal");
            ++pos_;
        }
    }

    JsonValue parse_value() {
        char c = peek();
        if (c == '{') return parse_object();
        if (c == '[') return parse_array();
        if (c == '"') return JsonValue(parse_string());
        if (c == 't') {
            expect_literal("true");
            return JsonValue(true);
        }
        if (c == 'f') {
            expect_literal("false");
            return JsonValue(false);
        }
        if (c == 'n') {
            expect_literal("null");
            return JsonValue();
        }
        return JsonValue(parse_number());
    }

    JsonValue parse_object() {
        expect('{');
        JsonObject obj;
        skip_ws();
        if (peek() == '}') {
            ++pos_;
            return JsonValue(obj);
        }
        while (true) {
            skip_ws();
            if (peek() != '"') fail("expected string key");
            std::string key = parse_string();
            skip_ws();
            expect(':');
            skip_ws();
            obj[key] = parse_value();
            skip_ws();
            char c = peek();
            if (c == ',') {
                ++pos_;
                continue;
            }
            if (c == '}') {
                ++pos_;
                break;
            }
            fail("expected ',' or '}'");
        }
        return JsonValue(obj);
    }

    JsonValue parse_array() {
        expect('[');
        JsonArray arr;
        skip_ws();
        if (peek() == ']') {
            ++pos_;
            return JsonValue(arr);
        }
        while (true) {
            skip_ws();
            arr.push_back(parse_value());
            skip_ws();
            char c = peek();
            if (c == ',') {
                ++pos_;
                continue;
            }
            if (c == ']') {
                ++pos_;
                break;
            }
            fail("expected ',' or ']'");
        }
        return JsonValue(arr);
    }

    std::string parse_string() {
        expect('"');
        std::string out;
        while (true) {
            if (pos_ >= s_.size()) fail("unterminated string");
            char c = s_[pos_++];
            if (c == '"') break;
            if (c == '\\') {
                if (pos_ >= s_.size()) fail("unterminated escape");
                char e = s_[pos_++];
                switch (e) {
                    case '"': out += '"'; break;
                    case '\\': out += '\\'; break;
                    case '/': out += '/'; break;
                    case 'b': out += '\b'; break;
                    case 'f': out += '\f'; break;
                    case 'n': out += '\n'; break;
                    case 'r': out += '\r'; break;
                    case 't': out += '\t'; break;
                    case 'u': {
                        if (pos_ + 4 > s_.size()) fail("bad \\u escape");
                        unsigned code = 0;
                        for (int i = 0; i < 4; ++i) {
                            char h = s_[pos_++];
                            code <<= 4;
                            if (h >= '0' && h <= '9') code |= static_cast<unsigned>(h - '0');
                            else if (h >= 'a' && h <= 'f') code |= static_cast<unsigned>(h - 'a' + 10);
                            else if (h >= 'A' && h <= 'F') code |= static_cast<unsigned>(h - 'A' + 10);
                            else fail("bad hex digit");
                        }
                        // Encode as UTF-8 (BMP only; sufficient for board files).
                        if (code < 0x80) {
                            out += static_cast<char>(code);
                        } else if (code < 0x800) {
                            out += static_cast<char>(0xC0 | (code >> 6));
                            out += static_cast<char>(0x80 | (code & 0x3F));
                        } else {
                            out += static_cast<char>(0xE0 | (code >> 12));
                            out += static_cast<char>(0x80 | ((code >> 6) & 0x3F));
                            out += static_cast<char>(0x80 | (code & 0x3F));
                        }
                        break;
                    }
                    default: fail("bad escape");
                }
            } else {
                out += c;
            }
        }
        return out;
    }

    double parse_number() {
        std::size_t start = pos_;
        if (pos_ < s_.size() && (s_[pos_] == '-' || s_[pos_] == '+')) ++pos_;
        bool any = false;
        while (pos_ < s_.size() && s_[pos_] >= '0' && s_[pos_] <= '9') {
            ++pos_;
            any = true;
        }
        if (pos_ < s_.size() && s_[pos_] == '.') {
            ++pos_;
            while (pos_ < s_.size() && s_[pos_] >= '0' && s_[pos_] <= '9') {
                ++pos_;
                any = true;
            }
        }
        if (pos_ < s_.size() && (s_[pos_] == 'e' || s_[pos_] == 'E')) {
            ++pos_;
            if (pos_ < s_.size() && (s_[pos_] == '-' || s_[pos_] == '+')) ++pos_;
            while (pos_ < s_.size() && s_[pos_] >= '0' && s_[pos_] <= '9') ++pos_;
        }
        if (!any) fail("expected value");
        return std::stod(s_.substr(start, pos_ - start));
    }
};

void write_string(std::string& out, const std::string& s) {
    out += '"';
    char buf[8];
    for (char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\b': out += "\\b"; break;
            case '\f': out += "\\f"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                    out += buf;
                } else {
                    out += c;
                }
        }
    }
    out += '"';
}

void write_number(std::string& out, double d) {
    if (std::isnan(d) || std::isinf(d)) {
        out += "null";
        return;
    }
    if (d == std::trunc(d) && std::fabs(d) < 9.0e15) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%lld", static_cast<long long>(d));
        out += buf;
        return;
    }
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.9g", d);
    out += buf;
}

void write_value(std::string& out, const JsonValue& v, bool pretty, int indent) {
    const auto& d = v.data;
    if (std::holds_alternative<std::nullptr_t>(d)) {
        out += "null";
    } else if (auto* p = std::get_if<bool>(&d)) {
        out += *p ? "true" : "false";
    } else if (auto* p = std::get_if<double>(&d)) {
        write_number(out, *p);
    } else if (auto* p = std::get_if<std::string>(&d)) {
        write_string(out, *p);
    } else if (auto* p = std::get_if<JsonArray>(&d)) {
        out += '[';
        bool first = true;
        for (const auto& e : *p) {
            if (!first) out += pretty ? ",\n" + std::string(indent + 1, ' ') : ",";
            else if (pretty && !p->empty()) out += "\n" + std::string(indent + 1, ' ');
            if (!first || !pretty) {
                // spacing handled above
            }
            if (!pretty && !first) {
                // already added comma
            }
            write_value(out, e, pretty, indent + 1);
            first = false;
        }
        if (pretty && !p->empty()) out += "\n" + std::string(indent, ' ');
        out += ']';
    } else if (auto* p = std::get_if<JsonObject>(&d)) {
        out += '{';
        bool first = true;
        for (const auto& [k, e] : *p) {
            if (!first) out += pretty ? ",\n" : ",";
            else if (pretty && !p->empty()) out += "\n";
            if (pretty) out += std::string(indent + 1, ' ');
            write_string(out, k);
            out += pretty ? ": " : ":";
            write_value(out, e, pretty, indent + 1);
            first = false;
        }
        if (pretty && !p->empty()) out += "\n" + std::string(indent, ' ');
        out += '}';
    }
}

}  // namespace

JsonValue parse_json(const std::string& text) { return Parser(text).run(); }

// D5: shared array builders (single home for the repeated
// "JsonValue a = array(); for (...) push_back(...)" pattern).
JsonValue json_string_array(const std::vector<std::string>& v) {
    JsonValue a = JsonValue::array();
    for (const auto& s : v) a.as_array().push_back(JsonValue(s));
    return a;
}

JsonValue json_double_array(const std::vector<double>& v) {
    JsonValue a = JsonValue::array();
    for (double d : v) a.as_array().push_back(JsonValue(d));
    return a;
}

JsonValue json_int_array(const std::vector<int>& v) {
    JsonValue a = JsonValue::array();
    for (int i : v) a.as_array().push_back(JsonValue(static_cast<double>(i)));
    return a;
}

JsonValue json_int_array(const std::vector<long long>& v) {
    JsonValue a = JsonValue::array();
    for (long long i : v) a.as_array().push_back(JsonValue(static_cast<double>(i)));
    return a;
}

std::string serialize_json(const JsonValue& v, bool pretty) {
    std::string out;
    write_value(out, v, pretty, 0);
    if (pretty) out += "\n";
    return out;
}

}  // namespace copperline
