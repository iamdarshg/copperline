// Copperline: minimal dependency-free JSON value, parser and serializer.
//
// Supports the JSON subset used by board files, configs and reports:
// null, bool, number, string, array, object. Objects use std::map so
// serialization key order (and therefore report bytes) is deterministic.
#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <variant>
#include <vector>

namespace copperline {

struct JsonValue;
using JsonArray = std::vector<JsonValue>;
using JsonObject = std::map<std::string, JsonValue>;

struct JsonValue {
    using Storage = std::variant<std::nullptr_t, bool, double, std::string, JsonArray, JsonObject>;
    Storage data = nullptr;

    JsonValue() = default;
    JsonValue(std::nullptr_t) : data(nullptr) {}
    JsonValue(bool b) : data(b) {}
    JsonValue(int i) : data(static_cast<double>(i)) {}
    JsonValue(long long i) : data(static_cast<double>(i)) {}
    JsonValue(double d) : data(d) {}
    JsonValue(const char* s) : data(std::string(s)) {}
    JsonValue(const std::string& s) : data(s) {}
    JsonValue(const JsonArray& a) : data(a) {}
    JsonValue(const JsonObject& o) : data(o) {}

    static JsonValue array() { return JsonValue(JsonArray{}); }
    static JsonValue object() { return JsonValue(JsonObject{}); }

    bool is_null() const { return std::holds_alternative<std::nullptr_t>(data); }
    bool is_bool() const { return std::holds_alternative<bool>(data); }
    bool is_number() const { return std::holds_alternative<double>(data); }
    bool is_string() const { return std::holds_alternative<std::string>(data); }
    bool is_array() const { return std::holds_alternative<JsonArray>(data); }
    bool is_object() const { return std::holds_alternative<JsonObject>(data); }

    bool as_bool(bool fallback = false) const;
    double as_number(double fallback = 0.0) const;
    const std::string& as_string() const;
    const JsonArray& as_array() const;
    const JsonObject& as_object() const;
    JsonArray& as_array();
    JsonObject& as_object();

    const JsonValue* find(const std::string& key) const;
    bool has(const std::string& key) const { return find(key) != nullptr; }

    double get_number(const std::string& key, double fallback) const;
    std::string get_string(const std::string& key, const std::string& fallback = "") const;
    bool get_bool(const std::string& key, bool fallback) const;

    JsonValue& operator[](const std::string& key);
};

// D5: common JSON helpers consolidating the ~20x to_json boilerplate.
// All helpers produce byte-identical values to the manual sequences they
// replace (ints stored as doubles, same insertion content; objects use
// std::map so key order is always sorted at serialization).
JsonValue json_string_array(const std::vector<std::string>& v);
JsonValue json_double_array(const std::vector<double>& v);
// Integer IDs stored as JSON numbers (double), matching the existing
// static_cast<double>(id) convention in every to_json call site.
JsonValue json_int_array(const std::vector<int>& v);
JsonValue json_int_array(const std::vector<long long>& v);
// Millimetre / coordinate field helpers (thin, documents intent; the
// caller still performs nm_to_mm so arithmetic is unchanged).
inline void json_add_mm(JsonValue& o, const std::string& key, double mm) { o[key] = mm; }
inline void json_add_point_mm(JsonValue& o, const std::string& xk,
                              const std::string& yk, double x_mm,
                              double y_mm) {
    o[xk] = x_mm;
    o[yk] = y_mm;
}

// Throws std::runtime_error with offset info on malformed input.
JsonValue parse_json(const std::string& text);

// Compact by default; pretty mode indents with two spaces.
std::string serialize_json(const JsonValue& v, bool pretty = false);

}  // namespace copperline
