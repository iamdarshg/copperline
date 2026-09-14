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

// Throws std::runtime_error with offset info on malformed input.
JsonValue parse_json(const std::string& text);

// Compact by default; pretty mode indents with two spaces.
std::string serialize_json(const JsonValue& v, bool pretty = false);

}  // namespace copperline
