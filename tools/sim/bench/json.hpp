// Minimal JSON reader: returns the top-level numeric members of an object; strings, booleans,
// nulls, arrays and nested objects are parsed and skipped. Standard library only.
#pragma once
#include <cctype>
#include <cstdlib>
#include <map>
#include <stdexcept>
#include <string>

namespace sim {
class JsonFlat {
    public:
        static std::map<std::string, double> parse(const std::string& text) {
            JsonFlat j(text);
            std::map<std::string, double> out;
            j.ws();
            j.expect('{');
            j.ws();
            if (j.peek() == '}') return out;
            while (true) {
                j.ws();
                std::string key = j.str();
                j.ws();
                j.expect(':');
                j.ws();
                const char c = j.peek();
                if (c == '-' || std::isdigit(static_cast<unsigned char>(c))) out[key] = j.num();
                else j.skip();
                j.ws();
                if (j.peek() == ',') {
                    j.i++;
                    continue;
                }
                j.expect('}');
                break;
            }
            return out;
        }

    private:
        explicit JsonFlat(const std::string& t)
            : s(t) {}
        const std::string& s;
        size_t i = 0;
        char peek() const { return i < s.size() ? s[i] : '\0'; }
        void ws() {
            while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) i++;
        }
        void expect(char c) {
            if (peek() != c) throw std::runtime_error(std::string("robot.json: expected '") + c + "' at offset " + std::to_string(i));
            i++;
        }
        std::string str() {
            expect('"');
            std::string out;
            while (i < s.size() && s[i] != '"') {
                if (s[i] == '\\' && i + 1 < s.size()) i++;
                out += s[i++];
            }
            expect('"');
            return out;
        }
        double num() {
            const char* b = s.c_str() + i;
            char* e = nullptr;
            const double v = std::strtod(b, &e);
            if (e == b) throw std::runtime_error("robot.json: bad number at offset " + std::to_string(i));
            i += size_t(e - b);
            return v;
        }
        void skip() {
            ws();
            const char c = peek();
            if (c == '"') {
                str();
            } else if (c == '{' || c == '[') {
                const char close = c == '{' ? '}' : ']';
                i++;
                ws();
                if (peek() == close) {
                    i++;
                    return;
                }
                while (true) {
                    ws();
                    if (c == '{') {
                        str();
                        ws();
                        expect(':');
                    }
                    skip();
                    ws();
                    if (peek() == ',') {
                        i++;
                        continue;
                    }
                    expect(close);
                    break;
                }
            } else if (c == '-' || std::isdigit(static_cast<unsigned char>(c))) {
                num();
            } else {
                while (i < s.size() && std::isalpha(static_cast<unsigned char>(s[i]))) i++; // true/false/null
            }
        }
};
} // namespace sim
