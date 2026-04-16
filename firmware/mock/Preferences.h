#pragma once
// File-backed Preferences shim for mock build.
// Stores key-value pairs in ~/.bark-buddy/mock/<namespace>.json.
#include <string>
#include <unordered_map>
#include <fstream>
#include <cstdlib>
#include <filesystem>
#include <cstdio>

class Preferences {
    std::string _ns;
    std::unordered_map<std::string, int32_t> _cache;
    std::filesystem::path _path;

    std::filesystem::path _state_dir() {
        const char* home = getenv("HOME");
        auto p = std::filesystem::path(home ? home : "/tmp") / ".bark-buddy" / "mock";
        std::filesystem::create_directories(p);
        return p;
    }

    void _load() {
        _path = _state_dir() / (_ns + ".json");
        std::ifstream f(_path);
        if (!f.is_open()) return;
        // Simple hand-parsed JSON: {"key":value,...}
        std::string line, content((std::istreambuf_iterator<char>(f)),
                                   std::istreambuf_iterator<char>());
        size_t i = 0;
        auto skip_ws = [&]() { while (i < content.size() && (content[i]==' '||content[i]=='\n'||content[i]=='\r'||content[i]=='\t')) i++; };
        skip_ws();
        if (i < content.size() && content[i] == '{') i++;
        while (i < content.size()) {
            skip_ws();
            if (content[i] == '}') break;
            if (content[i] == ',') { i++; continue; }
            if (content[i] != '"') { i++; continue; }
            i++;
            size_t ks = i;
            while (i < content.size() && content[i] != '"') i++;
            std::string key = content.substr(ks, i - ks);
            i++;  // closing "
            skip_ws();
            if (i < content.size() && content[i] == ':') i++;
            skip_ws();
            size_t vs = i;
            while (i < content.size() && content[i] != ',' && content[i] != '}') i++;
            try { _cache[key] = std::stoi(content.substr(vs, i - vs)); } catch (...) {}
        }
    }

    void _save() {
        std::ofstream f(_path);
        f << "{\n";
        bool first = true;
        for (auto& [k, v] : _cache) {
            if (!first) f << ",\n";
            f << "  \"" << k << "\": " << v;
            first = false;
        }
        f << "\n}\n";
    }

public:
    void begin(const char* ns, bool = false) { _ns = ns; _load(); }
    void end() { _save(); }
    int32_t getInt(const char* key, int32_t def = 0) {
        auto it = _cache.find(key); return it != _cache.end() ? it->second : def;
    }
    void putInt(const char* key, int32_t val) { _cache[key] = val; _save(); }
    bool isKey(const char* key) { return _cache.count(key) > 0; }
};
