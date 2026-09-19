#pragma once

#include <algorithm>
#include <cctype>
#include <fstream>
#include <stdexcept>
#include <string>

// Run native diagnostics from the project root, where config.txt lives.
inline std::string configured_lidar_port() {
    std::ifstream config("config.txt");
    if (!config) {
        throw std::runtime_error("Missing config.txt. Run from the project root and copy example_config.txt to config.txt.");
    }
    const auto trim = [](const std::string& value) -> std::string {
        const auto start = value.find_first_not_of(" \t\r\n");
        if (start == std::string::npos) return "";
        return value.substr(start, value.find_last_not_of(" \t\r\n") - start + 1);
    };
    std::string port = "/dev/ttyUSB0";
    std::string line;
    while (std::getline(config, line)) {
        line = line.substr(0, line.find('#'));
        const auto equals = line.find('=');
        if (equals == std::string::npos) continue;
        auto key = trim(line.substr(0, equals));
        std::transform(key.begin(), key.end(), key.begin(),
                       [](unsigned char c) { return std::tolower(c); });
        if (key == "lidar_port") port = trim(line.substr(equals + 1));
    }
    if (port.empty()) {
        throw std::runtime_error("config.txt: lidar_port must be a non-empty serial device path");
    }
    return port;
}
