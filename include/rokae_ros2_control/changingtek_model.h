#pragma once

#include <algorithm>
#include <cctype>
#include <optional>
#include <string>

namespace rokae_ros2_control::changingtek {
struct Model {
    const char* name;
    int close_position;
    int feedback_register;
    int max_velocity;
    int max_torque;
};

// Values match the existing Marvin ChangingTek model definitions. Position
// commands are normalized (0 closed, 1 open), not physical opening distances.
inline std::optional<Model> modelFor(std::string type) {
    std::transform(type.begin(), type.end(), type.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (type == "ag2f90_c" || type == "ag2f90c" || type == "ag2f90" || type == "changingtek90c") {
        return Model{"AG2F90C", 9000, 0x060D, 255, 255};
    }
    if (type == "ag2f90_d" || type == "ag2f90d" || type == "changingtek90d") {
        return Model{"AG2F90D", 9000, 0x0418, 255, 255};
    }
    if (type == "ag2f120s") return Model{"AG2F120S", 12000, 0x060D, 100, 100};
    if (type == "ag2f120s_d") return Model{"AG2F120S-D", 1000, 0x0418, 100, 100};
    return std::nullopt;
}
} // namespace rokae_ros2_control::changingtek
