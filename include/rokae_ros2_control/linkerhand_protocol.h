#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace rokae_ros2_control::linkerhand {

// ROS joint order; O7 register order is pitch, yaw, index, middle, ring, pinky, roll.
struct Model {
    std::string name;
    std::vector<std::string> suffixes;
    std::vector<int> registers;
    std::vector<double> upper_limits;

    size_t jointCount() const { return suffixes.size(); }
    int torqueRegister() const { return static_cast<int>(jointCount()); }
    int speedRegister() const { return 2 * torqueRegister(); }
};

inline bool isSupported(const std::string& type) {
    return type == "linkerhand_o6" || type == "o6" ||
           type == "linkerhand_o7" || type == "o7" ||
           type == "linkerhand_l6" || type == "l6";
}

inline Model modelFor(const std::string& type, bool right) {
    const std::vector<std::string> six = {
        "thumb_joint1", "thumb_joint2", "index_joint", "middle_joint", "ring_joint", "pinky_joint"};
    if (type == "linkerhand_l6" || type == "l6") {
        return {"L6", six, {0, 1, 2, 3, 4, 5}, {0.99, 1.39, 1.26, 1.26, 1.26, 1.26}};
    }
    if (type == "linkerhand_o6" || type == "o6") {
        // Match the side-dependent yaw limit in linkerhand_description/xacro/o6.xacro.
        return {"O6", six, {0, 1, 2, 3, 4, 5}, {0.58, right ? 1.36 : 1.30, 1.60, 1.60, 1.60, 1.60}};
    }
    if (type == "linkerhand_o7" || type == "o7") {
        return {"O7", {"thumb_joint1", "thumb_joint2", "thumb_joint3", "index_joint",
                       "middle_joint", "ring_joint", "pinky_joint"},
                {6, 1, 0, 2, 3, 4, 5}, {0.5146, 1.9189, 1.1339, 1.3607, 1.3607, 1.3607, 1.3607}};
    }
    throw std::invalid_argument("Unsupported LinkerHand type: " + type);
}

// Require a complete, unambiguous set of joints from one hand, independent of URDF order.
inline std::vector<std::string> orderedJoints(
    const Model& model, const std::vector<std::string>& names) {
    std::vector<std::string> result;
    std::string prefix;
    for (const auto& suffix : model.suffixes) {
        std::vector<std::string> matches;
        for (const auto& name : names) {
            if (name.size() >= suffix.size() &&
                name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0 &&
                (name.size() == suffix.size() || name[name.size() - suffix.size() - 1] == '_')) {
                matches.push_back(name);
            }
        }
        if (matches.size() != 1) throw std::invalid_argument("Missing or duplicate LinkerHand joint: " + suffix);
        const auto current_prefix = matches.front().substr(0, matches.front().size() - suffix.size());
        if (!result.empty() && prefix != current_prefix) throw std::invalid_argument("Mixed LinkerHand joint prefixes");
        prefix = current_prefix;
        result.push_back(matches.front());
    }
    return result;
}

inline int positionToRegister(double position, double upper) {
    if (!std::isfinite(position)) throw std::invalid_argument("Non-finite LinkerHand position");
    return 255 - static_cast<int>(std::lround(std::clamp(position / upper, 0.0, 1.0) * 255.0));
}

inline double registerToPosition(int raw, double upper) {
    return (255.0 - static_cast<double>(std::clamp(raw & 0xFF, 0, 255))) / 255.0 * upper;
}

// The SDK accepts at most three int16 registers; O7 needs a final one-register request.
template<class Transfer>
bool transferRegisters(int start, std::vector<int>& values, Transfer transfer) {
    for (size_t offset = 0; offset < values.size(); offset += 3) {
        const size_t count = std::min(size_t{3}, values.size() - offset);
        std::vector<int> chunk(values.begin() + offset, values.begin() + offset + count);
        if (!transfer(start + static_cast<int>(offset), chunk)) return false;
        if (chunk.size() != count) return false;
        std::copy(chunk.begin(), chunk.end(), values.begin() + offset);
    }
    return true;
}

} // namespace rokae_ros2_control::linkerhand
