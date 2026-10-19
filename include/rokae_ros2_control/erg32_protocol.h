#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace rokae_ros2_control::erg32 {
constexpr double pi = 3.14159265358979323846;
constexpr double finger_travel = 0.016;
using Packet = std::array<uint16_t, 6>;

// Canonical order is rotate, grip; caller order may differ.
inline std::array<std::string, 2> orderedJoints(const std::vector<std::string>& names) {
    for (const auto& prefix : {std::string("left_hand_"), std::string("right_hand_"), std::string("")}) {
        const std::array<std::string, 2> expected{prefix + "rotate_joint", prefix + "gripper_joint"};
        if (std::count(names.begin(), names.end(), expected[0]) == 1 &&
            std::count(names.begin(), names.end(), expected[1]) == 1) {
            for (const auto& name : names) {
                if ((name.find("hand") != std::string::npos || name.find("gripper") != std::string::npos ||
                     name.find("rotate") != std::string::npos) && name != expected[0] && name != expected[1]) {
                    throw std::invalid_argument("Unexpected or mixed-side ERG32 joint: " + name);
                }
            }
            return expected;
        }
    }
    throw std::invalid_argument("ERG32 requires one rotate_joint and one gripper_joint with matching side");
}

inline int byte(double value) {
    if (!std::isfinite(value)) throw std::invalid_argument("Non-finite ERG32 dynamics");
    return static_cast<int>(std::lround(std::clamp(value, 0.0, 1.0) * 255.0));
}

inline Packet encode(const std::array<double, 2>& position, double torque, double velocity) {
    if (!std::isfinite(position[0]) || !std::isfinite(position[1])) {
        throw std::invalid_argument("Non-finite ERG32 position");
    }
    const auto angle = static_cast<int16_t>(std::lround(std::clamp(position[0], -pi, pi) * 180.0 / pi));
    const int grip = 255 - static_cast<int>(std::lround(std::clamp(position[1], 0.0, finger_travel) / finger_travel * 255.0));
    const int speed = byte(velocity);
    const int force = byte(torque);
    // 0x03EA..0x03EF. Relative position and turns remain zero; absolute trigger = 1.
    return {static_cast<uint16_t>(grip | (speed << 8)), static_cast<uint16_t>(1 | (force << 8)),
            static_cast<uint16_t>(angle), static_cast<uint16_t>(speed | (force << 8)), 0, 1};
}

struct Feedback {
    std::array<double, 2> position{};
    std::array<uint8_t, 2> status{}, fault{}, object{}, raw_velocity{}, raw_current{};
    int turns = 0;
    bool dropped = false;
    uint8_t voltage = 0, temperature = 0;
    bool ready() const { return (status[0] & 0x18) == 0x18 && (status[1] & 0x18) == 0x18; }
    bool healthy() const { return fault[0] == 0 && fault[1] == 0; }
};

inline Feedback decode(const std::array<uint16_t, 9>& r) {
    Feedback f;
    f.status = {static_cast<uint8_t>(r[1]), static_cast<uint8_t>(r[0])};
    f.fault = {static_cast<uint8_t>(r[1] >> 8), static_cast<uint8_t>(r[0] >> 8)};
    for (size_t i = 0; i < 2; ++i) f.object[i] = (f.status[i] >> 5) & 7;
    const int angle = r[4] < 0x8000 ? r[4] : static_cast<int>(r[4]) - 65536;
    f.position = {angle * pi / 180.0, (1.0 - (r[2] & 255) / 255.0) * finger_travel};
    f.raw_velocity = {static_cast<uint8_t>(r[5]), static_cast<uint8_t>(r[2] >> 8)};
    f.raw_current = {static_cast<uint8_t>(r[5] >> 8), static_cast<uint8_t>(r[3] >> 8)};
    const int raw_turns = r[6] >> 8;
    f.turns = raw_turns < 128 ? raw_turns : raw_turns - 256;
    f.dropped = (r[0] & 2) != 0;
    f.voltage = r[8] & 255;
    f.temperature = r[8] >> 8;
    return f;
}

// Keep trigger registers at the end of each axis transaction. No relative motion is sent.
// A failed transfer prevents later transfers and leaves that axis's last packet unchanged.
template<class Transfer>
bool writeMotion(const Packet& packet, Packet& last, Transfer transfer) {
    if (packet[0] != last[0] || packet[1] != last[1]) {
        std::vector<int> grip{packet[0], packet[1]};
        if (!transfer(0x10, 0x03EA, grip)) return false;
        last[0] = packet[0]; last[1] = packet[1];
    }
    if (!std::equal(packet.begin() + 2, packet.end(), last.begin() + 2)) {
        std::vector<int> parameters{packet[2], packet[3], packet[4]};
        if (!transfer(0x10, 0x03EC, parameters)) return false;
        std::vector<int> trigger{packet[5]};
        if (!transfer(0x10, 0x03EF, trigger)) return false;
        std::copy(packet.begin() + 2, packet.end(), last.begin() + 2);
    }
    return true;
}
} // namespace rokae_ros2_control::erg32
