#pragma once

#include <chrono>
#include <string>
#include <vector>
#include <sys/types.h>

namespace rokae_ros2_control::detail {
struct ShutdownEndpoint {
    std::string remote;
    std::string local;
};

// A separate session supervises cleanup even if an SDK call or join blocks.
// No SDK object or thread is shared with the emergency power-off processes.
class ShutdownGuard {
public:
    explicit ShutdownGuard(const std::vector<ShutdownEndpoint>& endpoints,
                           std::chrono::milliseconds timeout = std::chrono::milliseconds(3000),
                           std::chrono::milliseconds emergency_timeout = std::chrono::milliseconds(2000),
                           std::string executable = {});
    ~ShutdownGuard();
    ShutdownGuard(const ShutdownGuard&) = delete;
    ShutdownGuard& operator=(const ShutdownGuard&) = delete;
    [[noreturn]] void fail();

private:
    int channel_{-1};
    pid_t child_{-1};
    int uncaught_on_entry_{0};
};
} // namespace rokae_ros2_control::detail
