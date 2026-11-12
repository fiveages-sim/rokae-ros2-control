#include "shutdown_guard.h"

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <dlfcn.h>
#include <exception>
#include <filesystem>
#include <poll.h>
#include <spawn.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

extern char** environ;

namespace rokae_ros2_control::detail {
namespace {
std::string helperPath() {
    Dl_info info{};
    if (!dladdr(reinterpret_cast<void*>(&helperPath), &info) || !info.dli_fname) return {};
    const auto directory = std::filesystem::absolute(info.dli_fname).parent_path();
    // Installed ROS layout, followed by the CMake build-tree layout.
    for (const auto& path : {directory / "rokae_ros2_control" / "rokae_shutdown_guard",
                             directory / "rokae_shutdown_guard"}) {
        if (access(path.c_str(), X_OK) == 0) return path.string();
    }
    return {};
}

[[noreturn]] void unavailable() {
    std::fprintf(stderr, "Rokae shutdown watchdog unavailable; refusing unsafe SDK destruction\n");
    std::_Exit(EXIT_FAILURE);
}
} // namespace

ShutdownGuard::ShutdownGuard(const std::vector<ShutdownEndpoint>& endpoints,
                             std::chrono::milliseconds timeout,
                             std::chrono::milliseconds emergency_timeout,
                             std::string executable)
    : uncaught_on_entry_(std::uncaught_exceptions()) {
    if (executable.empty()) executable = helperPath();
    if (executable.empty()) unavailable();
    int sockets[2];
    if (socketpair(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0, sockets) != 0) unavailable();
    std::vector<std::string> arguments{executable, "--watchdog", "3", std::to_string(getpid()),
                                      std::to_string(timeout.count()), std::to_string(emergency_timeout.count())};
    for (const auto& endpoint : endpoints) {
        arguments.push_back(endpoint.remote);
        arguments.push_back(endpoint.local);
    }
    std::vector<char*> argv;
    for (auto& argument : arguments) argv.push_back(argument.data());
    argv.push_back(nullptr);
    posix_spawn_file_actions_t actions;
    posix_spawnattr_t attributes;
    if (posix_spawn_file_actions_init(&actions) != 0) unavailable();
    if (posix_spawnattr_init(&attributes) != 0) unavailable();
    // Close the parent channel first: it may occupy the destination fd 3.
    int rc = posix_spawn_file_actions_addclose(&actions, sockets[0]);
    if (!rc) rc = posix_spawn_file_actions_adddup2(&actions, sockets[1], 3);
    if (!rc) rc = posix_spawn_file_actions_addclosefrom_np(&actions, 4);
    if (!rc) rc = posix_spawnattr_setflags(&attributes, POSIX_SPAWN_SETSID);
    if (!rc) rc = posix_spawn(&child_, executable.c_str(), &actions, &attributes, argv.data(), environ);
    posix_spawn_file_actions_destroy(&actions);
    posix_spawnattr_destroy(&attributes);
    close(sockets[1]);
    channel_ = sockets[0];
    if (rc) unavailable();
    // The helper confirms it has a pidfd for this exact process before cleanup.
    pollfd ready{channel_, POLLIN, 0};
    char response = 0;
    int polled;
    do { polled = poll(&ready, 1, 1000); } while (polled < 0 && errno == EINTR);
    if (polled <= 0 || recv(channel_, &response, 1, 0) != 1 || response != 'R') unavailable();
}

ShutdownGuard::~ShutdownGuard() {
    // Unexpected exceptions must not disarm supervision while unwinding into
    // SDK members that cleanup has not yet made safe to destroy.
    if (std::uncaught_exceptions() > uncaught_on_entry_) fail();
    const char done = 'D';
    // Never turn a supervisor failure into SIGPIPE in the controller manager.
    if (send(channel_, &done, 1, MSG_NOSIGNAL) != 1) unavailable();
    close(channel_);
    int status = 0;
    pid_t result;
    do { result = waitpid(child_, &status, 0); } while (result < 0 && errno == EINTR);
    if (result != child_ || !WIFEXITED(status) || WEXITSTATUS(status) != 0) unavailable();
}

[[noreturn]] void ShutdownGuard::fail() {
    const char failed = 'F';
    if (send(channel_, &failed, 1, MSG_NOSIGNAL) != 1) unavailable();
    // The watchdog attempts emergency power-off and SIGKILLs this process.
    // Do not unwind into SDK or joinable-thread destructors in the meantime.
    for (;;) pause();
}
} // namespace rokae_ros2_control::detail
