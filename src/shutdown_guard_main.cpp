#include "rokae/robot.h"
#include "bounded_processes.h"

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <poll.h>
#include <spawn.h>
#include <string>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

extern char** environ;

namespace {
using Clock = std::chrono::steady_clock;

int powerOff(const char* remote, const char* local) {
    // Exit without SDK destructors: even disconnect/destruction may hang. This
    // process owns no other robot and the watchdog enforces its deadline.
    int result = EXIT_FAILURE;
    try {
        rokae::xMateErProRobot robot;
        robot.connectToRobot(remote, local);
        std::error_code ec;
        robot.setPowerState(false, ec);
        if (ec) {
            std::fprintf(stderr, "Rokae emergency power-off %s rejected: %s; trying Idle\n",
                         remote, ec.message().c_str());
            robot.setMotionControlMode(rokae::MotionControlMode::Idle, ec);
            if (ec) std::fprintf(stderr, "Rokae emergency Idle %s failed: %s\n", remote, ec.message().c_str());
            robot.setPowerState(false, ec);
        }
        if (ec) {
            std::fprintf(stderr, "Rokae emergency power-off %s failed: %s\n", remote, ec.message().c_str());
        } else {
            const auto state = robot.powerState(ec);
            if (!ec && state == rokae::PowerState::off) {
                std::fprintf(stderr, "Rokae emergency power-off %s confirmed\n", remote);
                result = EXIT_SUCCESS;
            } else {
                std::fprintf(stderr, "Rokae emergency power-off %s not confirmed\n", remote);
            }
        }
        std::_Exit(result);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Rokae emergency connection/power-off %s failed: %s\n", remote, e.what());
    } catch (...) {
        std::fprintf(stderr, "Rokae emergency connection/power-off %s failed\n", remote);
    }
    std::_Exit(result);
}

int watchdog(int argc, char** argv) {
    if (argc < 6 || (argc - 6) % 2 != 0) return EXIT_FAILURE;
    const int channel = std::stoi(argv[2]);
    const pid_t parent = std::stoi(argv[3]);
    const int timeout_ms = std::stoi(argv[4]);
    const int emergency_ms = std::stoi(argv[5]);
    if (parent != getppid() || timeout_ms <= 0 || emergency_ms <= 0) return EXIT_FAILURE;
    // A pidfd prevents signalling an unrelated process after PID reuse.
    const int parent_fd = static_cast<int>(syscall(SYS_pidfd_open, parent, 0));
    if (parent_fd < 0) return EXIT_FAILURE;
    if (fcntl(channel, F_SETFD, FD_CLOEXEC) < 0) return EXIT_FAILURE;
    const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
    const char ready = 'R';
    if (send(channel, &ready, 1, MSG_NOSIGNAL) != 1) return EXIT_FAILURE;
    for (;;) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()).count();
        if (remaining <= 0) break;
        pollfd input{channel, POLLIN, 0};
        const int rc = poll(&input, 1, static_cast<int>(remaining));
        if (rc < 0 && errno == EINTR) continue;
        if (rc <= 0) break;
        char message = 0;
        const auto received = recv(channel, &message, 1, 0);
        if (received == 1 && message == 'D') return EXIT_SUCCESS;
        // Failure or EOF (e.g. parent crash) invokes emergency power-off too.
        break;
    }
    std::fprintf(stderr, "Rokae shutdown incomplete/timed out; emergency power-off before process termination\n");
    std::vector<std::vector<std::string>> commands;
    for (int i = 6; i < argc; i += 2) {
        commands.push_back({argv[0], "--power-off", argv[i], argv[i + 1]});
    }
    const auto attempts = rokae_ros2_control::detail::runBoundedProcesses(
        std::move(commands), std::chrono::milliseconds(emergency_ms));
    for (size_t i = 0; i < attempts.size(); ++i) {
        using Outcome = rokae_ros2_control::detail::ProcessOutcome;
        if (attempts[i].outcome == Outcome::succeeded) continue;
        const char* reason = attempts[i].outcome == Outcome::timed_out ? "timed out" :
            attempts[i].outcome == Outcome::spawn_failed ? "could not start" : "failed";
        std::fprintf(stderr, "Rokae emergency power-off %s %s; physical state unconfirmed\n", argv[6 + i * 2], reason);
    }
    std::fprintf(stderr, "Rokae shutdown watchdog terminating controller process %d without SDK destruction\n", parent);
    if (syscall(SYS_pidfd_send_signal, parent_fd, SIGKILL, nullptr, 0) < 0 && errno != ESRCH) {
        std::fprintf(stderr, "Rokae watchdog could not terminate controller: errno %d\n", errno);
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
} // namespace

int main(int argc, char** argv) {
    try {
        if (argc == 4 && std::string(argv[1]) == "--power-off") return powerOff(argv[2], argv[3]);
        if (argc >= 2 && std::string(argv[1]) == "--watchdog") std::_Exit(watchdog(argc, argv));
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Rokae shutdown helper failed: %s\n", e.what());
    }
    std::_Exit(EXIT_FAILURE);
}
