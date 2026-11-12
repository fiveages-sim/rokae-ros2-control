#pragma once

#include <cerrno>
#include <chrono>
#include <csignal>
#include <poll.h>
#include <spawn.h>
#include <string>
#include <sys/wait.h>
#include <vector>

extern char** environ;

namespace rokae_ros2_control::detail {
enum class ProcessOutcome { succeeded, failed, timed_out, spawn_failed };
struct ProcessAttempt {
    pid_t pid{-1};
    ProcessOutcome outcome{ProcessOutcome::spawn_failed};
};

// Start every command before waiting on any of them. The shared deadline also
// bounds a dead SDK child; never block in waitpid() after sending SIGKILL.
inline std::vector<ProcessAttempt> runBoundedProcesses(
    std::vector<std::vector<std::string>> commands, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    std::vector<ProcessAttempt> attempts(commands.size());
    std::vector<bool> running(commands.size(), false);
    size_t pending = 0;
    for (size_t i = 0; i < commands.size(); ++i) {
        if (commands[i].empty()) continue;
        std::vector<char*> argv;
        for (auto& argument : commands[i]) argv.push_back(argument.data());
        argv.push_back(nullptr);
        if (posix_spawn(&attempts[i].pid, argv[0], nullptr, nullptr, argv.data(), environ) == 0) {
            running[i] = true;
            ++pending;
        }
    }
    while (pending && std::chrono::steady_clock::now() < deadline) {
        for (size_t i = 0; i < attempts.size(); ++i) {
            if (!running[i]) continue;
            int status = 0;
            const auto result = waitpid(attempts[i].pid, &status, WNOHANG);
            if (result == attempts[i].pid || (result < 0 && errno == ECHILD)) {
                attempts[i].outcome = result > 0 && WIFEXITED(status) && WEXITSTATUS(status) == 0 ?
                    ProcessOutcome::succeeded : ProcessOutcome::failed;
                running[i] = false;
                --pending;
            }
        }
        if (pending) poll(nullptr, 0, 10);
    }
    for (size_t i = 0; i < attempts.size(); ++i) {
        if (!running[i]) continue;
        attempts[i].outcome = ProcessOutcome::timed_out;
        kill(attempts[i].pid, SIGKILL);
        waitpid(attempts[i].pid, nullptr, WNOHANG);
    }
    return attempts;
}
} // namespace rokae_ros2_control::detail
