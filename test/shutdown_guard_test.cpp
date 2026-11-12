#include "shutdown_guard.h"
#include "bounded_processes.h"

#include <gtest/gtest.h>
#include <csignal>
#include <fcntl.h>
#include <fstream>
#include <thread>
#include <unistd.h>

namespace rokae_ros2_control::detail {
using namespace std::chrono_literals;

class ShutdownSupervision : public ::testing::Test {
    void SetUp() override { ::testing::FLAGS_gtest_death_test_style = "threadsafe"; }
};

TEST_F(ShutdownSupervision, CompletedCleanupDisarmsSupervisor) {
    ShutdownGuard guard({}, 1000ms, 100ms, SHUTDOWN_GUARD_EXECUTABLE);
}

TEST_F(ShutdownSupervision, HungCleanupTerminatesProcessWithoutUnwinding) {
    // This is a real subprocess supervisor, with no robot endpoints. A stuck
    // destructor must never run when the watchdog terminates its parent.
    EXPECT_EXIT({
        struct NeverDestroy { ~NeverDestroy() { for (;;) pause(); } } object;
        ShutdownGuard guard({}, 80ms, 100ms, SHUTDOWN_GUARD_EXECUTABLE);
        for (;;) pause();
    }, ::testing::KilledBySignal(SIGKILL), "terminating controller process");
}

TEST_F(ShutdownSupervision, UnresolvedWorkerTriggersEmergencyExitWithoutWaitingForFullBudget) {
    EXPECT_EXIT({
        ShutdownGuard guard({}, 3000ms, 100ms, SHUTDOWN_GUARD_EXECUTABLE);
        guard.fail();
    }, ::testing::KilledBySignal(SIGKILL), "shutdown incomplete/timed out");
}

TEST_F(ShutdownSupervision, FailedHelperStartupDoesNotUnwindIntoSdkDestructors) {
    EXPECT_EXIT({
        struct NeverDestroy { ~NeverDestroy() { for (;;) pause(); } } object;
        ShutdownGuard guard({}, 100ms, 100ms, "/nonexistent/rokae_shutdown_guard");
    }, ::testing::ExitedWithCode(EXIT_FAILURE), "watchdog unavailable");
}

TEST_F(ShutdownSupervision, UnexpectedCleanupExceptionDoesNotDisarmSupervisor) {
    EXPECT_EXIT({
        try {
            ShutdownGuard guard({}, 1000ms, 100ms, SHUTDOWN_GUARD_EXECUTABLE);
            throw std::runtime_error("unexpected cleanup exception");
        } catch (...) {
            std::_Exit(2);
        }
    }, ::testing::KilledBySignal(SIGKILL), "terminating controller process");
}

TEST_F(ShutdownSupervision, ParentDeathStillInvokesEmergencyPath) {
    // GTest reads its captured stderr as soon as the victim dies, before the
    // independent watchdog has necessarily logged. Observe a separate file.
    // The environment carries the same temporary path into GTest's re-exec.
    std::string path;
    if (const char* inherited = std::getenv("ROKAE_GUARD_TEST_LOG")) {
        path = inherited;
    } else {
        char pattern[] = "/tmp/rokae_guard_test_XXXXXX";
        const int file = mkstemp(pattern);
        ASSERT_GE(file, 0);
        close(file);
        path = pattern;
        setenv("ROKAE_GUARD_TEST_LOG", path.c_str(), 1);
    }
    EXPECT_EXIT({
        const int file = open(path.c_str(), O_WRONLY | O_TRUNC);
        if (file < 0 || dup2(file, STDERR_FILENO) < 0) std::_Exit(2);
        close(file);
        ShutdownGuard guard({}, 3000ms, 100ms, SHUTDOWN_GUARD_EXECUTABLE);
        std::_Exit(17);
    }, ::testing::ExitedWithCode(17), "");
    bool observed = false;
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (!observed && std::chrono::steady_clock::now() < deadline) {
        std::ifstream file(path);
        const std::string contents((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        observed = contents.find("terminating controller process") != std::string::npos;
        if (!observed) std::this_thread::sleep_for(10ms);
    }
    EXPECT_TRUE(observed);
    unlink(path.c_str());
    unsetenv("ROKAE_GUARD_TEST_LOG");
}

TEST_F(ShutdownSupervision, HungPowerOffChildDoesNotPreventOtherAttemptAndDeadline) {
    const auto start = std::chrono::steady_clock::now();
    const auto attempts = runBoundedProcesses({{"/bin/sleep", "60"}, {"/bin/true"}}, 100ms);
    const auto elapsed = std::chrono::steady_clock::now() - start;
    ASSERT_EQ(attempts.size(), 2u);
    EXPECT_EQ(attempts[0].outcome, ProcessOutcome::timed_out);
    EXPECT_EQ(attempts[1].outcome, ProcessOutcome::succeeded);
    EXPECT_LT(elapsed, 2s);
    // The production supervisor exits immediately; reap the known sleep child
    // here so a test run does not leave zombies in its own process.
    waitpid(attempts[0].pid, nullptr, 0);
}

TEST_F(ShutdownSupervision, FailedChildOrSpawnDoesNotSkipOtherAttempt) {
    const auto attempts = runBoundedProcesses(
        {{"/bin/false"}, {"/nonexistent/poweroff"}, {"/bin/true"}}, 1000ms);
    ASSERT_EQ(attempts.size(), 3u);
    EXPECT_EQ(attempts[0].outcome, ProcessOutcome::failed);
    EXPECT_EQ(attempts[1].outcome, ProcessOutcome::spawn_failed);
    EXPECT_EQ(attempts[2].outcome, ProcessOutcome::succeeded);
}
} // namespace rokae_ros2_control::detail
