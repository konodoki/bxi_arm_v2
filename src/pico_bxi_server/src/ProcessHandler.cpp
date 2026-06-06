#include "ProcessHandler.h"

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

namespace {
constexpr auto kTerminatePolls = 20;
constexpr auto kTerminatePollDelay = std::chrono::milliseconds(50);
}  // namespace

ProcessHandler::ProcessHandler() : pipe_fd_{-1, -1}, pid_(-1) {}

ProcessHandler::~ProcessHandler() {
    stop();
}

bool ProcessHandler::execute(
    const std::string& command,
    const std::vector<std::string>& args
) {
    if (isRunning()) {
        std::cerr << "process already running: " << command << std::endl;
        return false;
    }

    closePipe();
    if (pipe(pipe_fd_) == -1) {
        std::cerr << "pipe failed: " << std::strerror(errno) << std::endl;
        return false;
    }

    pid_ = fork();
    if (pid_ == 0) {
        close(pipe_fd_[0]);
        dup2(pipe_fd_[1], STDOUT_FILENO);
        dup2(pipe_fd_[1], STDERR_FILENO);
        close(pipe_fd_[1]);

        std::vector<char*> argv;
        argv.reserve(args.size() + 2);
        argv.push_back(const_cast<char*>(command.c_str()));
        for (const auto& arg : args) {
            argv.push_back(const_cast<char*>(arg.c_str()));
        }
        argv.push_back(nullptr);

        execvp(command.c_str(), argv.data());
        std::cerr << "execvp failed: " << command << ": "
                  << std::strerror(errno) << std::endl;
        _exit(EXIT_FAILURE);
    }

    if (pid_ < 0) {
        std::cerr << "fork failed: " << std::strerror(errno) << std::endl;
        closePipe();
        return false;
    }

    close(pipe_fd_[1]);
    pipe_fd_[1] = -1;
    return true;
}

std::string ProcessHandler::readLine() {
    if (pipe_fd_[0] == -1) {
        return {};
    }

    std::string line;
    char c = '\0';
    while (true) {
        const ssize_t bytes_read = read(pipe_fd_[0], &c, 1);
        if (bytes_read > 0) {
            line.push_back(c);
            if (c == '\n') {
                break;
            }
            continue;
        }

        if (bytes_read == 0) {
            break;
        }

        if (errno == EINTR) {
            continue;
        }

        break;
    }
    return line;
}

bool ProcessHandler::isRunning() {
    if (pid_ <= 0) {
        return false;
    }

    int status = 0;
    const pid_t result = waitpid(pid_, &status, WNOHANG);
    if (result == 0) {
        return true;
    }

    if (result == pid_ || errno == ECHILD) {
        pid_ = -1;
    }
    return false;
}

void ProcessHandler::stop() {
    if (pid_ <= 0) {
        closePipe();
        return;
    }

    int status = 0;
    if (waitpid(pid_, &status, WNOHANG) == 0) {
        kill(pid_, SIGTERM);
        for (int i = 0; i < kTerminatePolls; ++i) {
            if (waitpid(pid_, &status, WNOHANG) == pid_) {
                pid_ = -1;
                closePipe();
                return;
            }
            std::this_thread::sleep_for(kTerminatePollDelay);
        }

        kill(pid_, SIGKILL);
        waitpid(pid_, &status, 0);
    }

    pid_ = -1;
    closePipe();
}

void ProcessHandler::closePipe() {
    if (pipe_fd_[0] != -1) {
        close(pipe_fd_[0]);
        pipe_fd_[0] = -1;
    }
    if (pipe_fd_[1] != -1) {
        close(pipe_fd_[1]);
        pipe_fd_[1] = -1;
    }
}
