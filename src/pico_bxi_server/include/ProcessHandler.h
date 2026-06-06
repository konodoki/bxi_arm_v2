#ifndef PROCESS_HANDLER_H
#define PROCESS_HANDLER_H

#include <sys/types.h>

#include <string>
#include <vector>

class ProcessHandler {
public:
    ProcessHandler();
    ~ProcessHandler();

    ProcessHandler(const ProcessHandler&) = delete;
    ProcessHandler& operator=(const ProcessHandler&) = delete;

    bool execute(
        const std::string& command,
        const std::vector<std::string>& args
    );
    std::string readLine();
    bool isRunning();
    void stop();

private:
    void closePipe();

    int pipe_fd_[2];
    pid_t pid_;
};

#endif
