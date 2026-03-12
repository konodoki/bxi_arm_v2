#include "ProcessHandler.h"
#include <unistd.h>
#include <sys/wait.h>
#include <iostream>
#include <array>

ProcessHandler::ProcessHandler() : pid(-1) {
    pipe_fd[0] = -1;
    pipe_fd[1] = -1;
}

ProcessHandler::~ProcessHandler() {
    if (pipe_fd[0] != -1) close(pipe_fd[0]);
    if (pid > 0) waitpid(pid, nullptr, WNOHANG);
}

bool ProcessHandler::execute(const std::string& command, const std::vector<std::string>& args) {
    // 1. 创建管道
    if (pipe(pipe_fd) == -1) {
        perror("pipe");
        return false;
    }

    // 2. 创建子进程
    pid = fork();

    if (pid == 0) { // 子进程
        // 关闭不需要的读取端
        close(pipe_fd[0]);

        // 将标准输出 (STDOUT) 重定向到管道的写入端
        dup2(pipe_fd[1], STDOUT_FILENO);
        // 如果也想捕获标准错误，取消下面这行的注释
        // dup2(pipe_fd[1], STDERR_FILENO);

        close(pipe_fd[1]);

        // 准备参数执行
        std::vector<char*> c_args;
        c_args.push_back(const_cast<char*>(command.c_str()));
        for (const auto& arg : args) {
            c_args.push_back(const_cast<char*>(arg.c_str()));
        }
        c_args.push_back(nullptr);

        execvp(command.c_str(), c_args.data());
        
        // 如果 execvp 执行失败
        perror("execvp");
        exit(1);
    } else if (pid > 0) { // 父进程
        // 关闭不需要的写入端
        close(pipe_fd[1]);
        return true;
    } else {
        perror("fork");
        return false;
    }
}

std::string ProcessHandler::readLine() {
    char buffer[1024];
    std::string result = "";
    
    // 这里使用简单的 read，生产环境建议使用更健壮的缓冲读取
    ssize_t bytesRead = read(pipe_fd[0], buffer, sizeof(buffer) - 1);
    if (bytesRead > 0) {
        buffer[bytesRead] = '\0';
        result = buffer;
    }
    return result;
}

bool ProcessHandler::isRunning() const {
    if (pid <= 0) return false;
    int status;
    return waitpid(pid, &status, WNOHANG) == 0;
}