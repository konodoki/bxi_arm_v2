#ifndef PROCESS_HANDLER_H
#define PROCESS_HANDLER_H

#include <string>
#include <vector>

class ProcessHandler {
public:
    ProcessHandler();
    ~ProcessHandler();

    // 启动程序并重定向输出
    bool execute(const std::string& command, const std::vector<std::string>& args);
    
    // 从管道读取一行输出
    std::string readLine();

    // 检查子进程是否还在运行
    bool isRunning() const;

private:
    int pipe_fd[2]; // 0: 读取端, 1: 写入端
    pid_t pid;
};

#endif