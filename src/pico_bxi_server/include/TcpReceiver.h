#ifndef TCP_RECEIVER_H
#define TCP_RECEIVER_H

#include <functional>
#include <vector>
#include <string>
#include <cstdint>

typedef struct Tcp_Pack {
    char header[4];
    float head_pos[3];
    float head_ori[4];
    float left_hand_pos[3];
    float left_hand_ori[4];
    float left_trigger;
    float left_grip;
    float right_hand_pos[3];
    float right_hand_ori[4];
    float right_trigger;
    float right_grip;
    float left_hand_joy[2];
    float right_hand_joy[2];
    uint32_t crc;
} Tcp_Pack_t;

typedef std::function<void(const Tcp_Pack_t& p)> TcpPackCb;
class LogParser {
public:
    // 提取 "created by " 之后的 IP
    static std::string extractCreatedByIP(const std::string& logLine);

    // 提取 "torn down by " 之后的 IP (新增加)
    static std::string extractTornDownIP(const std::string& logLine);

private:
    // 内部通用的提取逻辑
    static std::string extractIPAfterPrefix(const std::string& logLine, const std::string& prefix);
};
class TcpReceiver {
public:
    std::string ip;
    TcpReceiver(TcpPackCb cb);
    ~TcpReceiver();

    bool connectTo(const std::string& ip, int port);
    // 核心逻辑：不断运行，处理数据并回调
    void startListening();

private:
    int sock;
    bool isStop=false;
    std::vector<uint8_t> buffer; // 接收缓冲区
    TcpPackCb cb;
    // 解析缓冲区中的数据
    void processBuffer();
};

#endif