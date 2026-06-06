#ifndef TCP_RECEIVER_H
#define TCP_RECEIVER_H

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>
#include <vector>

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
    static std::string extractCreatedByIP(const std::string& log_line);
    static std::string extractTornDownIP(const std::string& log_line);

private:
    static std::string extractIPAfterPrefix(
        const std::string& log_line,
        const std::string& prefix
    );
};

class TcpReceiver {
public:
    explicit TcpReceiver(TcpPackCb cb);
    ~TcpReceiver();

    TcpReceiver(const TcpReceiver&) = delete;
    TcpReceiver& operator=(const TcpReceiver&) = delete;

    bool connectTo(const std::string& remote_ip, int port);
    void stop();
    const std::string& remoteIp() const;

private:
    void readLoop();
    void processBuffer();

    std::atomic<int> sock_;
    std::atomic_bool stop_requested_;
    std::thread receiver_thread_;
    std::vector<uint8_t> buffer_;
    TcpPackCb cb_;
    std::string remote_ip_;
};

#endif
