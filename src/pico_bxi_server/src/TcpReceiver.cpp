#include "TcpReceiver.h"

#include <CRC.h>
#include <algorithm>
#include <arpa/inet.h>
#include <array>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <regex>
#include <sys/socket.h>
#include <unistd.h>
#include <utility>

namespace {
constexpr std::array<uint8_t, 4> kPacketHeader = {
    0xA1,
    0xA2,
    0xA3,
    0xA4,
};
constexpr size_t kReadBufferSize = 512;
constexpr size_t kPacketSize = sizeof(Tcp_Pack_t);
}  // namespace

std::string LogParser::extractCreatedByIP(const std::string& log_line) {
    return extractIPAfterPrefix(log_line, "created by");
}

std::string LogParser::extractTornDownIP(const std::string& log_line) {
    return extractIPAfterPrefix(log_line, "torn down by");
}

std::string LogParser::extractIPAfterPrefix(
    const std::string& log_line,
    const std::string& prefix
) {
    const std::regex pattern(prefix + R"(\s+((\d{1,3}\.){3}\d{1,3}))");
    std::smatch match;
    if (std::regex_search(log_line, match, pattern)) {
        return match[1].str();
    }
    return {};
}

TcpReceiver::TcpReceiver(TcpPackCb cb)
    : sock_(-1),
      stop_requested_(false),
      cb_(std::move(cb)) {
    buffer_.reserve(kPacketSize * 2);
}

TcpReceiver::~TcpReceiver() {
    stop();
}

bool TcpReceiver::connectTo(const std::string& remote_ip, int port) {
    stop();
    stop_requested_.store(false);
    remote_ip_ = remote_ip;

    if (port <= 0 || port > 65535) {
        std::cerr << "invalid TCP port: " << port << std::endl;
        return false;
    }

    const int socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd == -1) {
        std::cerr << "socket failed: " << std::strerror(errno) << std::endl;
        return false;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, remote_ip.c_str(), &addr.sin_addr) != 1) {
        std::cerr << "invalid IPv4 address: " << remote_ip << std::endl;
        close(socket_fd);
        return false;
    }

    if (connect(socket_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr))
        != 0) {
        std::cerr << "connect failed to " << remote_ip << ":" << port
                  << ": " << std::strerror(errno) << std::endl;
        close(socket_fd);
        return false;
    }

    sock_.store(socket_fd);
    receiver_thread_ = std::thread(&TcpReceiver::readLoop, this);
    return true;
}

void TcpReceiver::stop() {
    stop_requested_.store(true);

    const int socket_fd = sock_.exchange(-1);
    if (socket_fd != -1) {
        shutdown(socket_fd, SHUT_RDWR);
        close(socket_fd);
    }

    if (receiver_thread_.joinable()
        && receiver_thread_.get_id() != std::this_thread::get_id()) {
        receiver_thread_.join();
    }
}

const std::string& TcpReceiver::remoteIp() const {
    return remote_ip_;
}

void TcpReceiver::readLoop() {
    std::array<uint8_t, kReadBufferSize> temp_buf{};

    while (!stop_requested_.load()) {
        const int socket_fd = sock_.load();
        if (socket_fd == -1) {
            break;
        }

        const ssize_t len = recv(
            socket_fd,
            temp_buf.data(),
            temp_buf.size(),
            0
        );
        if (len > 0) {
            buffer_.insert(
                buffer_.end(),
                temp_buf.begin(),
                temp_buf.begin() + len
            );
            processBuffer();
            continue;
        }

        if (len == 0) {
            std::cerr << "tcp connection closed: " << remote_ip_ << std::endl;
        } else if (!stop_requested_.load()) {
            std::cerr << "recv failed from " << remote_ip_ << ": "
                      << std::strerror(errno) << std::endl;
        }
        break;
    }

    const int socket_fd = sock_.exchange(-1);
    if (socket_fd != -1) {
        close(socket_fd);
    }
    stop_requested_.store(true);
}

void TcpReceiver::processBuffer() {
    while (buffer_.size() >= kPacketSize && !stop_requested_.load()) {
        if (!std::equal(
                kPacketHeader.begin(),
                kPacketHeader.end(),
                buffer_.begin()
            )) {
            buffer_.erase(buffer_.begin());
            continue;
        }

        Tcp_Pack_t pack{};
        std::memcpy(&pack, buffer_.data(), kPacketSize);

        const uint32_t crc = CRC::Calculate(
            buffer_.data(),
            kPacketSize - sizeof(pack.crc),
            CRC::CRC_32()
        );
        if (pack.crc == crc && cb_) {
            cb_(pack);
        } else {
            std::cerr << "invalid tcp packet crc" << std::endl;
        }

        buffer_.erase(buffer_.begin(), buffer_.begin() + kPacketSize);
    }
}
