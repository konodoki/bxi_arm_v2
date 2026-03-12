#include "TcpReceiver.h"
#include <cstdint>
#include <cstdio>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <thread>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <regex>
#include <CRC.h>
std::string LogParser::extractCreatedByIP(const std::string& logLine) {
    return extractIPAfterPrefix(logLine, "created by");
}

std::string LogParser::extractTornDownIP(const std::string& logLine) {
    return extractIPAfterPrefix(logLine, "torn down by");
}

std::string LogParser::extractIPAfterPrefix(const std::string& logLine, const std::string& prefix) {
    // 动态构建正则表达式：前缀 + 空格 + IP捕获组
    // \Q...\E 确保前缀中的特殊字符被当作普通字符处理
    std::regex pattern(prefix + R"(\s+((\d{1,3}\.){3}\d{1,3}))");
    std::smatch match;

    if (std::regex_search(logLine, match, pattern)) {
        return match[1].str(); // 返回第一个捕获组，即 IP 部分
    }
    return "";
}
TcpReceiver::TcpReceiver(TcpPackCb cb) : sock(-1) ,cb(cb){
    buffer.reserve(1024); // 预留空间
}

TcpReceiver::~TcpReceiver() {
    isStop=true;
    if (sock != -1) close(sock);
    std::cout<<"客户端销毁"<<ip<<std::endl;
}

bool TcpReceiver::connectTo(const std::string& ip, int port) {
    this->ip = ip;
    sock = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    inet_pton(AF_INET, ip.c_str(), &addr.sin_addr);
    if(connect(sock, (struct sockaddr*)&addr, sizeof(addr)) == 0){
        std::thread rec_thread([this](){
            uint8_t temp_buf[512];
            while (!isStop) {
                ssize_t len = recv(sock, temp_buf, sizeof(temp_buf), 0);
                if (len <= 0) {
                    std::cerr << "连接断开或错误" << std::endl;
                    isStop=true;
                }

                // 1. 将新数据存入缓冲区
                buffer.insert(buffer.end(), temp_buf, temp_buf + len);

                // 2. 解析缓冲区
                processBuffer();
            }
        });
        rec_thread.detach();
    }
    return true;
}

void TcpReceiver::processBuffer() {
    const size_t PACK_SIZE = sizeof(Tcp_Pack_t);

    // 只要缓冲区长度够一个包，就尝试解析
    while (buffer.size() >= PACK_SIZE && !isStop) {
        // 检查包头是否匹配 0xA1 0xA2 0xA3 0xA4
        if ((uint8_t)buffer[0] == 0xA1 && (uint8_t)buffer[1] == 0xA2 && 
            (uint8_t)buffer[2] == 0xA3 && (uint8_t)buffer[3] == 0xA4) {
            
            // 提取数据包
            union {
                Tcp_Pack_t pack;
                uint8_t bytes[PACK_SIZE];
            } u;

            std::copy(buffer.begin(), buffer.begin() + PACK_SIZE, u.bytes);
            uint32_t crc = CRC::Calculate(u.bytes,PACK_SIZE-4,CRC::CRC_32());
            if(u.pack.crc==crc){
                cb(u.pack);
            }else{
                printf("数据有误");
            }
            // 从缓冲区移除已处理的数据
            buffer.erase(buffer.begin(), buffer.begin() + PACK_SIZE);
        } else {
            // 如果头不匹配，说明流数据错位，丢弃第一个字节继续寻找下一个可能的头
            buffer.erase(buffer.begin());
        }
    }
}