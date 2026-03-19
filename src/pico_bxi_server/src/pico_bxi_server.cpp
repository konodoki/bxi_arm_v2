#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <std_msgs/msg/detail/u_int8_multi_array__struct.hpp>
#include <std_msgs/msg/float32.hpp>
#include <thread>
#include <unistd.h>
#include "ProcessHandler.h"
#include "TcpReceiver.h"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
using namespace std;
Tcp_Pack_t g_tcp_pack;
uint64_t g_last_pack_tick;
uint64_t getCurrentTimestampMs() {
    using namespace std::chrono;
    // 获取当前时间点
    auto now = system_clock::now();
    // 转换为自 1970-01-01 以来的时长
    auto duration = now.time_since_epoch();
    // 转换为毫秒并取值
    return duration_cast<milliseconds>(duration).count();
}
void tcp_pack_hander(const Tcp_Pack_t& p){
  g_tcp_pack=p;
  g_last_pack_tick = getCurrentTimestampMs();
  // std::cout << "收到包 - 左手触发器: " << p.left_trigger 
  //           << " 右手位置 X: " << p.right_hand_pos[0] << std::endl;
}
void rtsp_server_thread(){
  //启动mediamtx rtsp服务器
  ProcessHandler ph;
  ph.execute("./bin/mediamtx", {});
  std::shared_ptr<TcpReceiver> tcp_client=nullptr;
  while (ph.isRunning()) {
      std::string output = ph.readLine();
      if (!output.empty()) {
        std::cout<<output<<std::endl;
        //解析是否有客户端连接
        std::string ip = LogParser::extractCreatedByIP(output);
        if (!ip.empty() && ip.compare("127.0.0.1")!=0) {
            if(tcp_client==nullptr){
              std::cout << "新连接 IP: " << ip << std::endl;
              tcp_client = std::make_shared<TcpReceiver>(tcp_pack_hander);
              tcp_client->connectTo(ip, 8000);
            }
        }
        ip = LogParser::extractTornDownIP(output);
        if (!ip.empty() && ip.compare("127.0.0.1")!=0) {
            if(tcp_client!=nullptr&&tcp_client->ip==ip){
              std::cout << "断开连接 IP: " << ip << std::endl;
              tcp_client.reset();
            }
        }
      }
  }
}
using namespace std::chrono_literals;
class PicoPublisher : public rclcpp::Node {
public:
  PicoPublisher() : Node("PicoPublisher"){
    // Create a publisher for the "topic" with a queue size of 10
    pico_bytes_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("pico/data", 10);
    pico_r_tri = this->create_publisher<std_msgs::msg::Float32>("pico/right_trigger",10);
    pico_l_tri = this->create_publisher<std_msgs::msg::Float32>("pico/left_trigger",10);
    pico_l_grip = this->create_publisher<std_msgs::msg::Float32>("pico/left_grip",10);
    pico_r_grip = this->create_publisher<std_msgs::msg::Float32>("pico/right_grip",10);
    // Create a timer to publish messages every 500ms
    timer_ = this->create_wall_timer(1ms, std::bind(&PicoPublisher::timer_callback, this));
  }
private:
  void timer_callback() {
      if(getCurrentTimestampMs()-g_last_pack_tick > 500){
        return;
      }
      union{
        Tcp_Pack_t pack;
        uint8_t bytes[sizeof(Tcp_Pack_t)];
      }u;
      u.pack = g_tcp_pack;
      std_msgs::msg::UInt8MultiArray byte_multi_array_msg;
      std_msgs::msg::MultiArrayDimension dim;
      dim.label = "dimension1";
      dim.size = sizeof(Tcp_Pack_t);
      dim.stride = sizeof(Tcp_Pack_t);
      byte_multi_array_msg.layout.dim.push_back(dim);
      byte_multi_array_msg.layout.data_offset = 0;
      
      byte_multi_array_msg.data.resize(sizeof(Tcp_Pack_t));
      std::memcpy(byte_multi_array_msg.data.data(), u.bytes, sizeof(Tcp_Pack_t));
      pico_bytes_publisher_->publish(byte_multi_array_msg);


      std_msgs::msg::Float32 f;
      f.set__data(u.pack.right_trigger);
      pico_r_tri->publish(f);
      f.set__data(u.pack.left_trigger);
      pico_l_tri->publish(f);
      f.set__data(u.pack.left_grip);
      pico_l_grip->publish(f);
      f.set__data(u.pack.right_grip);
      pico_r_grip->publish(f);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pico_bytes_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pico_r_tri;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pico_l_tri;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pico_l_grip;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pico_r_grip;
};
int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  std::thread rtsp_server(rtsp_server_thread);
  rtsp_server.detach();
  usleep(1000000);
  //启动ffmpeg推流
  ProcessHandler ffmpeg;
  //经过测试 0是深度 2是红外 4是rgb
  ffmpeg.execute("ffmpeg", {"-loglevel","quiet","-framerate","60","-video_size","424x240","-i","/dev/video4","-vcodec","libx264","-preset","ultrafast","-tune","zerolatency","-rtsp_transport","udp","-f","rtsp","rtsp://127.0.0.1:2212/video"});
  // ffmpeg.execute("ffmpeg", {"-loglevel","quiet","-framerate","60","-video_size","1920x1680","-i","/dev/video4","-vcodec","libx264","-preset","ultrafast","-tune","zerolatency","-rtsp_transport","udp","-f","rtsp","rtsp://127.0.0.1:2212/video"});
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PicoPublisher>());
  rclcpp::shutdown();
  return 0;
}
