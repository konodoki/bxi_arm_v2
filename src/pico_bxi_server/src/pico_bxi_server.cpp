#include "ProcessHandler.h"
#include "TcpReceiver.h"

#include <atomic>
#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

namespace {
constexpr auto kDefaultStaleTimeout = std::chrono::milliseconds(500);

bool isLoopbackClient(const std::string& ip) {
    return ip == "127.0.0.1";
}
}  // namespace

class PacketStore {
public:
    void update(const Tcp_Pack_t& packet) {
        const std::lock_guard<std::mutex> lock(mutex_);
        packet_ = packet;
        last_update_ = Clock::now();
        has_packet_ = true;
    }

    std::optional<Tcp_Pack_t> latest(
        std::chrono::milliseconds max_age
    ) const {
        const std::lock_guard<std::mutex> lock(mutex_);
        if (!has_packet_) {
            return std::nullopt;
        }

        if (Clock::now() - last_update_ > max_age) {
            return std::nullopt;
        }
        return packet_;
    }

private:
    using Clock = std::chrono::steady_clock;

    mutable std::mutex mutex_;
    Tcp_Pack_t packet_{};
    Clock::time_point last_update_{};
    bool has_packet_{false};
};

class PicoPublisher : public rclcpp::Node {
public:
    PicoPublisher() : Node("pico_bxi_server") {
        declareParameters();
        loadParameters();
        createPublishers();

        timer_ = create_wall_timer(
            std::chrono::milliseconds(publish_period_ms_),
            std::bind(&PicoPublisher::timerCallback, this)
        );

        if (start_video_pipeline_) {
            startVideoPipeline();
        }
    }

    ~PicoPublisher() override {
        stop_requested_.store(true);

        {
            const std::lock_guard<std::mutex> lock(tcp_client_mutex_);
            tcp_client_.reset();
        }

        mediamtx_.stop();
        if (rtsp_thread_.joinable()) {
            rtsp_thread_.join();
        }
        ffmpeg_.stop();
    }

private:
    void declareParameters() {
        declare_parameter("start_video_pipeline", true);
        declare_parameter("mediamtx_command", "./bin/mediamtx");
        declare_parameter("ffmpeg_command", "ffmpeg");
        declare_parameter("tcp_port", 8000);
        declare_parameter("publish_period_ms", 1);
        declare_parameter(
            "stale_timeout_ms",
            static_cast<int>(kDefaultStaleTimeout.count())
        );
        declare_parameter("video_device", "/dev/video4");
        declare_parameter("video_framerate", 60);
        declare_parameter("video_size", "424x240");
        declare_parameter("rtsp_url", "rtsp://127.0.0.1:2212/video");
        declare_parameter("startup_delay_ms", 1000);
    }

    void loadParameters() {
        start_video_pipeline_ =
            get_parameter("start_video_pipeline").as_bool();
        mediamtx_command_ = get_parameter("mediamtx_command").as_string();
        ffmpeg_command_ = get_parameter("ffmpeg_command").as_string();
        tcp_port_ = positiveIntParameter("tcp_port");
        if (tcp_port_ > 65535) {
            throw std::runtime_error("tcp_port must be <= 65535");
        }

        publish_period_ms_ = positiveIntParameter("publish_period_ms");
        stale_timeout_ = std::chrono::milliseconds(
            positiveIntParameter("stale_timeout_ms")
        );
        video_device_ = get_parameter("video_device").as_string();
        video_framerate_ = positiveIntParameter("video_framerate");
        video_size_ = get_parameter("video_size").as_string();
        rtsp_url_ = get_parameter("rtsp_url").as_string();
        startup_delay_ms_ = nonNegativeIntParameter("startup_delay_ms");
    }

    int positiveIntParameter(const std::string& name) {
        const auto value = get_parameter(name).as_int();
        if (value <= 0) {
            throw std::runtime_error(name + " must be positive");
        }
        return static_cast<int>(value);
    }

    int nonNegativeIntParameter(const std::string& name) {
        const auto value = get_parameter(name).as_int();
        if (value < 0) {
            throw std::runtime_error(name + " must be non-negative");
        }
        return static_cast<int>(value);
    }

    void createPublishers() {
        pico_bytes_publisher_ =
            create_publisher<std_msgs::msg::UInt8MultiArray>(
                "pico/data",
                10
            );
        right_trigger_publisher_ =
            create_publisher<std_msgs::msg::Float32>(
                "pico/right_trigger",
                10
            );
        left_trigger_publisher_ =
            create_publisher<std_msgs::msg::Float32>(
                "pico/left_trigger",
                10
            );
        left_grip_publisher_ =
            create_publisher<std_msgs::msg::Float32>(
                "pico/left_grip",
                10
            );
        right_grip_publisher_ =
            create_publisher<std_msgs::msg::Float32>(
                "pico/right_grip",
                10
            );
    }

    void startVideoPipeline() {
        rtsp_thread_ = std::thread(&PicoPublisher::rtspServerLoop, this);
        std::this_thread::sleep_for(
            std::chrono::milliseconds(startup_delay_ms_)
        );
        startFfmpeg();
    }

    void startFfmpeg() {
        const std::vector<std::string> args = {
            "-loglevel",
            "quiet",
            "-framerate",
            std::to_string(video_framerate_),
            "-video_size",
            video_size_,
            "-i",
            video_device_,
            "-vcodec",
            "libx264",
            "-preset",
            "ultrafast",
            "-tune",
            "zerolatency",
            "-rtsp_transport",
            "udp",
            "-f",
            "rtsp",
            rtsp_url_,
        };

        if (!ffmpeg_.execute(ffmpeg_command_, args)) {
            RCLCPP_ERROR(
                get_logger(),
                "failed to start ffmpeg command: %s",
                ffmpeg_command_.c_str()
            );
        }
    }

    void rtspServerLoop() {
        if (!mediamtx_.execute(mediamtx_command_, {})) {
            RCLCPP_ERROR(
                get_logger(),
                "failed to start mediamtx command: %s",
                mediamtx_command_.c_str()
            );
            return;
        }

        while (!stop_requested_.load() && mediamtx_.isRunning()) {
            const std::string output = mediamtx_.readLine();
            if (output.empty()) {
                continue;
            }

            RCLCPP_INFO(get_logger(), "%s", output.c_str());
            handleMediaMtxLog(output);
        }
    }

    void handleMediaMtxLog(const std::string& output) {
        const std::string created_ip =
            LogParser::extractCreatedByIP(output);
        if (!created_ip.empty() && !isLoopbackClient(created_ip)) {
            connectTcpClient(created_ip);
        }

        const std::string torn_down_ip =
            LogParser::extractTornDownIP(output);
        if (!torn_down_ip.empty() && !isLoopbackClient(torn_down_ip)) {
            disconnectTcpClient(torn_down_ip);
        }
    }

    void connectTcpClient(const std::string& ip) {
        const std::lock_guard<std::mutex> lock(tcp_client_mutex_);
        if (tcp_client_) {
            return;
        }

        auto receiver = std::make_shared<TcpReceiver>(
            [this](const Tcp_Pack_t& packet) {
                packet_store_.update(packet);
            }
        );

        if (!receiver->connectTo(ip, tcp_port_)) {
            RCLCPP_ERROR(
                get_logger(),
                "failed to connect Pico TCP client at %s:%d",
                ip.c_str(),
                tcp_port_
            );
            return;
        }

        tcp_client_ = std::move(receiver);
        RCLCPP_INFO(
            get_logger(),
            "connected Pico TCP client: %s:%d",
            ip.c_str(),
            tcp_port_
        );
    }

    void disconnectTcpClient(const std::string& ip) {
        const std::lock_guard<std::mutex> lock(tcp_client_mutex_);
        if (!tcp_client_ || tcp_client_->remoteIp() != ip) {
            return;
        }

        RCLCPP_INFO(get_logger(), "disconnected Pico TCP client: %s", ip.c_str());
        tcp_client_.reset();
    }

    void timerCallback() {
        const auto packet = packet_store_.latest(stale_timeout_);
        if (!packet) {
            return;
        }

        publishRawPacket(*packet);
        publishFloat(right_trigger_publisher_, packet->right_trigger);
        publishFloat(left_trigger_publisher_, packet->left_trigger);
        publishFloat(left_grip_publisher_, packet->left_grip);
        publishFloat(right_grip_publisher_, packet->right_grip);
    }

    void publishRawPacket(const Tcp_Pack_t& packet) {
        std_msgs::msg::UInt8MultiArray message;
        std_msgs::msg::MultiArrayDimension dimension;
        dimension.label = "dimension1";
        dimension.size = sizeof(Tcp_Pack_t);
        dimension.stride = sizeof(Tcp_Pack_t);
        message.layout.dim.push_back(dimension);
        message.layout.data_offset = 0;

        message.data.resize(sizeof(Tcp_Pack_t));
        std::memcpy(message.data.data(), &packet, sizeof(Tcp_Pack_t));
        pico_bytes_publisher_->publish(message);
    }

    void publishFloat(
        const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr& publisher,
        float value
    ) {
        std_msgs::msg::Float32 message;
        message.data = value;
        publisher->publish(message);
    }

    PacketStore packet_store_;
    ProcessHandler mediamtx_;
    ProcessHandler ffmpeg_;
    std::thread rtsp_thread_;
    std::atomic_bool stop_requested_{false};

    std::mutex tcp_client_mutex_;
    std::shared_ptr<TcpReceiver> tcp_client_;

    bool start_video_pipeline_{true};
    std::string mediamtx_command_;
    std::string ffmpeg_command_;
    int tcp_port_{8000};
    int publish_period_ms_{1};
    std::chrono::milliseconds stale_timeout_{kDefaultStaleTimeout};
    std::string video_device_;
    int video_framerate_{60};
    std::string video_size_;
    std::string rtsp_url_;
    int startup_delay_ms_{1000};

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr
        pico_bytes_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
        right_trigger_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
        left_trigger_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
        left_grip_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
        right_grip_publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PicoPublisher>();
    rclcpp::spin(node);
    node.reset();
    rclcpp::shutdown();
    return 0;
}
