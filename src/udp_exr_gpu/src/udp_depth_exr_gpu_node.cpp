#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>  // for fcntl()
#include <cstdio>   // for remove()
#include <fstream>
#include <sstream>
#include <chrono>

// JSON 用於 Parse (nlohmann-json)
#include <nlohmann/json.hpp>  // sudo apt-get install nlohmann-json3-dev

// Base64 decode (需要自備或使用第三方函式庫)
#include "base64_utils.hpp"

// OpenEXR
#include <ImfInputFile.h>
#include <ImfChannelList.h>
#include <ImfNamespace.h>
#include <ImfHeader.h>
#include <ImathBox.h>

// half type
#include <half.h>  // 可能在 <ImfHalf.h> 或 <half/half.hpp> 依據系統安裝

using namespace std::chrono_literals;

class UdpDepthExrCpuNode : public rclcpp::Node
{
public:
    UdpDepthExrCpuNode(const std::string& ip, int port)
    : Node("udp_depth_exr_cpu_node"), udp_ip_(ip), udp_port_(port)
    {
        // 建立 Publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("depth_image_topic", 10);

        // 初始化 Socket
        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket!");
            throw std::runtime_error("Failed to create socket");
        }

        sockaddr_in servaddr;
        servaddr.sin_family = AF_INET;
        servaddr.sin_addr.s_addr = inet_addr(udp_ip_.c_str());
        servaddr.sin_port = htons(udp_port_);

        // 若 ip == "0.0.0.0" → INADDR_ANY
        if (udp_ip_ == "0.0.0.0") {
            servaddr.sin_addr.s_addr = INADDR_ANY;
        }

        if (bind(sockfd_, (struct sockaddr*)&servaddr, sizeof(servaddr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind socket!");
            throw std::runtime_error("Failed to bind socket");
        }

        RCLCPP_INFO(this->get_logger(), "UDP server listening on %s:%d", ip.c_str(), port);

        // 設定一個 timer 週期性檢查是否有封包
        timer_ = this->create_wall_timer(10ms, 
                    std::bind(&UdpDepthExrCpuNode::udp_polling, this));
    }

    ~UdpDepthExrCpuNode() override
    {
        if (sockfd_ >= 0) {
            close(sockfd_);
        }
    }

private:
    void udp_polling()
    {
        // non-blocking recvfrom
        sockaddr_in clientaddr;
        socklen_t len = sizeof(clientaddr);

        int flags = fcntl(sockfd_, F_GETFL, 0);
        fcntl(sockfd_, F_SETFL, flags | O_NONBLOCK);

        char buffer[65535];
        int n = recvfrom(sockfd_, buffer, sizeof(buffer), 0, 
                         (struct sockaddr*)&clientaddr, &len);
        if (n <= 0) {
            // no data or error
            return;
        }
        // 處理封包
        std::string data_str(buffer, n);
        handle_udp_data(data_str);
    }

    void handle_udp_data(const std::string& data_str)
    {
        try {
            // 1) 解析 JSON
            auto json_obj = nlohmann::json::parse(data_str);
            std::string timestamp_str = json_obj["timestamp"];
            std::string base64_data   = json_obj["data"];

            // 2) base64 decode
            std::vector<unsigned char> exr_bytes = base64_decode(base64_data);

            // 3) 寫到暫存檔
            char tmpname[] = "/tmp/exr_cpu_XXXXXX.exr";
            int fd = mkstemps(tmpname, 4); // 後綴名長度 .exr = 4
            if (fd < 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create temp file!");
                return;
            }
            write(fd, exr_bytes.data(), exr_bytes.size());
            close(fd);

            // 4) OpenEXR 讀取 R channel (half) → float
            Imf::InputFile file(tmpname);
            Imf::Header header = file.header();
            Imath::Box2i dw = header.dataWindow();

            int width  = dw.max.x - dw.min.x + 1;
            int height = dw.max.y - dw.min.y + 1;

            // 確認 R channel 存在
            const Imf::ChannelList & channels = header.channels();
            if (!channels.findChannel("R")) {
                RCLCPP_ERROR(this->get_logger(), "EXR does not contain R channel!");
                remove(tmpname);
                return;
            }

            // 讀 R channel: half
            std::vector<half> half_pixels(width * height);
            Imf::FrameBuffer fb;
            fb.insert("R", Imf::Slice(Imf::HALF,
                     (char*)half_pixels.data(),
                     sizeof(half),
                     sizeof(half) * width));
            file.setFrameBuffer(fb);
            file.readPixels(dw.min.y, dw.max.y);

            // half -> float
            std::vector<float> float_pixels(width * height);
            for (size_t i = 0; i < half_pixels.size(); i++) {
                float_pixels[i] = half_pixels[i];
            }

            // 這裡「不使用 GPU」，直接在 CPU 上 clamp & scale
            // 假設: clamp [0..1]，然後乘以 65535
            for (auto & val : float_pixels) {
                if (val < 0.0f) val = 0.0f;
                if (val > 1.0f) val = 1.0f;
                val *= 65535.0f;
            }

            // 5) 組成 uint16
            std::vector<uint16_t> depth_in_uint16(float_pixels.size());
            for (size_t i = 0; i < float_pixels.size(); i++)
            {
                float v = float_pixels[i];
                if (v < 0.0f) v = 0.0f;
                if (v > 65535.0f) v = 65535.0f;
                depth_in_uint16[i] = static_cast<uint16_t>(v);
            }

            // 6) 封裝成 ROS2 Image (mono16)
            auto msg = sensor_msgs::msg::Image();
            msg.header.stamp.sec = std::stoll(timestamp_str) / 1000; 
            msg.header.stamp.nanosec = (std::stoll(timestamp_str) % 1000) * 1000000;
            msg.header.frame_id = "camera_depth_frame";
            msg.width  = width;
            msg.height = height;
            msg.encoding = "mono16";
            msg.is_bigendian = 0;
            msg.step = width * 2; // 2 bytes per pixel
            msg.data.resize(depth_in_uint16.size() * sizeof(uint16_t));
            memcpy(msg.data.data(), depth_in_uint16.data(), msg.data.size());

            publisher_->publish(msg);

            // 刪除暫存檔
            remove(tmpname);

        } catch (std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "Error parsing EXR data: %s", e.what());
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

    std::string udp_ip_;
    int udp_port_;
    int sockfd_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UdpDepthExrCpuNode>("0.0.0.0", 10000);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
