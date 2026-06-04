// AI Grand Prix IMU driver: republishes the simulator's MAVLink HIGHRES_IMU
// stream as sensor_msgs/Imu on /imu0 for OpenVINS VIO.
//
// The AI-GP sim is not a PX4/ArduPilot autopilot (HEARTBEAT reports
// autopilot=GENERIC from component 200), so MAVSDK never attaches. We parse the
// UDP MAVLink stream directly with the standard mavlink C library and decode
// HIGHRES_IMU (id 105) — body-FRD accel (m/s^2) and gyro (rad/s), published as-is
// (the body->IMU transform is identity per spec; OpenVINS owns the frame).

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <atomic>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <mavlink/v2.0/common/mavlink.h>

namespace aigp_imu_driver
{

class ImuBridgeNode : public rclcpp::Node
{
public:
  explicit ImuBridgeNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("aigp_imu_bridge_node", options)
  {
    const auto url = declare_parameter<std::string>("connection_url", "udpin://0.0.0.0:14550");
    const auto topic = declare_parameter<std::string>("imu_topic", "/imu0");
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    use_sim_timestamp_ = declare_parameter<bool>("use_sim_timestamp", true);

    pub_ = create_publisher<sensor_msgs::msg::Imu>(topic, rclcpp::SensorDataQoS());

    port_ = parse_port(url);
    if (!open_socket(port_)) {
      throw std::runtime_error("aigp_imu_driver: failed to bind UDP port " + std::to_string(port_));
    }
    running_ = true;
    rx_thread_ = std::thread(&ImuBridgeNode::receive_loop, this);

    RCLCPP_INFO(
      get_logger(), "AIGP IMU driver listening on udpin://0.0.0.0:%u, publishing %s on HIGHRES_IMU",
      port_, topic.c_str());
  }

  ~ImuBridgeNode() override
  {
    running_ = false;
    if (fd_ >= 0) {
      ::shutdown(fd_, SHUT_RDWR);
      ::close(fd_);
    }
    if (rx_thread_.joinable()) {
      rx_thread_.join();
    }
  }

private:
  static uint16_t parse_port(const std::string & url)
  {
    const auto pos = url.rfind(':');
    return pos == std::string::npos ? 14550 : static_cast<uint16_t>(std::stoi(url.substr(pos + 1)));
  }

  bool open_socket(uint16_t port)
  {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
      return false;
    }
    const int one = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    // Unblock receive_loop periodically so it can observe running_ == false.
    timeval tv{1, 0};
    ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);
    if (::bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
      ::close(fd_);
      fd_ = -1;
      return false;
    }
    return true;
  }

  void receive_loop()
  {
    uint8_t buf[2048];
    mavlink_message_t msg;
    mavlink_status_t status;
    while (running_) {
      const ssize_t n = ::recv(fd_, buf, sizeof(buf), 0);
      if (n <= 0) {
        continue;  // timeout or transient error
      }
      for (ssize_t i = 0; i < n; ++i) {
        if (mavlink_frame_char_buffer(&rx_msg_, &rx_status_, buf[i], &msg, &status) ==
          MAVLINK_FRAMING_OK && msg.msgid == MAVLINK_MSG_ID_HIGHRES_IMU)
        {
          mavlink_highres_imu_t h;
          mavlink_msg_highres_imu_decode(&msg, &h);
          publish(h);
        }
      }
    }
  }

  void publish(const mavlink_highres_imu_t & h)
  {
    auto m = std::make_unique<sensor_msgs::msg::Imu>();
    m->header.frame_id = frame_id_;
    m->header.stamp = use_sim_timestamp_
      ? rclcpp::Time(static_cast<int64_t>(h.time_usec) * 1000)
      : now();

    m->linear_acceleration.x = h.xacc;
    m->linear_acceleration.y = h.yacc;
    m->linear_acceleration.z = h.zacc;
    m->angular_velocity.x = h.xgyro;
    m->angular_velocity.y = h.ygyro;
    m->angular_velocity.z = h.zgyro;
    // Raw IMU has no orientation estimate (REP-145: covariance[0] = -1).
    m->orientation_covariance[0] = -1.0;

    pub_->publish(std::move(m));
  }

  std::string frame_id_;
  bool use_sim_timestamp_{true};
  uint16_t port_{14550};
  int fd_{-1};
  std::atomic<bool> running_{false};
  std::thread rx_thread_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  mavlink_message_t rx_msg_{};   // per-instance MAVLink parse buffers
  mavlink_status_t rx_status_{};
};

}  // namespace aigp_imu_driver

RCLCPP_COMPONENTS_REGISTER_NODE(aigp_imu_driver::ImuBridgeNode)
