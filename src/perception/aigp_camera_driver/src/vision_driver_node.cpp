// AI Grand Prix vision driver: reassembles the simulator's chunked-JPEG UDP
// stream (spec VADR-TS-002 sec. 4.6) and republishes the standard /cam0 camera
// contract: image_raw (mono8, VIO), image_rgb (rgb8, gate detection),
// camera_info (pinhole). Decoding and per-stream publishing are lazy — skipped
// entirely when no one is subscribed — and images are built in place and moved
// out as unique_ptr for intra-process zero-copy.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <atomic>
#include <cstring>
#include <map>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace aigp_camera_driver
{

// Wire header, little-endian (spec sec. 4.6). Targets are x86/ARM LE.
struct __attribute__((packed)) PacketHeader
{
  uint32_t frame_id;
  uint16_t chunk_id;
  uint16_t total_chunks;
  uint32_t jpeg_size;
  uint32_t payload_size;
  uint64_t sim_time_ns;
};
static_assert(sizeof(PacketHeader) == 24, "unexpected header padding");

class VisionDriverNode : public rclcpp::Node
{
public:
  explicit VisionDriverNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("aigp_vision_driver_node", options)
  {
    port_ = static_cast<uint16_t>(declare_parameter<int>("udp_port", 5600));
    bind_address_ = declare_parameter<std::string>("udp_bind_address", "0.0.0.0");
    frame_id_ = declare_parameter<std::string>("frame_id", "cam0");
    use_sim_timestamp_ = declare_parameter<bool>("use_sim_timestamp", true);
    declare_parameter<int>("width", 640);
    declare_parameter<int>("height", 360);
    build_camera_info();

    const auto qos = rclcpp::SensorDataQoS();
    image_raw_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", qos);
    image_rgb_pub_ = create_publisher<sensor_msgs::msg::Image>("image_rgb", qos);
    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", qos);

    if (!open_socket()) {
      throw std::runtime_error("aigp_camera_driver: failed to bind UDP port " + std::to_string(port_));
    }
    running_ = true;
    rx_thread_ = std::thread(&VisionDriverNode::receive_loop, this);

    RCLCPP_INFO(
      get_logger(), "AIGP vision driver listening on udp://%s:%u, publishing %s",
      bind_address_.c_str(), port_, frame_id_.c_str());
  }

  ~VisionDriverNode() override
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
  struct FrameBuffer
  {
    uint16_t total_chunks{0};
    uint32_t jpeg_size{0};
    uint64_t sim_time_ns{0};
    std::unordered_map<uint16_t, std::vector<uint8_t>> chunks;
  };

  void build_camera_info()
  {
    auto k = declare_parameter<std::vector<double>>(
      "camera_matrix", {320.0, 0.0, 320.0, 0.0, 320.0, 180.0, 0.0, 0.0, 1.0});
    if (k.size() != 9) {
      RCLCPP_WARN(get_logger(), "camera_matrix must have 9 elements; using default intrinsics");
      k = {320.0, 0.0, 320.0, 0.0, 320.0, 180.0, 0.0, 0.0, 1.0};
    }
    info_.d = declare_parameter<std::vector<double>>(
      "distortion_coefficients", {0.0, 0.0, 0.0, 0.0, 0.0});
    info_.distortion_model = declare_parameter<std::string>("distortion_model", "plumb_bob");
    info_.header.frame_id = frame_id_;
    info_.width = static_cast<uint32_t>(get_parameter("width").as_int());
    info_.height = static_cast<uint32_t>(get_parameter("height").as_int());
    for (size_t i = 0; i < 9; ++i) {
      info_.k[i] = k[i];
    }
    info_.p = {k[0], k[1], k[2], 0.0, k[3], k[4], k[5], 0.0, k[6], k[7], k[8], 0.0};
    info_.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  }

  bool open_socket()
  {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
      return false;
    }
    const int one = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    // Large receive buffer so high-rate frame bursts are not dropped before we read.
    const int rcvbuf = 16 * 1024 * 1024;
    ::setsockopt(fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
    timeval tv{1, 0};
    ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(bind_address_.c_str());
    addr.sin_port = htons(port_);
    if (::bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
      ::close(fd_);
      fd_ = -1;
      return false;
    }
    return true;
  }

  void receive_loop()
  {
    std::vector<uint8_t> buf(65536);
    while (running_) {
      const ssize_t n = ::recv(fd_, buf.data(), buf.size(), 0);
      if (n < static_cast<ssize_t>(sizeof(PacketHeader))) {
        continue;  // timeout or runt packet
      }
      handle_packet(buf.data(), static_cast<size_t>(n));
    }
  }

  void handle_packet(const uint8_t * packet, size_t len)
  {
    PacketHeader h;
    std::memcpy(&h, packet, sizeof(h));
    // Copy out of the packed struct (can't bind references to packed fields).
    const uint32_t frame_id = h.frame_id;
    const uint16_t chunk_id = h.chunk_id;
    const uint16_t total_chunks = h.total_chunks;
    const uint32_t jpeg_size = h.jpeg_size;
    const uint32_t payload_size = h.payload_size;
    const uint64_t sim_time_ns = h.sim_time_ns;

    if (sizeof(PacketHeader) + payload_size > len) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Dropped packet: payload truncated");
      return;
    }

    auto & fb = frames_[frame_id];
    if (fb.chunks.empty()) {
      fb.total_chunks = total_chunks;
      fb.jpeg_size = jpeg_size;
      fb.sim_time_ns = sim_time_ns;
    }
    fb.chunks.emplace(
      chunk_id,
      std::vector<uint8_t>(packet + sizeof(PacketHeader), packet + sizeof(PacketHeader) + payload_size));

    if (static_cast<int64_t>(frame_id) > latest_frame_id_) {
      latest_frame_id_ = frame_id;
      evict_stale(frame_id);
    }

    if (fb.chunks.size() >= fb.total_chunks) {
      FrameBuffer done = std::move(fb);
      frames_.erase(frame_id);
      publish_frame(done);
    }
  }

  void evict_stale(uint32_t current)
  {
    for (auto it = frames_.begin(); it != frames_.end(); ) {
      it = (it->first + 1 < current) ? frames_.erase(it) : std::next(it);
    }
  }

  void publish_frame(const FrameBuffer & fb)
  {
    const bool want_mono = image_raw_pub_->get_subscription_count() > 0;
    const bool want_rgb = image_rgb_pub_->get_subscription_count() > 0;
    if (!want_mono && !want_rgb) {
      return;  // nobody listening: skip the JPEG decode entirely
    }

    std::vector<uint8_t> jpeg;
    jpeg.reserve(fb.jpeg_size);
    for (uint16_t i = 0; i < fb.total_chunks; ++i) {
      const auto it = fb.chunks.find(i);
      if (it == fb.chunks.end()) {
        return;  // missing chunk (count matched via duplicate); drop frame
      }
      jpeg.insert(jpeg.end(), it->second.begin(), it->second.end());
    }

    const cv::Mat encoded(1, static_cast<int>(jpeg.size()), CV_8UC1, jpeg.data());
    const cv::Mat bgr = cv::imdecode(encoded, cv::IMREAD_COLOR);
    if (bgr.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "JPEG decode failed; dropping frame");
      return;
    }

    const rclcpp::Time stamp = use_sim_timestamp_
      ? rclcpp::Time(static_cast<int64_t>(fb.sim_time_ns))
      : now();

    if (want_mono) {
      image_raw_pub_->publish(to_image(bgr, "mono8", 1, cv::COLOR_BGR2GRAY, stamp));
    }
    if (want_rgb) {
      image_rgb_pub_->publish(to_image(bgr, "rgb8", 3, cv::COLOR_BGR2RGB, stamp));
    }
    if (camera_info_pub_->get_subscription_count() > 0) {
      auto info = std::make_unique<sensor_msgs::msg::CameraInfo>(info_);
      info->header.stamp = stamp;
      camera_info_pub_->publish(std::move(info));
    }
  }

  // Build an Image and write the colour conversion straight into its buffer.
  std::unique_ptr<sensor_msgs::msg::Image> to_image(
    const cv::Mat & bgr, const char * encoding, int channels, int conversion,
    const rclcpp::Time & stamp)
  {
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->header.stamp = stamp;
    msg->header.frame_id = frame_id_;
    msg->height = bgr.rows;
    msg->width = bgr.cols;
    msg->encoding = encoding;
    msg->is_bigendian = 0;
    msg->step = bgr.cols * channels;
    msg->data.resize(static_cast<size_t>(msg->step) * bgr.rows);
    cv::Mat dst(bgr.rows, bgr.cols, channels == 1 ? CV_8UC1 : CV_8UC3, msg->data.data(), msg->step);
    cv::cvtColor(bgr, dst, conversion);
    return msg;
  }

  std::string bind_address_;
  std::string frame_id_;
  bool use_sim_timestamp_{true};
  uint16_t port_{5600};
  int fd_{-1};
  std::atomic<bool> running_{false};
  std::thread rx_thread_;

  std::map<uint32_t, FrameBuffer> frames_;
  int64_t latest_frame_id_{-1};

  sensor_msgs::msg::CameraInfo info_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
};

}  // namespace aigp_camera_driver

RCLCPP_COMPONENTS_REGISTER_NODE(aigp_camera_driver::VisionDriverNode)
