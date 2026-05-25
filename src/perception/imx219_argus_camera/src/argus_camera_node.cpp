#include <chrono>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gst/app/gstappsink.h>
#include <gst/gst.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace
{
constexpr int kCameraInfoQueue = 10;

std::string join_range(const std::vector<double> & range)
{
  if (range.size() < 2) {
    return "";
  }

  std::ostringstream out;
  out << static_cast<long long>(range[0]) << " " << static_cast<long long>(range[1]);
  return out.str();
}
}  // namespace

class ArgusCameraNode : public rclcpp::Node
{
public:
  ArgusCameraNode()
  : Node("argus_camera_node")
  {
    gst_init(nullptr, nullptr);

    declare_parameter<int>("sensor_id", 0);
    declare_parameter<int>("sensor_mode", 4);
    declare_parameter<int>("width", 1280);
    declare_parameter<int>("height", 720);
    declare_parameter<double>("framerate", 60.0);
    declare_parameter<std::string>("output_encoding", "mono8");
    declare_parameter<std::string>("frame_id", "cam0");
    declare_parameter<int>("flip_method", 2);
    declare_parameter<bool>("publish_color", false);
    declare_parameter<std::string>("color_encoding", "rgb8");
    declare_parameter<bool>("color_reliable", false);
    declare_parameter<bool>("publish_h264", false);
    declare_parameter<bool>("enable_h264_stream", false);
    declare_parameter<std::string>("ground_station_ip", "127.0.0.1");
    declare_parameter<int>("stream_port", 5000);
    declare_parameter<int>("stream_bitrate", 4000000);
    declare_parameter<int>("iframe_interval", 30);
    declare_parameter<int>("payload_type", 96);
    declare_parameter<bool>("use_exposure_gain_range", false);
    declare_parameter<std::vector<double>>("exposure_time_range_ns", {1000000.0, 5000000.0});
    declare_parameter<std::vector<double>>("gain_range", {1.0, 8.0});
    declare_parameter<std::vector<double>>("camera_matrix", {
      559.381783658702, 0.0, 637.688519620621,
      0.0, 559.454491361351, 352.82329454271405,
      0.0, 0.0, 1.0});
    declare_parameter<std::vector<double>>("distortion_coefficients", {
      0.06702863120032837, -0.028541813941209592,
      -0.018213245805178305, 0.05872486156737494});
    declare_parameter<std::string>("distortion_model", "equidistant");

    frame_id_ = get_parameter("frame_id").as_string();
    encoding_ = get_parameter("output_encoding").as_string();
    color_encoding_ = get_parameter("color_encoding").as_string();
    publish_color_ = get_parameter("publish_color").as_bool();
    publish_h264_ = get_parameter("publish_h264").as_bool();
    width_ = get_parameter("width").as_int();
    height_ = get_parameter("height").as_int();
    channels_ = encoding_ == "mono8" ? 1 : 3;
    color_channels_ = color_encoding_ == "mono8" ? 1 : 3;

    image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "image_raw", rclcpp::SensorDataQoS().keep_last(1));
    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      "camera_info", rclcpp::QoS(kCameraInfoQueue));
    if (publish_color_) {
      if (get_parameter("color_reliable").as_bool()) {
        color_pub_ = create_publisher<sensor_msgs::msg::Image>(
          "image_rgb", rclcpp::QoS(1).reliable());
      } else {
        color_pub_ = create_publisher<sensor_msgs::msg::Image>(
          "image_rgb", rclcpp::SensorDataQoS().keep_last(1));
      }
    }
    if (publish_h264_) {
      h264_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
        "image_h264", rclcpp::SensorDataQoS().keep_last(1));
    }

    camera_info_ = make_camera_info();

    const auto pipeline = build_pipeline();
    RCLCPP_INFO(get_logger(), "Starting IMX219 Argus camera pipeline:");
    RCLCPP_INFO(get_logger(), "%s", pipeline.c_str());

    GError * error = nullptr;
    pipeline_ = gst_parse_launch(pipeline.c_str(), &error);
    if (error != nullptr) {
      const std::string message = error->message;
      g_error_free(error);
      throw std::runtime_error("Failed to parse camera pipeline: " + message);
    }
    if (pipeline_ == nullptr) {
      throw std::runtime_error("Failed to create camera pipeline");
    }

    appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
    if (appsink_ == nullptr) {
      throw std::runtime_error("Camera pipeline does not contain appsink name=sink");
    }
    g_signal_connect(appsink_, "new-sample", G_CALLBACK(&ArgusCameraNode::on_sample_static), this);

    if (publish_color_) {
      color_appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "color_sink");
      if (color_appsink_ == nullptr) {
        throw std::runtime_error("Camera pipeline does not contain appsink name=color_sink");
      }
      g_signal_connect(
        color_appsink_, "new-sample", G_CALLBACK(&ArgusCameraNode::on_color_sample_static), this);
    }
    if (publish_h264_) {
      h264_appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "h264_sink");
      if (h264_appsink_ == nullptr) {
        throw std::runtime_error("Camera pipeline does not contain appsink name=h264_sink");
      }
      g_signal_connect(
        h264_appsink_, "new-sample", G_CALLBACK(&ArgusCameraNode::on_h264_sample_static), this);
    }

    const GstStateChangeReturn state_ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (state_ret == GST_STATE_CHANGE_FAILURE) {
      throw std::runtime_error("Failed to start camera pipeline");
    }

    bus_timer_ = create_wall_timer(
      std::chrono::milliseconds(250),
      std::bind(&ArgusCameraNode::poll_bus, this));
  }

  ~ArgusCameraNode() override
  {
    if (pipeline_ != nullptr) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
    }
    if (appsink_ != nullptr) {
      gst_object_unref(appsink_);
      appsink_ = nullptr;
    }
    if (color_appsink_ != nullptr) {
      gst_object_unref(color_appsink_);
      color_appsink_ = nullptr;
    }
    if (h264_appsink_ != nullptr) {
      gst_object_unref(h264_appsink_);
      h264_appsink_ = nullptr;
    }
    if (pipeline_ != nullptr) {
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
    }
  }

private:
  std::string build_pipeline()
  {
    std::string exposure = "";
    std::string gain = "";
    if (get_parameter("use_exposure_gain_range").as_bool()) {
      exposure = join_range(get_parameter("exposure_time_range_ns").as_double_array());
      gain = join_range(get_parameter("gain_range").as_double_array());
    }
    const auto gst_format = encoding_ == "mono8" ? "GRAY8" : "RGB";
    const auto fps = get_parameter("framerate").as_double();
    const int flip_method = get_parameter("flip_method").as_int();
    const bool enable_stream = get_parameter("enable_h264_stream").as_bool();
    const bool publish_h264 = get_parameter("publish_h264").as_bool();

    std::ostringstream pipeline;
    pipeline
      << "nvarguscamerasrc sensor-id=" << get_parameter("sensor_id").as_int()
      << " sensor-mode=" << get_parameter("sensor_mode").as_int();
    if (!exposure.empty()) {
      pipeline << " exposuretimerange=\"" << exposure << "\"";
    }
    if (!gain.empty()) {
      pipeline << " gainrange=\"" << gain << "\"";
    }
    pipeline
      << " ! video/x-raw(memory:NVMM),width=" << width_
      << ",height=" << height_
      << ",format=NV12,framerate=" << static_cast<int>(fps) << "/1"
      << " ! tee name=t"
      << " t. ! queue max-size-buffers=1 leaky=downstream"
      << " ! nvvidconv"
      << " flip-method=" << flip_method
      << " ! video/x-raw,format=" << gst_format
      << " ! appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true";

    if (publish_color_) {
      pipeline
        << " t. ! queue max-size-buffers=1 leaky=downstream"
        << " ! nvvidconv"
        << " flip-method=" << flip_method;
      if (color_encoding_ == "mono8") {
        pipeline
          << " ! video/x-raw,format=GRAY8";
      } else {
        const auto color_gst_format = color_encoding_ == "bgr8" ? "BGR" : "RGB";
        pipeline
          << " ! video/x-raw,format=BGRx"
          << " ! videoconvert"
          << " ! video/x-raw,format=" << color_gst_format;
      }
      pipeline
        << " ! appsink name=color_sink emit-signals=true sync=false max-buffers=1 drop=true";
    }

    if (enable_stream || publish_h264) {
      pipeline
        << " t. ! queue max-size-buffers=1 leaky=downstream"
        << " ! nvv4l2h264enc"
        << " preset-level=0"
        << " insert-sps-pps=1"
        << " iframeinterval=" << get_parameter("iframe_interval").as_int()
        << " bitrate=" << get_parameter("stream_bitrate").as_int()
        << " control-rate=1"
        << " maxperf-enable=true"
        << " ! h264parse config-interval=1"
        << " ! tee name=h264t";

      if (publish_h264) {
        pipeline
          << " h264t. ! queue max-size-buffers=1 leaky=downstream"
          << " ! video/x-h264,stream-format=byte-stream,alignment=au"
          << " ! appsink name=h264_sink emit-signals=true sync=false max-buffers=1 drop=true";
      }

      if (enable_stream) {
        pipeline
          << " h264t. ! queue max-size-buffers=1 leaky=downstream"
          << " ! rtph264pay config-interval=1 pt=" << get_parameter("payload_type").as_int()
          << " ! udpsink host=" << get_parameter("ground_station_ip").as_string()
          << " port=" << get_parameter("stream_port").as_int()
          << " sync=false async=false";
      }
    }

    return pipeline.str();
  }

  sensor_msgs::msg::CameraInfo make_camera_info()
  {
    sensor_msgs::msg::CameraInfo info;
    info.header.frame_id = frame_id_;
    info.width = width_;
    info.height = height_;
    info.distortion_model = get_parameter("distortion_model").as_string();

    const auto k = get_parameter("camera_matrix").as_double_array();
    if (k.size() == info.k.size()) {
      std::copy(k.begin(), k.end(), info.k.begin());
      info.p = {
        k[0], k[1], k[2], 0.0,
        k[3], k[4], k[5], 0.0,
        k[6], k[7], k[8], 0.0};
    }

    const auto d = get_parameter("distortion_coefficients").as_double_array();
    info.d.assign(d.begin(), d.end());
    info.r = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0};

    return info;
  }

  static GstFlowReturn on_sample_static(GstElement * sink, gpointer data)
  {
    return static_cast<ArgusCameraNode *>(data)->on_sample(sink);
  }

  static GstFlowReturn on_color_sample_static(GstElement * sink, gpointer data)
  {
    return static_cast<ArgusCameraNode *>(data)->on_color_sample(sink);
  }

  static GstFlowReturn on_h264_sample_static(GstElement * sink, gpointer data)
  {
    return static_cast<ArgusCameraNode *>(data)->on_h264_sample(sink);
  }

  GstFlowReturn on_sample(GstElement * sink)
  {
    RCLCPP_INFO_ONCE(get_logger(), "on_sample callback triggered!");
    GstSample * sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
    if (sample == nullptr) {
      return GST_FLOW_ERROR;
    }

    GstBuffer * buffer = gst_sample_get_buffer(sample);
    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
      gst_sample_unref(sample);
      return GST_FLOW_ERROR;
    }

    const auto stamp = now();
    auto image = sensor_msgs::msg::Image();
    image.header.stamp = stamp;
    image.header.frame_id = frame_id_;
    image.height = height_;
    image.width = width_;
    image.encoding = encoding_;
    image.is_bigendian = false;
    image.step = width_ * channels_;
    image.data.assign(map.data, map.data + map.size);

    camera_info_.header.stamp = stamp;

    image_pub_->publish(std::move(image));
    camera_info_pub_->publish(camera_info_);

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
  }

  GstFlowReturn on_color_sample(GstElement * sink)
  {
    GstSample * sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
    if (sample == nullptr) {
      return GST_FLOW_ERROR;
    }

    GstBuffer * buffer = gst_sample_get_buffer(sample);
    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
      gst_sample_unref(sample);
      return GST_FLOW_ERROR;
    }

    auto image = sensor_msgs::msg::Image();
    image.header.stamp = now();
    image.header.frame_id = frame_id_;
    image.height = height_;
    image.width = width_;
    image.encoding = color_encoding_;
    image.is_bigendian = false;
    image.step = width_ * color_channels_;
    image.data.assign(map.data, map.data + map.size);

    color_pub_->publish(std::move(image));

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
  }

  GstFlowReturn on_h264_sample(GstElement * sink)
  {
    GstSample * sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
    if (sample == nullptr) {
      return GST_FLOW_ERROR;
    }

    GstBuffer * buffer = gst_sample_get_buffer(sample);
    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
      gst_sample_unref(sample);
      return GST_FLOW_ERROR;
    }

    auto msg = sensor_msgs::msg::CompressedImage();
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.format = "h264";
    msg.data.assign(map.data, map.data + map.size);

    h264_pub_->publish(std::move(msg));

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
  }

  void poll_bus()
  {
    GstBus * bus = gst_element_get_bus(pipeline_);
    if (bus == nullptr) {
      return;
    }

    while (GstMessage * msg = gst_bus_pop(bus)) {
      switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_ERROR: {
          GError * error = nullptr;
          gchar * debug = nullptr;
          gst_message_parse_error(msg, &error, &debug);
          RCLCPP_ERROR(
            get_logger(), "GStreamer error from %s: %s",
            GST_OBJECT_NAME(msg->src), error != nullptr ? error->message : "unknown");
          if (debug != nullptr) {
            RCLCPP_ERROR(get_logger(), "GStreamer debug: %s", debug);
          }
          if (error != nullptr) {
            g_error_free(error);
          }
          if (debug != nullptr) {
            g_free(debug);
          }
          break;
        }
        case GST_MESSAGE_WARNING: {
          GError * error = nullptr;
          gchar * debug = nullptr;
          gst_message_parse_warning(msg, &error, &debug);
          RCLCPP_WARN(
            get_logger(), "GStreamer warning from %s: %s",
            GST_OBJECT_NAME(msg->src), error != nullptr ? error->message : "unknown");
          if (debug != nullptr) {
            RCLCPP_WARN(get_logger(), "GStreamer debug: %s", debug);
          }
          if (error != nullptr) {
            g_error_free(error);
          }
          if (debug != nullptr) {
            g_free(debug);
          }
          break;
        }
        case GST_MESSAGE_EOS:
          RCLCPP_ERROR(get_logger(), "Camera pipeline reached EOS");
          break;
        default:
          break;
      }
      gst_message_unref(msg);
    }
    gst_object_unref(bus);
  }

  GstElement * pipeline_{nullptr};
  GstElement * appsink_{nullptr};
  GstElement * color_appsink_{nullptr};
  GstElement * h264_appsink_{nullptr};
  rclcpp::TimerBase::SharedPtr bus_timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr h264_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  sensor_msgs::msg::CameraInfo camera_info_;
  std::string frame_id_;
  std::string encoding_;
  std::string color_encoding_;
  bool publish_color_{false};
  bool publish_h264_{false};
  int width_{1280};
  int height_{720};
  int channels_{1};
  int color_channels_{3};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArgusCameraNode>());
  rclcpp::shutdown();
  return 0;
}
