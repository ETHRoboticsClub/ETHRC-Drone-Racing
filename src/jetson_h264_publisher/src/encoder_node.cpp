#include <chrono>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <gst/gst.h>

#include "rclcpp/rclcpp.hpp"

namespace
{
std::string bool_to_gst(const bool value)
{
  return value ? "true" : "false";
}
}  // namespace

class JetsonH264Streamer : public rclcpp::Node
{
public:
  JetsonH264Streamer()
  : Node("jetson_h264_streamer")
  {
    gst_init(nullptr, nullptr);

    declare_parameter<std::string>("ground_station_ip", "127.0.0.1");
    declare_parameter<int>("port", 5000);
    declare_parameter<int>("sensor_id", 0);
    declare_parameter<int>("sensor_mode", 4);
    declare_parameter<int>("width", 1280);
    declare_parameter<int>("height", 720);
    declare_parameter<int>("framerate", 60);
    declare_parameter<int>("bitrate", 4000000);
    declare_parameter<int>("iframe_interval", 30);
    declare_parameter<int>("preset_level", 0);
    declare_parameter<int>("control_rate", 1);
    declare_parameter<bool>("insert_sps_pps", true);
    declare_parameter<int>("payload_type", 96);
    declare_parameter<int>("config_interval", 1);
    declare_parameter<bool>("sync", false);
    declare_parameter<bool>("async", false);
    declare_parameter<bool>("enable_max_performance", true);
    declare_parameter<std::string>("source_options", "");
    declare_parameter<std::string>("encoder_options", "tune=zerolatency");

    const auto pipeline_str = build_pipeline();
    RCLCPP_INFO(get_logger(), "Starting Jetson H.264 RTP pipeline:");
    RCLCPP_INFO(get_logger(), "%s", pipeline_str.c_str());

    GError * error = nullptr;
    pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
    if (error != nullptr) {
      const std::string message = error->message;
      g_error_free(error);
      throw std::runtime_error("Failed to parse GStreamer pipeline: " + message);
    }
    if (pipeline_ == nullptr) {
      throw std::runtime_error("Failed to create GStreamer pipeline");
    }

    const GstStateChangeReturn state_ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (state_ret == GST_STATE_CHANGE_FAILURE) {
      throw std::runtime_error("Failed to start GStreamer pipeline");
    }

    bus_timer_ = create_wall_timer(
      std::chrono::milliseconds(250),
      std::bind(&JetsonH264Streamer::poll_bus, this));
  }

  ~JetsonH264Streamer() override
  {
    if (pipeline_ != nullptr) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
    }
  }

private:
  std::string build_pipeline()
  {
    const auto ground_station_ip = get_parameter("ground_station_ip").as_string();
    const auto source_options = get_parameter("source_options").as_string();
    const auto encoder_options = get_parameter("encoder_options").as_string();

    std::ostringstream pipeline;
    pipeline
      << "nvarguscamerasrc sensor-id=" << get_parameter("sensor_id").as_int()
      << " sensor-mode=" << get_parameter("sensor_mode").as_int();

    if (!source_options.empty()) {
      pipeline << " " << source_options;
    }

    pipeline
      << " ! video/x-raw(memory:NVMM),"
      << "width=" << get_parameter("width").as_int() << ","
      << "height=" << get_parameter("height").as_int() << ","
      << "format=NV12,"
      << "framerate=" << get_parameter("framerate").as_int() << "/1"
      << " ! nvv4l2h264enc"
      << " preset-level=" << get_parameter("preset_level").as_int()
      << " insert-sps-pps=" << (get_parameter("insert_sps_pps").as_bool() ? 1 : 0)
      << " iframeinterval=" << get_parameter("iframe_interval").as_int()
      << " bitrate=" << get_parameter("bitrate").as_int()
      << " control-rate=" << get_parameter("control_rate").as_int()
      << " maxperf-enable=" << bool_to_gst(get_parameter("enable_max_performance").as_bool());

    if (!encoder_options.empty()) {
      pipeline << " " << encoder_options;
    }

    pipeline
      << " ! h264parse"
      << " ! rtph264pay config-interval=" << get_parameter("config_interval").as_int()
      << " pt=" << get_parameter("payload_type").as_int()
      << " ! udpsink host=" << ground_station_ip
      << " port=" << get_parameter("port").as_int()
      << " sync=" << bool_to_gst(get_parameter("sync").as_bool())
      << " async=" << bool_to_gst(get_parameter("async").as_bool());

    return pipeline.str();
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
          RCLCPP_ERROR(get_logger(), "GStreamer pipeline reached EOS");
          break;
        default:
          break;
      }
      gst_message_unref(msg);
    }

    gst_object_unref(bus);
  }

  GstElement * pipeline_{nullptr};
  rclcpp::TimerBase::SharedPtr bus_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JetsonH264Streamer>());
  rclcpp::shutdown();
  return 0;
}
