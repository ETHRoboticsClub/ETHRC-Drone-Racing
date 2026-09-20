#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <array>
#include <atomic>
#include <mutex>
#include <thread>
#include <sstream>
#include <iomanip>
#include <limits>
extern "C" {
#include <libserialport.h>
}
#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <unistd.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <fstream>
#include <numeric>

using namespace std::chrono_literals;

class BlackboxDriverNode : public rclcpp::Node {
public:
    BlackboxDriverNode() : Node("betaflight_blackbox_driver") {
        this->declare_parameter("port", "/dev/ttyTHS1");
        this->declare_parameter("baud", 1500000);
        this->declare_parameter("imu_topic", "/imu0");
        this->declare_parameter("imu_filtered_topic", "/imu_filtered");
        this->declare_parameter("target_hz", 150);
        this->declare_parameter("intra_skip_fields", 13);
        this->declare_parameter("debug_raw", false);
        this->declare_parameter("enable_msp_control", false);
        this->declare_parameter("attitude_command_topic", "/drone_attitude_cmd");
        this->declare_parameter("motor_poles", 14);
        this->declare_parameter("rpm_topic", "/motor_rpm");
        this->declare_parameter("battery_topic", "/battery");
        this->declare_parameter("vbat_scale", 0.01); // Betaflight vbat is usually 10mV/unit
        this->declare_parameter("motor_topic", "/motor_commands");
        this->declare_parameter("joystick_topic", "/joystick_inputs");
        this->declare_parameter("min_battery_voltage", 14.0);
        this->declare_parameter("use_blackbox_time_for_stamp", false);
        this->declare_parameter("validate_imu_stream", true);
        this->declare_parameter("reject_zero_imu", true);
        this->declare_parameter("max_frame_delta_us", 10000);
        // Accept both old and new parameter names for backward compatibility
        this->declare_parameter("max_gyro_deg_s", 4000.0);
        this->declare_parameter("max_gyro_raw_deg_s", -1.0);  // deprecated alias
        this->declare_parameter("max_accel_g", 32.0);
        this->declare_parameter("max_gyro_jump_deg_s", 1000.0);
        this->declare_parameter("max_accel_jump_g", 8.0);
        this->declare_parameter("max_consecutive_rejects", 5);
        this->declare_parameter("outlier_sigma_k", 4.0);
        this->declare_parameter("diagnostics_topic", "/blackbox_diagnostics");

        std::string port_name = this->get_parameter("port").as_string();
        int baud = this->get_parameter("baud").as_int();
        imu_topic_ = this->get_parameter("imu_topic").as_string();
        int requested_target_hz = this->get_parameter("target_hz").as_int();
        target_hz_ = std::max(kMinImuPublishHz, std::min(requested_target_hz, kMaxImuPublishHz));
        publish_period_us_ = 1000000ULL / static_cast<uint64_t>(target_hz_);
        if (target_hz_ != requested_target_hz) {
            RCLCPP_WARN(this->get_logger(), "Clamped target_hz from %d to %d Hz (valid range: %d-%d Hz)",
                        requested_target_hz, target_hz_, kMinImuPublishHz, kMaxImuPublishHz);
        }
        intra_skip_fields_ = this->get_parameter("intra_skip_fields").as_int();
        debug_raw_ = this->get_parameter("debug_raw").as_bool();
        bool enable_msp_control = this->get_parameter("enable_msp_control").as_bool();
        std::string attitude_command_topic = this->get_parameter("attitude_command_topic").as_string();
        motor_poles_ = this->get_parameter("motor_poles").as_int();
        std::string rpm_topic = this->get_parameter("rpm_topic").as_string();
        std::string battery_topic = this->get_parameter("battery_topic").as_string();
        std::string motor_topic = this->get_parameter("motor_topic").as_string();
        std::string joystick_topic = this->get_parameter("joystick_topic").as_string();
        min_battery_voltage_ = this->get_parameter("min_battery_voltage").as_double();
        vbat_scale_ = this->get_parameter("vbat_scale").as_double();
        use_blackbox_time_for_stamp_ = this->get_parameter("use_blackbox_time_for_stamp").as_bool();
        validate_imu_stream_ = this->get_parameter("validate_imu_stream").as_bool();
        reject_zero_imu_ = this->get_parameter("reject_zero_imu").as_bool();
        max_frame_delta_us_ = std::max(1, static_cast<int>(this->get_parameter("max_frame_delta_us").as_int()));
        {
            // Backward compatibility: prefer max_gyro_deg_s, fall back to deprecated max_gyro_raw_deg_s
            double deprecated_val = this->get_parameter("max_gyro_raw_deg_s").as_double();
            if (deprecated_val > 0.0) {
                RCLCPP_WARN(this->get_logger(), "Parameter 'max_gyro_raw_deg_s' is deprecated; use 'max_gyro_deg_s' instead.");
                max_gyro_deg_s_ = std::max(1.0, deprecated_val);
            } else {
                max_gyro_deg_s_ = std::max(1.0, this->get_parameter("max_gyro_deg_s").as_double());
            }
        }
        max_accel_raw_ = std::max(1.0, this->get_parameter("max_accel_g").as_double()) * kAccelRawPerG;
        max_gyro_jump_deg_s_ = std::max(1.0, this->get_parameter("max_gyro_jump_deg_s").as_double());
        max_accel_jump_raw_ = std::max(1.0, this->get_parameter("max_accel_jump_g").as_double()) * kAccelRawPerG;
        max_consecutive_rejects_ = std::max(1, static_cast<int>(this->get_parameter("max_consecutive_rejects").as_int()));
        outlier_sigma_k_ = std::max(1.0, this->get_parameter("outlier_sigma_k").as_double());
        std::string diagnostics_topic = this->get_parameter("diagnostics_topic").as_string();

        auto qos_profile = rclcpp::SensorDataQoS();
        qos_profile.keep_last(1);
        std::string imu_filtered_topic = this->get_parameter("imu_filtered_topic").as_string();
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, qos_profile);
        imu_filtered_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_filtered_topic, qos_profile);
        rpm_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(rpm_topic, qos_profile);
        battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>(battery_topic, qos_profile);
        motor_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(motor_topic, qos_profile);
        rc_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(joystick_topic, qos_profile);
        diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic, 10);
        
        if (enable_msp_control) {
            // Subscription for Shared UART Control
            att_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                attitude_command_topic, 10, std::bind(&BlackboxDriverNode::attitudeCmdCallback, this, std::placeholders::_1));
            
            // Safety timer to disarm if no commands received
            watchdog_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&BlackboxDriverNode::safetyWatchdog, this));
            
            RCLCPP_INFO(this->get_logger(), "MSP Control ENABLED on %s for topic %s", port_name.c_str(), attitude_command_topic.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "MSP Control DISABLED (UART will be read-only for Blackbox)");
        }

        // 1. MANDATORY CLOCK CHECK (Jetson-specific for 1.5Mbps)
        if (baud >= 1000000) {
            std::ifstream clk_file("/sys/kernel/debug/bpmp/debug/clk/uarti/rate");
            if (clk_file.is_open()) {
                uint64_t rate;
                clk_file >> rate;
                if (rate < 80000000) {
                    RCLCPP_FATAL(this->get_logger(), "!!! FATAL: UART Clock Rate is too low (%lu Hz) for 1.5Mbps communication!", rate);
                    RCLCPP_FATAL(this->get_logger(), "Please run: echo 80000000 | sudo tee /sys/kernel/debug/bpmp/debug/clk/uarti/rate");
                    exit(1);
                }
                RCLCPP_INFO(this->get_logger(), "Verified UART Clock Rate: %lu Hz (Optimal for high-speed).", rate);
            } else {
                RCLCPP_WARN(this->get_logger(), "Could not verify UART Clock Rate. Ensure /sys/kernel/debug is accessible.");
            }
        }

        if (sp_get_port_by_name(port_name.c_str(), &port_) != SP_OK) {
            RCLCPP_ERROR(this->get_logger(), "Failed to find serial port: %s", port_name.c_str());
            return;
        }
        if (sp_open(port_, SP_MODE_READ_WRITE) != SP_OK) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
            return;
        }
        sp_set_baudrate(port_, baud);
        sp_set_bits(port_, 8);
        sp_set_parity(port_, SP_PARITY_NONE);
        sp_set_stopbits(port_, 1);
        sp_set_flowcontrol(port_, SP_FLOWCONTROL_NONE);

        // Enable low-latency mode via ioctl
        void* handle;
        if (sp_get_port_handle(port_, &handle) == SP_OK) {
            int fd = (int)(intptr_t)handle;
            struct serial_struct serial;
            if (ioctl(fd, TIOCGSERIAL, &serial) == 0) {
                serial.flags |= ASYNC_LOW_LATENCY;
                if (ioctl(fd, TIOCSSERIAL, &serial) == 0) {
                    RCLCPP_INFO(this->get_logger(), "Auto-enabled low-latency mode for %s", port_name.c_str());
                }
            }
        }

        sp_flush(port_, SP_BUF_BOTH);
        RCLCPP_INFO(this->get_logger(),
                    "Connected to Betaflight Blackbox on %s at %d baud (IMU: %d Hz, best-effort QoS, timer-paced latest sample, stamp source: %s, Debug: %s, validation: %s)",
                    port_name.c_str(), baud, target_hz_,
                    use_blackbox_time_for_stamp_ ? "blackbox time" : "ROS publish timer",
                    debug_raw_ ? "ON" : "OFF",
                    validate_imu_stream_ ? "ON" : "OFF");
        
        if (debug_raw_) {
            raw_dump_file_ = fopen("/tmp/capture.bbl", "wb");
            RCLCPP_INFO(this->get_logger(), "Raw binary dumping enabled -> /tmp/capture.bbl");
        }
        buffer_.resize(CIRCULAR_SIZE);
        imu_publish_timer_ = this->create_wall_timer(
            std::chrono::microseconds(publish_period_us_),
            std::bind(&BlackboxDriverNode::publishLatestIMU, this));
        thread_ = std::thread(&BlackboxDriverNode::readLoop, this);
        
        // RPM Health Check Timer (every 10 seconds)
        health_check_timer_ = this->create_wall_timer(std::chrono::seconds(10), std::bind(&BlackboxDriverNode::rpmHealthCheck, this));
        
        // Diagnostics Timer (every 2 seconds)
        diag_timer_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&BlackboxDriverNode::publishDiagnostics, this));
        
        // Try to Boost thread priority to Real-Time
        struct sched_param param;
        param.sched_priority = 80;
        if (pthread_setschedparam(thread_.native_handle(), SCHED_FIFO, &param) != 0) {
            RCLCPP_WARN(this->get_logger(), "Running without Real-Time priority (normal user mode).");
        }
    }

    virtual ~BlackboxDriverNode() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
        if (raw_dump_file_) fclose(raw_dump_file_);
        if (port_) {
            sp_close(port_);
            sp_free_port(port_);
        }
    }

private:
    struct FrameData {
        int32_t gyro[3];
        int32_t accel[3];
        uint32_t time;
        int32_t motor[4];
        int32_t erpm[4];
        int32_t vbat;
        int32_t rc_command[4];
        int32_t gyro_filtered[3];
        int32_t accel_filtered[3];
    };

    enum class ParseState { SYNC, HEADER, INTRA, INTER, MSP_V1, MSP_V2 };
    
    // Circular Buffer for high-rate parsing
    static constexpr size_t CIRCULAR_SIZE = 65536;
    static constexpr int kMaxFrameFields = 128;
    static constexpr int kFrameValueStart = 2;
    static constexpr double kAccelRawPerG = 2048.0;
    using DecodedFields = std::array<int32_t, kMaxFrameFields>;
    std::vector<uint8_t> buffer_;
    size_t head_ = 0; // Write index
    size_t tail_ = 0; // Read index

    struct FieldMap {
        int time_idx = -1;
        int gyro_idx[3] = {-1, -1, -1};
        int accel_idx[3] = {-1, -1, -1};
        int motor_idx[4] = {-1, -1, -1, -1};
        int erpm_idx[4] = {-1, -1, -1, -1};
        int vbat_idx = -1;
        int rc_command_idx[4] = {-1, -1, -1, -1};
        int gyro_filtered_idx[3] = {-1, -1, -1};
        int accel_filtered_idx[3] = {-1, -1, -1};
        int total_fields = 0;
    };

    ParseState state_ = ParseState::SYNC;
    FieldMap i_map_, p_map_;
    bool headers_locked_ = false;
    std::string header_line_buffer_;

    static constexpr int kMinImuPublishHz = 100;
    static constexpr int kMaxImuPublishHz = 200;
    static constexpr uint64_t kBlackboxTimeWrapUs = 1ULL << 32;
    int target_hz_ = 150;
    uint64_t publish_period_us_ = 1000000ULL / 150ULL;
    int intra_skip_fields_ = 13;
    bool debug_raw_ = false;
    bool validate_imu_stream_ = true;
    bool reject_zero_imu_ = true;
    int max_frame_delta_us_ = 10000;
    double max_gyro_deg_s_ = 4000.0;
    double max_accel_raw_ = 32.0 * kAccelRawPerG;
    double max_gyro_jump_deg_s_ = 1000.0;
    double max_accel_jump_raw_ = 8.0 * kAccelRawPerG;
    int max_consecutive_rejects_ = 5;
    double outlier_sigma_k_ = 4.0;
    std::chrono::steady_clock::time_point last_warning_time_;
    std::chrono::steady_clock::time_point first_data_time_;
    bool saw_first_data_ = false;

    FrameData last_frame_ = {0};
    FrameData prev_frame_ = {0};  // Frame before last_frame_, needed for AVERAGE_2 predictor
    bool has_first_intra_ = false;
    uint32_t last_iteration_ = 0;
    int consecutive_rejects_ = 0;
    bool has_gyro_mapped_ = false;
    bool has_accel_mapped_ = false;
    bool time_synced_ = false;
    int64_t time_offset_ns_ = 0;
    bool has_last_blackbox_time_ = false;
    uint32_t last_blackbox_time_us_ = 0;
    uint64_t blackbox_time_epoch_us_ = 0;
    bool has_last_imu_stamp_ = false;
    int64_t last_imu_stamp_ns_ = 0;

    struct sp_port* port_ = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_filtered_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rpm_pub_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rc_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr att_sub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::TimerBase::SharedPtr imu_publish_timer_;
    rclcpp::TimerBase::SharedPtr health_check_timer_;
    rclcpp::TimerBase::SharedPtr diag_timer_;
    
    int motor_poles_ = 14;
    std::string imu_topic_;
    bool motor_poles_from_header_ = false;
    bool has_erpm_mapped_ = false;
    bool has_motor_mapped_ = false;
    bool has_vbat_mapped_ = false;
    bool has_rc_command_mapped_ = false;
    bool has_gyro_filtered_mapped_ = false;
    bool has_accel_filtered_mapped_ = false;
    bool erpm_valid_ = false;
    double vbat_scale_ = 0.01;
    double min_battery_voltage_ = 14.0;
    std::atomic<bool> battery_critical_{false};
    bool use_blackbox_time_for_stamp_ = false;
    std::mutex timestamp_mutex_;
    std::mutex imu_mutex_;
    FrameData latest_imu_frame_ = {0};
    uint64_t latest_imu_sample_time_us_ = 0;
    bool has_pending_imu_sample_ = false;
    std::chrono::steady_clock::time_point last_nonzero_motor_time_;
    std::chrono::steady_clock::time_point last_erpm_check_time_;
    
    float max_rate_deg_s_ = 500.0;
    rclcpp::Time last_cmd_time_;
    uint16_t rc_channels_[8] = {1500, 1500, 1000, 1500, 1000, 1500, 1500, 1500}; // Default Safe State

    std::thread thread_;
    bool running_ = true;
    std::atomic<uint64_t> total_bytes_read_{0};
    std::atomic<uint64_t> dropped_stream_bytes_{0};
    std::atomic<uint64_t> decoded_i_frames_{0};
    std::atomic<uint64_t> decoded_p_frames_{0};
    std::atomic<uint64_t> accepted_imu_frames_{0};
    std::atomic<uint64_t> rejected_imu_frames_{0};
    std::atomic<uint64_t> resync_count_{0};
    std::atomic<uint64_t> validation_only_rejects_{0};  // Rejected by validation (not decode error)

    // Sliding-window outlier detection
    static constexpr int kValidationWindowSize = 8;
    struct ImuSampleWindow {
        std::array<int32_t, kValidationWindowSize> gyro_x{}, gyro_y{}, gyro_z{};
        std::array<int32_t, kValidationWindowSize> accel_x{}, accel_y{}, accel_z{};
        int count = 0;
        int head = 0;

        void push(const FrameData& f) {
            gyro_x[head] = f.gyro[0]; gyro_y[head] = f.gyro[1]; gyro_z[head] = f.gyro[2];
            accel_x[head] = f.accel[0]; accel_y[head] = f.accel[1]; accel_z[head] = f.accel[2];
            head = (head + 1) % kValidationWindowSize;
            if (count < kValidationWindowSize) count++;
        }

        static double meanAndStddev(const std::array<int32_t, kValidationWindowSize>& arr, int n, double& stddev) {
            double sum = 0;
            for (int i = 0; i < n; i++) sum += arr[i];
            double mean = sum / n;
            double sq_sum = 0;
            for (int i = 0; i < n; i++) sq_sum += (arr[i] - mean) * (arr[i] - mean);
            stddev = std::sqrt(sq_sum / n);
            return mean;
        }

        bool isOutlier(const FrameData& candidate, double sigma_k) const {
            if (count < 4) return false;  // Not enough history
            const std::array<int32_t, kValidationWindowSize>* arrays[] = {
                &gyro_x, &gyro_y, &gyro_z, &accel_x, &accel_y, &accel_z
            };
            int32_t candidate_vals[] = {
                candidate.gyro[0], candidate.gyro[1], candidate.gyro[2],
                candidate.accel[0], candidate.accel[1], candidate.accel[2]
            };
            for (int i = 0; i < 6; i++) {
                double stddev;
                double mean = meanAndStddev(*arrays[i], count, stddev);
                // Minimum stddev floor to avoid false positives when stationary
                double effective_stddev = std::max(stddev, 10.0);
                if (std::fabs(candidate_vals[i] - mean) > sigma_k * effective_stddev) {
                    return true;
                }
            }
            return false;
        }
    };
    ImuSampleWindow imu_window_;
    uint64_t last_health_bytes_ = 0;
    uint64_t last_health_decoded_i_ = 0;
    uint64_t last_health_decoded_p_ = 0;
    uint64_t last_health_accepted_ = 0;
    uint64_t last_health_rejected_ = 0;
    uint64_t last_health_dropped_bytes_ = 0;
    std::chrono::steady_clock::time_point last_stream_health_time_;
    FILE* raw_dump_file_ = nullptr;

    // Helper methods for circular buffer
    size_t queueSize() const {
        return (head_ + CIRCULAR_SIZE - tail_) % CIRCULAR_SIZE;
    }

    uint8_t peekByte(size_t offset = 0) const {
        return buffer_[(tail_ + offset) % CIRCULAR_SIZE];
    }

    void popBytes(size_t n) {
        tail_ = (tail_ + n) % CIRCULAR_SIZE;
    }

    bool hasCompleteFrame(ParseState s, size_t &out_len) const {
        if (s == ParseState::INTRA || s == ParseState::INTER) {
            const std::vector<int>& encodings = (s == ParseState::INTRA) ? i_encoding_ : p_encoding_;
            if (encodings.empty()) return false;
            
            size_t current_peek = 0;
            for (size_t fi = 0; fi < encodings.size();) {
                const int enc = encodings[fi];
                
                if (enc == 10 || enc == 9) { // NULL or G-Zero
                    fi++;
                    continue; 
                }

                if (current_peek >= queueSize()) return false;

                if (enc == 7) { // Tag2_3S32 (in firmware, Tag2_3S32 = 7)
                    if (fi + 2 >= encodings.size()) return false;
                    uint8_t lead = peekByte(current_peek);
                    current_peek++;
                    switch (lead >> 6) {
                        case 0: break; // 2-bit fields (0 extra bytes)
                        case 1: { // 4-bit fields (1 extra byte)
                            if (queueSize() < current_peek + 1) return false;
                            current_peek += 1;
                            break;
                        }
                        case 2: { // 6-bit fields (2 extra bytes)
                            if (queueSize() < current_peek + 2) return false;
                            current_peek += 2;
                            break;
                        }
                        case 3: { // Variable width
                            uint8_t sel = lead;
                            for (int i = 0; i < 3; i++) {
                                int width_bits = sel & 0x03;
                                size_t bytes_needed = (width_bits == 3) ? 4 : (width_bits + 1);
                                if (queueSize() < current_peek + bytes_needed) return false;
                                current_peek += bytes_needed;
                                sel >>= 2;
                            }
                            break;
                        }
                    }
                    fi += 3;
                } else if (enc == 6) { // Tag8_8SVB group decoding (in firmware, Tag8_8SVB = 6)
                    size_t group_size = 0;
                    while (fi + group_size < encodings.size() && encodings[fi + group_size] == 6) {
                        group_size++;
                    }
                    if (group_size == 1) {
                        bool found_end = false;
                        while (current_peek < queueSize()) {
                            if (!(peekByte(current_peek++) & 0x80)) {
                                found_end = true;
                                break;
                            }
                        }
                        if (!found_end) return false;
                        fi++;
                    } else {
                        if (current_peek >= queueSize()) return false;
                        uint8_t header = peekByte(current_peek++);
                        for (size_t i = 0; i < group_size; i++) {
                            if (header & (1 << i)) {
                                bool found_end = false;
                                while (current_peek < queueSize()) {
                                    if (!(peekByte(current_peek++) & 0x80)) {
                                        found_end = true;
                                        break;
                                    }
                                }
                                if (!found_end) return false;
                            }
                        }
                        fi += group_size;
                    }
                } else { // Variable Bit (VB) variants
                    bool found_end = false;
                    while (current_peek < queueSize()) {
                        if (!(peekByte(current_peek++) & 0x80)) {
                            found_end = true;
                            break;
                        }
                    }
                    if (!found_end) return false;
                    fi++;
                }
            }
            out_len = current_peek;
            // Require 1 extra byte for CRC
            if (queueSize() <= current_peek) return false;
            return true;
        } else if (s == ParseState::MSP_V1) {
            // Header: $M[Direction][Size][Cmd]...[Checksum]
            // We've already popped '$M', sopeek(0)=Dir, peek(1)=Size
            if (queueSize() < 2) return false; 
            uint8_t size = peekByte(1);
            out_len = size + 2;
            return queueSize() >= (size_t)(size + 3); // Dir + Size + Cmd + Payload + Checksum
        } else if (s == ParseState::MSP_V2) {
            // Header: $X[Direction][Flags][CmdLo][CmdHi][SizeLo][SizeHi]...[CRC]
            if (queueSize() < 6) return false;
            uint16_t size = peekByte(4) | (peekByte(5) << 8);
            out_len = size + 6;
            return queueSize() >= (size_t)(size + 7); // Dir + Flag + Cmd(2) + Size(2) + Payload + CRC
        }
        return false;
    }

    uint32_t decodeVBUI() {
        uint32_t result = 0;
        int shift = 0;
        while (queueSize() > 0 && shift < 32) {
            uint8_t b = peekByte();
            popBytes(1);
            result |= (uint32_t)(b & 0x7F) << shift;
            if (!(b & 0x80)) return result;
            shift += 7;
        }
        return result; 
    }

    int32_t decodeVBSI() {
        uint32_t u = decodeVBUI();
        const int32_t value = static_cast<int32_t>(u >> 1);
        return (u & 1U) ? -value - 1 : value;
    }

    // Tag2_3S32 decoder: decodes 3 signed values from a compact encoding
    // Reference: betaflight/blackbox-tools/src/decoders.c
    void decodeTag2_3S32(int32_t out[3]) {
        if (queueSize() < 1) { out[0]=out[1]=out[2]=0; return; }
        uint8_t lead = peekByte(); popBytes(1);
        switch (lead >> 6) {
            case 0: // 2-bit fields
                out[0] = (int32_t)(int8_t)((lead >> 4) & 0x03) - ((lead >> 4) & 0x02 ? 4 : 0); // sign extend 2-bit
                out[1] = (int32_t)(int8_t)((lead >> 2) & 0x03) - ((lead >> 2) & 0x02 ? 4 : 0);
                out[2] = (int32_t)(int8_t)((lead     ) & 0x03) - ((lead     ) & 0x02 ? 4 : 0);
                break;
            case 1: { // 4-bit fields
                out[0] = (int8_t)((lead & 0x0F) | ((lead & 0x08) ? 0xF0 : 0)); // sign extend 4-bit
                if (queueSize() < 1) { out[1]=out[2]=0; return; }
                uint8_t b2 = peekByte(); popBytes(1);
                out[1] = (int8_t)((b2 >> 4) | ((b2 & 0x80) ? 0xF0 : 0));
                out[2] = (int8_t)((b2 & 0x0F) | ((b2 & 0x08) ? 0xF0 : 0));
                break;
            }
            case 2: { // 6-bit fields
                out[0] = (int8_t)((lead & 0x3F) | ((lead & 0x20) ? 0xC0 : 0));
                if (queueSize() < 2) { out[1]=out[2]=0; return; }
                uint8_t b2 = peekByte(); popBytes(1);
                out[1] = (int8_t)((b2 & 0x3F) | ((b2 & 0x20) ? 0xC0 : 0));
                uint8_t b3 = peekByte(); popBytes(1);
                out[2] = (int8_t)((b3 & 0x3F) | ((b3 & 0x20) ? 0xC0 : 0));
                break;
            }
            case 3: { // Variable width (8/16/24/32 bit per value)
                uint8_t sel = lead;
                for (int i = 0; i < 3; i++) {
                    switch (sel & 0x03) {
                        case 0: { // 8-bit
                            if (queueSize() < 1) { out[i]=0; continue; }
                            out[i] = (int8_t)peekByte(); popBytes(1);
                            break;
                        }
                        case 1: { // 16-bit
                            if (queueSize() < 2) { out[i]=0; continue; }
                            uint8_t lo = peekByte(); popBytes(1);
                            uint8_t hi = peekByte(); popBytes(1);
                            out[i] = (int16_t)(lo | (hi << 8));
                            break;
                        }
                        case 2: { // 24-bit
                            if (queueSize() < 3) { out[i]=0; continue; }
                            uint8_t b1 = peekByte(); popBytes(1);
                            uint8_t b2 = peekByte(); popBytes(1);
                            uint8_t b3 = peekByte(); popBytes(1);
                            int32_t v = b1 | (b2 << 8) | (b3 << 16);
                            if (v & 0x800000) v |= 0xFF000000; // sign extend 24-bit
                            out[i] = v;
                            break;
                        }
                        case 3: { // 32-bit
                            if (queueSize() < 4) { out[i]=0; continue; }
                            uint8_t b1 = peekByte(); popBytes(1);
                            uint8_t b2 = peekByte(); popBytes(1);
                            uint8_t b3 = peekByte(); popBytes(1);
                            uint8_t b4 = peekByte(); popBytes(1);
                            out[i] = (int32_t)(b1 | (b2 << 8) | (b3 << 16) | (b4 << 24));
                            break;
                        }
                    }
                    sel >>= 2;
                }
                break;
            }
        }
    }

    // Parse encoding/predictor headers: "H Field I encoding:1,1,3,0,..." → vector of ints
    std::vector<int> i_encoding_, p_encoding_;
    std::vector<int> i_predictor_, p_predictor_;

    // Betaflight P-frame predictor types (from blackbox_fielddefs.h)
    static constexpr int PRED_ZERO = 0;
    static constexpr int PRED_PREVIOUS = 1;
    static constexpr int PRED_STRAIGHT_LINE = 2;
    static constexpr int PRED_AVERAGE_2 = 3;

    int32_t applyPredictor(int pred_type, int32_t wire_val, int32_t prev1, int32_t prev2) const {
        switch (pred_type) {
            case PRED_ZERO:          return wire_val;
            case PRED_PREVIOUS:      return prev1 + wire_val;
            case PRED_STRAIGHT_LINE: return 2 * prev1 - prev2 + wire_val;
            case PRED_AVERAGE_2:     return (prev1 + prev2) / 2 + wire_val;
            default:                 return prev1 + wire_val;
        }
    }

    int getPPredictor(int local_field_idx) const {
        const int abs_idx = kFrameValueStart + local_field_idx;
        if (abs_idx >= 0 && abs_idx < static_cast<int>(p_predictor_.size())) {
            return p_predictor_[abs_idx];
        }
        return PRED_PREVIOUS;
    }

    static uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a) {
        crc ^= a;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0xD5;
            } else {
                crc <<= 1;
            }
        }
        return crc;
    }

    void readLoop() {
        std::vector<uint8_t> raw_buf(4096);
        while (running_ && rclcpp::ok()) {
            int bytes = sp_blocking_read(port_, raw_buf.data(), raw_buf.size(), 1);
            if (bytes > 0) {
                if (raw_dump_file_) { fwrite(raw_buf.data(), 1, bytes, raw_dump_file_); fflush(raw_dump_file_); }
                total_bytes_read_.fetch_add(static_cast<uint64_t>(bytes), std::memory_order_relaxed);
                for (int i = 0; i < bytes; ++i) {
                    size_t next_head = (head_ + 1) % CIRCULAR_SIZE;
                    if (next_head == tail_) {
                        dropped_stream_bytes_.fetch_add(static_cast<uint64_t>(bytes - i), std::memory_order_relaxed);
                        break;
                    }
                    buffer_[head_] = raw_buf[i];
                    head_ = next_head;
                }
                processData();
            }
        }
    }

    bool decodeFields(const std::vector<int>& encodings, DecodedFields& frame, int& total_fields) {
        total_fields = static_cast<int>(encodings.size());
        if (total_fields <= 0 || total_fields > kMaxFrameFields) {
            return false;
        }

        int fi = 0;
        while (fi < total_fields) {
            int enc = encodings[fi];
            if (enc == 1) { // Unsigned VB
                frame[fi] = static_cast<int32_t>(decodeVBUI());
                fi++;
            } else if (enc == 0 || enc == 3) { // Signed VB / NEG_14BIT (enc == 3)
                frame[fi] = decodeVBSI();
                fi++;
            } else if (enc == 6) { // Tag8_8SVB group decoding (in firmware, Tag8_8SVB = 6)
                int group_size = 0;
                while (fi + group_size < total_fields && encodings[fi + group_size] == 6) {
                    group_size++;
                }
                if (group_size == 1) {
                    frame[fi] = decodeVBSI();
                    fi++;
                } else {
                    if (queueSize() < 1) return false;
                    uint8_t header = peekByte();
                    popBytes(1);
                    for (int i = 0; i < group_size; i++) {
                        if (header & (1 << i)) {
                            frame[fi + i] = decodeVBSI();
                        } else {
                            frame[fi + i] = 0;
                        }
                    }
                    fi += group_size;
                }
            } else if (enc == 10 || enc == 9) { // NULL / G-Zero
                frame[fi] = 0;
                fi++;
            } else if (enc == 7) { // Tag2_3S32 decodes 3 fields at once (in firmware, Tag2_3S32 = 7)
                if (fi + 2 >= total_fields) {
                    return false;
                }
                int32_t vals[3];
                decodeTag2_3S32(vals);
                frame[fi] = vals[0];
                frame[fi + 1] = vals[1];
                frame[fi + 2] = vals[2];
                fi += 3;
            } else { // Unknown encoding, try signed VB
                frame[fi] = decodeVBSI();
                fi++;
            }
        }

        return true;
    }

    bool readMappedField(const DecodedFields& frame, int total_fields, int local_idx, int32_t& value) const {
        const int absolute_idx = kFrameValueStart + local_idx;
        if (local_idx < 0 || absolute_idx < 0 || absolute_idx >= total_fields || absolute_idx >= kMaxFrameFields) {
            return false;
        }

        value = frame[absolute_idx];
        return true;
    }

    bool buildIntraFrame(const DecodedFields& frame, int total_fields, FrameData& candidate, std::string& reason) const {
        if (total_fields < kFrameValueStart) {
            reason = "short I-frame";
            return false;
        }

        candidate = {};
        candidate.time = static_cast<uint32_t>(frame[1]);

        if (!has_gyro_mapped_ || !has_accel_mapped_) {
            reason = "missing gyro or accel mapping";
            return false;
        }

        for (int axis = 0; axis < 3; axis++) {
            if (!readMappedField(frame, total_fields, i_map_.gyro_idx[axis], candidate.gyro[axis])) {
                reason = "gyro I-frame index out of range";
                return false;
            }
            if (!readMappedField(frame, total_fields, i_map_.accel_idx[axis], candidate.accel[axis])) {
                reason = "accel I-frame index out of range";
                return false;
            }
        }

        for (int motor = 0; motor < 4; motor++) {
            if (has_erpm_mapped_ && i_map_.erpm_idx[motor] >= 0) {
                if (!readMappedField(frame, total_fields, i_map_.erpm_idx[motor], candidate.erpm[motor])) {
                    reason = "eRPM I-frame index out of range";
                    return false;
                }
            }
            if (has_motor_mapped_ && i_map_.motor_idx[motor] >= 0) {
                if (!readMappedField(frame, total_fields, i_map_.motor_idx[motor], candidate.motor[motor])) {
                    reason = "motor I-frame index out of range";
                    return false;
                }
            }
        }

        if (i_map_.vbat_idx >= 0) {
            if (!readMappedField(frame, total_fields, i_map_.vbat_idx, candidate.vbat)) {
                reason = "vbat I-frame index out of range";
                return false;
            }
        }

        for (int rc = 0; rc < 4; rc++) {
            if (has_rc_command_mapped_ && i_map_.rc_command_idx[rc] >= 0) {
                if (!readMappedField(frame, total_fields, i_map_.rc_command_idx[rc], candidate.rc_command[rc])) {
                    reason = "rcCommand I-frame index out of range";
                    return false;
                }
            }
        }

        for (int axis = 0; axis < 3; axis++) {
            if (has_gyro_filtered_mapped_ && i_map_.gyro_filtered_idx[axis] >= 0) {
                if (!readMappedField(frame, total_fields, i_map_.gyro_filtered_idx[axis], candidate.gyro_filtered[axis])) {
                    reason = "gyro_filtered I-frame index out of range";
                    return false;
                }
            }
            if (has_accel_filtered_mapped_ && i_map_.accel_filtered_idx[axis] >= 0) {
                if (!readMappedField(frame, total_fields, i_map_.accel_filtered_idx[axis], candidate.accel_filtered[axis])) {
                    reason = "accel_filtered I-frame index out of range";
                    return false;
                }
            }
        }

        return true;
    }

    bool buildInterFrame(const DecodedFields& frame, int total_fields, FrameData& candidate, std::string& reason) const {
        if (total_fields < kFrameValueStart) {
            reason = "short P-frame";
            return false;
        }

        candidate = {};

        // Time (field index 1): uses STRAIGHT_LINE predictor in BF
        {
            const int time_pred = (1 < static_cast<int>(p_predictor_.size()))
                ? p_predictor_[1] : PRED_PREVIOUS;
            candidate.time = static_cast<uint32_t>(applyPredictor(
                time_pred, frame[1],
                static_cast<int32_t>(last_frame_.time),
                static_cast<int32_t>(prev_frame_.time)));
        }

        // Validate actual time step (not the raw wire residual)
        {
            const int32_t actual_dt = static_cast<int32_t>(candidate.time - last_frame_.time);
            if (actual_dt <= 0 || actual_dt > max_frame_delta_us_) {
                reason = "implausible P-frame time delta (" + std::to_string(actual_dt) + " us)";
                return false;
            }
        }

        // Gyro and Accel: use per-field predictor (typically AVERAGE_2 in BF)
        for (int axis = 0; axis < 3; axis++) {
            if (has_gyro_mapped_) {
                int32_t wire_val = 0;
                if (!readMappedField(frame, total_fields, p_map_.gyro_idx[axis], wire_val)) {
                    reason = "gyro P-frame index out of range";
                    return false;
                }
                const int pred = getPPredictor(p_map_.gyro_idx[axis]);
                candidate.gyro[axis] = applyPredictor(pred, wire_val,
                    last_frame_.gyro[axis], prev_frame_.gyro[axis]);
            }
            if (has_accel_mapped_) {
                int32_t wire_val = 0;
                if (!readMappedField(frame, total_fields, p_map_.accel_idx[axis], wire_val)) {
                    reason = "accel P-frame index out of range";
                    return false;
                }
                const int pred = getPPredictor(p_map_.accel_idx[axis]);
                candidate.accel[axis] = applyPredictor(pred, wire_val,
                    last_frame_.accel[axis], prev_frame_.accel[axis]);
            }
        }

        // Motors and eRPM: use per-field predictor
        for (int motor = 0; motor < 4; motor++) {
            if (has_erpm_mapped_ && erpm_valid_ && p_map_.erpm_idx[motor] >= 0) {
                int32_t wire_val = 0;
                if (!readMappedField(frame, total_fields, p_map_.erpm_idx[motor], wire_val)) {
                    reason = "eRPM P-frame index out of range";
                    return false;
                }
                const int pred = getPPredictor(p_map_.erpm_idx[motor]);
                candidate.erpm[motor] = applyPredictor(pred, wire_val,
                    last_frame_.erpm[motor], prev_frame_.erpm[motor]);
            }
            if (has_motor_mapped_ && p_map_.motor_idx[motor] >= 0) {
                int32_t wire_val = 0;
                if (!readMappedField(frame, total_fields, p_map_.motor_idx[motor], wire_val)) {
                    reason = "motor P-frame index out of range";
                    return false;
                }
                const int pred = getPPredictor(p_map_.motor_idx[motor]);
                candidate.motor[motor] = applyPredictor(pred, wire_val,
                    last_frame_.motor[motor], prev_frame_.motor[motor]);
            }
        }

        // vbat: uses PREVIOUS predictor via TAG8_8SVB
        if (has_vbat_mapped_ && p_map_.vbat_idx >= 0) {
            int32_t wire_val = 0;
            if (!readMappedField(frame, total_fields, p_map_.vbat_idx, wire_val)) {
                reason = "vbat P-frame index out of range";
                return false;
            }
            const int pred = getPPredictor(p_map_.vbat_idx);
            candidate.vbat = applyPredictor(pred, wire_val,
                last_frame_.vbat, prev_frame_.vbat);
        }

        // rcCommand: uses per-field predictor
        for (int rc = 0; rc < 4; rc++) {
            if (has_rc_command_mapped_ && p_map_.rc_command_idx[rc] >= 0) {
                int32_t wire_val = 0;
                if (!readMappedField(frame, total_fields, p_map_.rc_command_idx[rc], wire_val)) {
                    reason = "rcCommand P-frame index out of range";
                    return false;
                }
                const int pred = getPPredictor(p_map_.rc_command_idx[rc]);
                candidate.rc_command[rc] = applyPredictor(pred, wire_val,
                    last_frame_.rc_command[rc], prev_frame_.rc_command[rc]);
            }
        }

        // Filtered gyro/accel: use per-field predictor
        for (int axis = 0; axis < 3; axis++) {
            if (has_gyro_filtered_mapped_ && p_map_.gyro_filtered_idx[axis] >= 0) {
                int32_t wire_val = 0;
                if (!readMappedField(frame, total_fields, p_map_.gyro_filtered_idx[axis], wire_val)) {
                    reason = "gyro_filtered P-frame index out of range";
                    return false;
                }
                const int pred = getPPredictor(p_map_.gyro_filtered_idx[axis]);
                candidate.gyro_filtered[axis] = applyPredictor(pred, wire_val,
                    last_frame_.gyro_filtered[axis], prev_frame_.gyro_filtered[axis]);
            }
            if (has_accel_filtered_mapped_ && p_map_.accel_filtered_idx[axis] >= 0) {
                int32_t wire_val = 0;
                if (!readMappedField(frame, total_fields, p_map_.accel_filtered_idx[axis], wire_val)) {
                    reason = "accel_filtered P-frame index out of range";
                    return false;
                }
                const int pred = getPPredictor(p_map_.accel_filtered_idx[axis]);
                candidate.accel_filtered[axis] = applyPredictor(pred, wire_val,
                    last_frame_.accel_filtered[axis], prev_frame_.accel_filtered[axis]);
            }
        }

        return true;
    }

    bool validateImuFrame(const FrameData& candidate, bool is_delta_frame, std::string& reason) const {
        if (!validate_imu_stream_) {
            return true;
        }

        bool all_zero = true;
        for (int axis = 0; axis < 3; axis++) {
            all_zero = all_zero && candidate.gyro[axis] == 0 && candidate.accel[axis] == 0;

            if (std::fabs(static_cast<double>(candidate.gyro[axis])) > max_gyro_deg_s_) {
                reason = "gyro outside configured range";
                return false;
            }
            if (std::fabs(static_cast<double>(candidate.accel[axis])) > max_accel_raw_) {
                reason = "accel outside configured range";
                return false;
            }

            if (is_delta_frame) {
                const double gyro_jump = std::fabs(static_cast<double>(candidate.gyro[axis] - last_frame_.gyro[axis]));
                const double accel_jump = std::fabs(static_cast<double>(candidate.accel[axis] - last_frame_.accel[axis]));
                if (gyro_jump > max_gyro_jump_deg_s_) {
                    reason = "gyro jump outside configured range";
                    return false;
                }
                if (accel_jump > max_accel_jump_raw_) {
                    reason = "accel jump outside configured range";
                    return false;
                }
            }
        }

        if (reject_zero_imu_ && all_zero) {
            reason = "all IMU axes decoded as zero";
            return false;
        }

        // Sliding-window outlier detection
        if (imu_window_.isOutlier(candidate, outlier_sigma_k_)) {
            reason = "statistical outlier vs recent window";
            return false;
        }

        return true;
    }

    void rejectFrame(const char* frame_type, const std::string& reason, bool lose_delta_reference) {
        rejected_imu_frames_.fetch_add(1, std::memory_order_relaxed);
        consecutive_rejects_++;

        if (lose_delta_reference) {
            has_first_intra_ = false;
            erpm_valid_ = false;
            imu_window_ = ImuSampleWindow{};  // Reset outlier window on resync
            resync_count_.fetch_add(1, std::memory_order_relaxed);
        } else {
            validation_only_rejects_.fetch_add(1, std::memory_order_relaxed);
        }

        // Hard resync: if too many consecutive rejects, flush the entire buffer
        if (consecutive_rejects_ >= max_consecutive_rejects_) {
            RCLCPP_WARN(this->get_logger(),
                "Hard resync: %d consecutive rejects, flushing buffer",
                consecutive_rejects_);
            tail_ = head_;  // Flush circular buffer
            state_ = ParseState::SYNC;
            has_first_intra_ = false;
            erpm_valid_ = false;
            imu_window_ = ImuSampleWindow{};
            consecutive_rejects_ = 0;
            resync_count_.fetch_add(1, std::memory_order_relaxed);
        }

        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Rejected %s Blackbox frame (%s)%s",
            frame_type, reason.c_str(), lose_delta_reference ? "; waiting for next I-frame" : "");
    }

    void acceptFrame(const FrameData& candidate, bool is_intra = false) {
        prev_frame_ = last_frame_;
        last_frame_ = candidate;
        if (is_intra) {
            // On I-frame, both history slots = same frame (no meaningful prev2 yet)
            prev_frame_ = candidate;
        }
        consecutive_rejects_ = 0;
        imu_window_.push(candidate);
        accepted_imu_frames_.fetch_add(1, std::memory_order_relaxed);
        publishIMU();
        publishRPM();
        publishBattery();
        publishMotors();
        publishRCCommands();
    }

    void processData() {
        while (queueSize() > 0) {
            if (state_ == ParseState::SYNC) {
                uint8_t b = peekByte();
                if (b == 'I') {
                    if (!saw_first_data_) {
                        saw_first_data_ = true;
                        first_data_time_ = std::chrono::steady_clock::now();
                    }
                    if (!headers_locked_) {
                        auto now = std::chrono::steady_clock::now();
                        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_warning_time_).count() > 2) {
                            RCLCPP_INFO(this->get_logger(), "Awaiting Dictionary (H)... Waiting for you to ARM the drone!");
                            last_warning_time_ = now;
                        }
                        popBytes(1);
                        continue;
                    }
                    state_ = ParseState::INTRA;
                    popBytes(1);
                } else if (b == 'P') {
                    if (headers_locked_) {
                        state_ = ParseState::INTER;
                        popBytes(1);
                    } else {
                        popBytes(1);
                    }
                } else if (b == 'H') {
                    if (queueSize() > 1) {
                        if (peekByte(1) == ' ') {
                            state_ = ParseState::HEADER;
                            header_line_buffer_.clear();
                            // Keep 'H' for HEADER logic
                        } else {
                            popBytes(1);
                        }
                    } else break;
                } else if (b == '$') {
                    if (queueSize() > 1) {
                        uint8_t protocol = peekByte(1);
                        if (protocol == 'M') {
                            state_ = ParseState::MSP_V1;
                            popBytes(2); // Consume $M
                        } else if (protocol == 'X') {
                            state_ = ParseState::MSP_V2;
                            popBytes(2); // Consume $X
                        } else {
                            popBytes(1); // Just noise
                        }
                    } else break;
                } else {
                    // FAST SYNC: Skip until we find a marker 'I', 'P', 'H' or '$'
                    size_t sz = queueSize();
                    size_t skip = 0;
                    for (size_t i = 1; i < sz; ++i) {
                        uint8_t next = peekByte(i);
                        if (next == 'I' || next == 'P' || next == 'H' || next == '$') {
                            skip = i;
                            break;
                        }
                    }
                    if (skip > 0) popBytes(skip);
                    else if (sz > 1) popBytes(sz - 1);
                    else popBytes(1);
                }
            } else if (state_ == ParseState::HEADER) {
                uint8_t b = peekByte();
                popBytes(1);
                if (b == '\n' || b == '\r') {
                    if (!header_line_buffer_.empty()) parseHeaderLine(header_line_buffer_);
                    header_line_buffer_.clear();
                    state_ = ParseState::SYNC;
                } else if (b == 'H' && queueSize() > 1) {
                    // Check for deterministic split markers: "H " or "H Field"
                    bool is_split = (peekByte(1) == ' ');
                    if (!is_split && queueSize() > 5) {
                        // Check if the next 5 bytes are "Field"
                        std::string next5 = "";
                        for(int i=1; i<=5; ++i) next5 += (char)peekByte(i);
                        if (next5 == "Field") is_split = true;
                    }

                    if (is_split) {
                        if (!header_line_buffer_.empty()) parseHeaderLine(header_line_buffer_);
                        header_line_buffer_ = "H"; // Fix: preserve the 'H' for the next line
                        // Stay in HEADER state to process the payload
                    } else {
                        header_line_buffer_ += (char)b;
                    }
                } else {
                    header_line_buffer_ += (char)b;
                    if (header_line_buffer_.length() > 2048) {
                        RCLCPP_WARN(this->get_logger(), "Header line too long (>2k), force resetting to SYNC.");
                        header_line_buffer_.clear();
                        state_ = ParseState::SYNC;
                    }
                }
            } else {
                if (queueSize() > 8192) {
                    state_ = ParseState::SYNC;
                    tail_ = head_;
                    continue;
                }

                size_t payload_len = 0;
                if (!hasCompleteFrame(state_, payload_len)) break; 

                if (state_ == ParseState::INTRA) {
                    // Compute CRC-8
                    uint8_t crc = crc8_dvb_s2(0, 'I');
                    for (size_t i = 0; i < payload_len; i++) {
                        crc = crc8_dvb_s2(crc, peekByte(i));
                    }
                    uint8_t received_crc = peekByte(payload_len);
                    if (crc != received_crc) {
                        rejectFrame("I", "CRC mismatch (computed 0x" + 
                            std::to_string(crc) + ", got 0x" + std::to_string(received_crc) + ")", false);
                        popBytes(1); // skip the first byte of payload
                        state_ = ParseState::SYNC;
                        continue;
                    }

                    try {
                        decoded_i_frames_.fetch_add(1, std::memory_order_relaxed);
                        DecodedFields frame = {};
                        int total_fields = 0;
                        FrameData candidate = {};
                        std::string reason;

                        if (!decodeFields(i_encoding_, frame, total_fields)) {
                            rejectFrame("I", "unable to decode fields", false);
                        } else if (!buildIntraFrame(frame, total_fields, candidate, reason)) {
                            rejectFrame("I", reason, false);
                        } else {
                            // I-frame structural integrity: check iteration monotonicity
                            uint32_t iteration = static_cast<uint32_t>(frame[0]);
                            if (has_first_intra_ && iteration != 0 && iteration <= last_iteration_ &&
                                (last_iteration_ - iteration) < (1U << 31)) {
                                rejectFrame("I", "iteration not monotonic", false);
                            } else if (!validateImuFrame(candidate, false, reason)) {
                                rejectFrame("I", reason, false);
                            } else {
                                last_iteration_ = iteration;
                                erpm_valid_ = has_erpm_mapped_;
                                if (i_map_.vbat_idx >= 0) {
                                    has_vbat_mapped_ = true;
                                }
                                has_first_intra_ = true;
                                acceptFrame(candidate, true);  // is_intra=true resets history
                            }
                        }
                    } catch (const std::exception& e) {
                        rejectFrame("I", std::string("decode exception: ") + e.what(), false);
                    } catch (...) {
                        rejectFrame("I", "unknown decode exception", false);
                    }
                    popBytes(1); // Consume the CRC byte
                    state_ = ParseState::SYNC;
                } else if (state_ == ParseState::INTER) {
                    if (!has_first_intra_) { 
                        popBytes(payload_len + 1); // skip payload + CRC
                        state_ = ParseState::SYNC; 
                        continue; 
                    }

                    // Compute CRC-8
                    uint8_t crc = crc8_dvb_s2(0, 'P');
                    for (size_t i = 0; i < payload_len; i++) {
                        crc = crc8_dvb_s2(crc, peekByte(i));
                    }
                    uint8_t received_crc = peekByte(payload_len);
                    if (crc != received_crc) {
                        rejectFrame("P", "CRC mismatch (computed 0x" + 
                            std::to_string(crc) + ", got 0x" + std::to_string(received_crc) + ")", true);
                        popBytes(1); // skip the first byte of payload
                        state_ = ParseState::SYNC;
                        continue;
                    }

                    try {
                        decoded_p_frames_.fetch_add(1, std::memory_order_relaxed);
                        DecodedFields frame = {};
                        int total_fields = 0;
                        FrameData candidate = {};
                        std::string reason;

                        if (!decodeFields(p_encoding_, frame, total_fields)) {
                            // Decode error: stream is out of sync, must resync
                            rejectFrame("P", "unable to decode fields", true);
                        } else if (!buildInterFrame(frame, total_fields, candidate, reason)) {
                            // Build error (e.g. field index OOB): stream may be corrupted, resync
                            rejectFrame("P", reason, true);
                        } else if (!validateImuFrame(candidate, true, reason)) {
                            // Validation-only error: the decode was fine but values are implausible.
                            // Do NOT lose the delta reference — just skip this one frame.
                            // The delta chain remains intact because last_frame_ is unchanged.
                            rejectFrame("P", reason, false);
                        } else {
                            acceptFrame(candidate);
                        }
                    } catch (const std::exception& e) {
                        rejectFrame("P", std::string("decode exception: ") + e.what(), true);
                    } catch (...) {
                        rejectFrame("P", "unknown decode exception", true);
                    }
                    popBytes(1); // Consume the CRC byte
                    state_ = ParseState::SYNC;
                } else if (state_ == ParseState::MSP_V1) {
                    popBytes(payload_len + 1); // Consume Direction + Size + Cmd + Payload + Checksum
                    state_ = ParseState::SYNC;
                } else if (state_ == ParseState::MSP_V2) {
                    popBytes(payload_len + 1); // Consume Direction + Flags + Cmd(2) + Size(2) + Payload + CRC
                    state_ = ParseState::SYNC;
                }
            }
        }
    }

    bool updateBlackboxSampleTime(uint64_t& sample_time_us) {
        std::lock_guard<std::mutex> lock(timestamp_mutex_);
        const uint32_t raw_time_us = last_frame_.time;

        if (has_last_blackbox_time_) {
            if (raw_time_us == last_blackbox_time_us_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Dropping IMU sample with repeated Betaflight timestamp (%u us)", raw_time_us);
                return false;
            }

            if (raw_time_us < last_blackbox_time_us_) {
                const uint32_t backwards_us = last_blackbox_time_us_ - raw_time_us;
                if (backwards_us > (1U << 31)) {
                    blackbox_time_epoch_us_ += kBlackboxTimeWrapUs;
                } else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "Betaflight timestamp jumped backwards by %u us; resynchronizing IMU time",
                        backwards_us);
                    blackbox_time_epoch_us_ = 0;
                    time_synced_ = false;
                }
            }
        }

        sample_time_us = blackbox_time_epoch_us_ + static_cast<uint64_t>(raw_time_us);
        last_blackbox_time_us_ = raw_time_us;
        has_last_blackbox_time_ = true;
        return true;
    }

    builtin_interfaces::msg::Time stampFromNanoseconds(int64_t stamp_ns) const {
        if (stamp_ns < 0) {
            stamp_ns = 0;
        }

        builtin_interfaces::msg::Time stamp;
        stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
        stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
        return stamp;
    }

    builtin_interfaces::msg::Time makeImuStamp(uint64_t sample_time_us) {
        std::lock_guard<std::mutex> lock(timestamp_mutex_);
        const int64_t sample_time_ns = static_cast<int64_t>(sample_time_us) * 1000LL;
        const int64_t now_ns = this->now().nanoseconds();
        const int64_t candidate_offset_ns = now_ns - sample_time_ns;

        if (!time_synced_) {
            time_offset_ns_ = candidate_offset_ns;
            time_synced_ = true;
        } else if (candidate_offset_ns < time_offset_ns_) {
            // If we are draining buffered data faster than real time, keep stamps out of the future.
            time_offset_ns_ = candidate_offset_ns;
        }

        int64_t stamp_ns = sample_time_ns + time_offset_ns_;
        if (has_last_imu_stamp_ && stamp_ns <= last_imu_stamp_ns_) {
            stamp_ns = last_imu_stamp_ns_ + 1;
        }

        last_imu_stamp_ns_ = stamp_ns;
        has_last_imu_stamp_ = true;
        return stampFromNanoseconds(stamp_ns);
    }

    builtin_interfaces::msg::Time makePublishTimerStamp() {
        std::lock_guard<std::mutex> lock(timestamp_mutex_);
        int64_t stamp_ns = this->now().nanoseconds();
        if (has_last_imu_stamp_ && stamp_ns <= last_imu_stamp_ns_) {
            stamp_ns = last_imu_stamp_ns_ + 1;
        }

        last_imu_stamp_ns_ = stamp_ns;
        has_last_imu_stamp_ = true;
        return stampFromNanoseconds(stamp_ns);
    }

    void publishIMU() {
        static int total_frames = 0;
        if (++total_frames % 5000 == 0) {
            RCLCPP_INFO(this->get_logger(), "Parsed 5000 frames. Last Accel Z: %d", last_frame_.accel[2]);
        }

        uint64_t sample_time_us = 0;
        if (!updateBlackboxSampleTime(sample_time_us)) {
            return;
        }

        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            latest_imu_frame_ = last_frame_;
            latest_imu_sample_time_us_ = sample_time_us;
            has_pending_imu_sample_ = true;
        }
    }

    void publishLatestIMU() {
        FrameData frame;
        uint64_t sample_time_us = 0;

        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            if (!has_pending_imu_sample_) {
                return;
            }
            frame = latest_imu_frame_;
            sample_time_us = latest_imu_sample_time_us_;
            has_pending_imu_sample_ = false;
        }

        auto msg = sensor_msgs::msg::Imu();

        msg.header.stamp = use_blackbox_time_for_stamp_
            ? makeImuStamp(sample_time_us)
            : makePublishTimerStamp();
        msg.header.frame_id = "imu_link";

        const float G_TO_MS2 = 9.80665f;
        const float ACCEL_SCALE = 1.0f / 2048.0f;  // acc_1G=2048 confirmed from header
        const float GYRO_SCALE = M_PI / 180.0f;    // gyroADC is already deg/s, only need deg→rad

        msg.linear_acceleration.x = frame.accel[0] * ACCEL_SCALE * G_TO_MS2;
        msg.linear_acceleration.y = frame.accel[1] * ACCEL_SCALE * G_TO_MS2;
        msg.linear_acceleration.z = frame.accel[2] * ACCEL_SCALE * G_TO_MS2;

        msg.angular_velocity.x = frame.gyro[0] * GYRO_SCALE;
        msg.angular_velocity.y = frame.gyro[1] * GYRO_SCALE;
        msg.angular_velocity.z = frame.gyro[2] * GYRO_SCALE;

        imu_pub_->publish(msg);

        if (has_gyro_filtered_mapped_) {
            auto filtered_msg = sensor_msgs::msg::Imu();
            filtered_msg.header = msg.header;
            filtered_msg.header.frame_id = "imu_link";
            
            filtered_msg.angular_velocity.x = frame.gyro_filtered[0] * GYRO_SCALE;
            filtered_msg.angular_velocity.y = frame.gyro_filtered[1] * GYRO_SCALE;
            filtered_msg.angular_velocity.z = frame.gyro_filtered[2] * GYRO_SCALE;
            
            if (has_accel_filtered_mapped_) {
                filtered_msg.linear_acceleration.x = frame.accel_filtered[0] * ACCEL_SCALE * G_TO_MS2;
                filtered_msg.linear_acceleration.y = frame.accel_filtered[1] * ACCEL_SCALE * G_TO_MS2;
                filtered_msg.linear_acceleration.z = frame.accel_filtered[2] * ACCEL_SCALE * G_TO_MS2;
            } else {
                filtered_msg.linear_acceleration = msg.linear_acceleration;
            }
            
            imu_filtered_pub_->publish(filtered_msg);
        }
    }

    void publishRPM() {
        if (!has_erpm_mapped_ || !erpm_valid_) return;

        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data.resize(4);

        // Convert eRPM/100 -> mechanical RPM
        // true_RPM = (raw_eRPM * 100) / (motor_poles / 2)
        double pole_pairs = motor_poles_ / 2.0;
        for (int i = 0; i < 4; i++) {
            double erpm = last_frame_.erpm[i] * 100.0;
            msg.data[i] = erpm / pole_pairs;
        }

        rpm_pub_->publish(msg);
    }

    void publishBattery() {
        if (!has_vbat_mapped_) return;
        
        static int battery_subsample = 0;
        if (++battery_subsample < 40) return; // Publish at ~10Hz if IMU is 400Hz
        battery_subsample = 0;

        auto msg = sensor_msgs::msg::BatteryState();
        msg.header.stamp = this->now();
        msg.header.frame_id = "imu_link";
        msg.voltage = last_frame_.vbat * (float)vbat_scale_;
        
        if (msg.voltage < min_battery_voltage_) {
            battery_critical_ = true;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "LOW BATTERY DETECTED! Current voltage: %.2f V, Minimum: %.2f V", msg.voltage, min_battery_voltage_);
        } else {
            battery_critical_ = false;
        }

        msg.current = std::numeric_limits<float>::quiet_NaN();
        msg.charge = std::numeric_limits<float>::quiet_NaN();
        msg.capacity = std::numeric_limits<float>::quiet_NaN();
        msg.design_capacity = std::numeric_limits<float>::quiet_NaN();
        msg.percentage = std::numeric_limits<float>::quiet_NaN();
        msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
        msg.power_supply_health = battery_critical_ 
            ? sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_DEAD 
            : sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
        msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
        msg.present = true;

        battery_pub_->publish(msg);
    }

    void publishMotors() {
        if (!has_motor_mapped_) return;

        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data.resize(4);
        for (int i = 0; i < 4; i++) {
            msg.data[i] = static_cast<double>(last_frame_.motor[i]);
        }
        motor_pub_->publish(msg);
    }

    void publishRCCommands() {
        if (!has_rc_command_mapped_) return;

        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data.resize(4);
        for (int i = 0; i < 4; i++) {
            msg.data[i] = static_cast<double>(last_frame_.rc_command[i]);
        }
        rc_pub_->publish(msg);
    }

    void rpmHealthCheck() {
        reportStreamHealth();

        const size_t imu_publishers = this->count_publishers(imu_topic_);
        if (imu_publishers > 1) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                "Detected %zu publishers on %s. Foxglove may show interleaved header stamps if another IMU node is active.",
                imu_publishers, imu_topic_.c_str());
        }

        if (has_erpm_mapped_ && erpm_valid_) {
            bool all_zero = true;
            bool motor_spinning = false;
            for (int i = 0; i < 4; i++) {
                if (last_frame_.erpm[i] != 0) all_zero = false;
                if (last_frame_.motor[i] > 1100) motor_spinning = true;
            }

            if (all_zero && motor_spinning) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000, 
                    "eRPM values are all zero but motor outputs are non-zero! Check DShot Bidirectional settings.");
            }
        } else if (headers_locked_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                "eRPM fields NOT found in Blackbox header! Ensure Bidirectional DShot is enabled.");
        }
    }

    void reportStreamHealth() {
        const auto now = std::chrono::steady_clock::now();
        if (last_stream_health_time_.time_since_epoch().count() == 0) {
            last_stream_health_time_ = now;
            last_health_bytes_ = total_bytes_read_.load(std::memory_order_relaxed);
            last_health_decoded_i_ = decoded_i_frames_.load(std::memory_order_relaxed);
            last_health_decoded_p_ = decoded_p_frames_.load(std::memory_order_relaxed);
            last_health_accepted_ = accepted_imu_frames_.load(std::memory_order_relaxed);
            last_health_rejected_ = rejected_imu_frames_.load(std::memory_order_relaxed);
            last_health_dropped_bytes_ = dropped_stream_bytes_.load(std::memory_order_relaxed);
            return;
        }

        const double dt = std::chrono::duration<double>(now - last_stream_health_time_).count();
        if (dt <= 0.0) {
            return;
        }

        const uint64_t bytes = total_bytes_read_.load(std::memory_order_relaxed);
        const uint64_t decoded_i = decoded_i_frames_.load(std::memory_order_relaxed);
        const uint64_t decoded_p = decoded_p_frames_.load(std::memory_order_relaxed);
        const uint64_t accepted = accepted_imu_frames_.load(std::memory_order_relaxed);
        const uint64_t rejected = rejected_imu_frames_.load(std::memory_order_relaxed);
        const uint64_t dropped = dropped_stream_bytes_.load(std::memory_order_relaxed);
        const uint64_t delta_bytes = bytes - last_health_bytes_;
        const uint64_t delta_decoded = (decoded_i - last_health_decoded_i_) + (decoded_p - last_health_decoded_p_);
        const uint64_t delta_accepted = accepted - last_health_accepted_;
        const uint64_t delta_rejected = rejected - last_health_rejected_;
        const uint64_t delta_dropped = dropped - last_health_dropped_bytes_;

        if (headers_locked_) {
            RCLCPP_INFO(this->get_logger(),
                "Blackbox stream health: %.1f kB/s, decoded %.1f fps (I:%llu P:%llu), accepted %.1f fps, rejected %llu, resyncs %llu",
                (static_cast<double>(delta_bytes) / dt) / 1000.0,
                static_cast<double>(delta_decoded) / dt,
                static_cast<unsigned long long>(decoded_i),
                static_cast<unsigned long long>(decoded_p),
                static_cast<double>(delta_accepted) / dt,
                static_cast<unsigned long long>(delta_rejected),
                static_cast<unsigned long long>(resync_count_.load(std::memory_order_relaxed)));
        } else if (delta_bytes > 0) {
            RCLCPP_INFO(this->get_logger(),
                "Blackbox bytes are arriving (%.1f kB/s), waiting for header dictionary before validating IMU frames",
                (static_cast<double>(delta_bytes) / dt) / 1000.0);
        }

        if (delta_rejected > 0) {
            RCLCPP_WARN(this->get_logger(),
                "Rejected %llu malformed/implausible Blackbox frame(s) in the last %.1f s; incoming stream is not clean or parser is out of sync",
                static_cast<unsigned long long>(delta_rejected), dt);
        }

        if (delta_dropped > 0) {
            RCLCPP_WARN(this->get_logger(),
                "Dropped %llu serial byte(s) because the parser buffer filled; lower Blackbox bandwidth or increase parser throughput",
                static_cast<unsigned long long>(delta_dropped));
        }

        last_stream_health_time_ = now;
        last_health_bytes_ = bytes;
        last_health_decoded_i_ = decoded_i;
        last_health_decoded_p_ = decoded_p;
        last_health_accepted_ = accepted;
        last_health_rejected_ = rejected;
        last_health_dropped_bytes_ = dropped;
    }

    void publishDiagnostics() {
        auto diag_msg = diagnostic_msgs::msg::DiagnosticArray();
        diag_msg.header.stamp = this->now();

        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = "Betaflight Blackbox Driver";
        status.hardware_id = "betaflight_blackbox";

        const uint64_t accepted = accepted_imu_frames_.load(std::memory_order_relaxed);
        const uint64_t rejected = rejected_imu_frames_.load(std::memory_order_relaxed);
        const uint64_t resyncs = resync_count_.load(std::memory_order_relaxed);
        const uint64_t val_only = validation_only_rejects_.load(std::memory_order_relaxed);
        const uint64_t decoded_i = decoded_i_frames_.load(std::memory_order_relaxed);
        const uint64_t decoded_p = decoded_p_frames_.load(std::memory_order_relaxed);
        const uint64_t total_decoded = decoded_i + decoded_p;

        const double reject_pct = total_decoded > 0
            ? (100.0 * rejected / total_decoded) : 0.0;

        if (battery_critical_) {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            status.message = "Low battery voltage!";
        } else if (!headers_locked_) {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            status.message = "Awaiting header dictionary";
        } else if (reject_pct > 10.0) {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            status.message = "High frame rejection rate";
        } else if (resyncs > 0 && reject_pct > 2.0) {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            status.message = "Elevated resyncs or rejects";
        } else {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            status.message = "Stream healthy";
        }

        auto kv = [](const std::string& key, const std::string& val) {
            diagnostic_msgs::msg::KeyValue kv;
            kv.key = key;
            kv.value = val;
            return kv;
        };

        status.values.push_back(kv("accepted_frames", std::to_string(accepted)));
        status.values.push_back(kv("rejected_frames", std::to_string(rejected)));
        status.values.push_back(kv("reject_pct", std::to_string(reject_pct)));
        status.values.push_back(kv("resyncs", std::to_string(resyncs)));
        status.values.push_back(kv("validation_only_rejects", std::to_string(val_only)));
        status.values.push_back(kv("decoded_i_frames", std::to_string(decoded_i)));
        status.values.push_back(kv("decoded_p_frames", std::to_string(decoded_p)));
        status.values.push_back(kv("has_first_intra", has_first_intra_ ? "true" : "false"));
        status.values.push_back(kv("headers_locked", headers_locked_ ? "true" : "false"));
        status.values.push_back(kv("consecutive_rejects", std::to_string(consecutive_rejects_)));
        status.values.push_back(kv("battery_critical", battery_critical_ ? "true" : "false"));
        status.values.push_back(kv("battery_voltage", std::to_string(last_frame_.vbat * vbat_scale_) + " V"));

        diag_msg.status.push_back(status);
        diag_pub_->publish(diag_msg);
    }

    void parseHeaderLine(const std::string& line) {
        if (line.find("Field I name:") != std::string::npos) {
            mapFields(line, i_map_);
            RCLCPP_INFO(this->get_logger(), "Parsed Field I name (%d fields)", i_map_.total_fields);
        }
        else if (line.find("Field I encoding:") != std::string::npos) {
            i_encoding_ = parseEncodingHeader(line);
        }
        else if (line.find("Field I predictor:") != std::string::npos) {
            i_predictor_ = parseEncodingHeader(line);  // Same CSV int format
            RCLCPP_INFO(this->get_logger(), "Parsed Field I predictor (%zu entries)", i_predictor_.size());
        }
        else if (line.find("Field P name:") != std::string::npos) {
            mapFields(line, p_map_);
            RCLCPP_INFO(this->get_logger(), "Parsed Field P name (%d fields)", p_map_.total_fields);
        }
        else if (line.find("Field P encoding:") != std::string::npos) {
            p_encoding_ = parseEncodingHeader(line);
            std::stringstream ss;
            for (int enc : p_encoding_) ss << enc << ",";
            RCLCPP_INFO(this->get_logger(), "Parsed Field P encoding values: %s", ss.str().c_str());
        }
        else if (line.find("Field P predictor:") != std::string::npos) {
            p_predictor_ = parseEncodingHeader(line);
            RCLCPP_INFO(this->get_logger(), "Parsed Field P predictor (%zu entries)", p_predictor_.size());
        }
        
        // Finalize mapping state
        if (i_map_.gyro_idx[0] != -1 && i_map_.accel_idx[0] != -1) {
            has_gyro_mapped_ = true;
            has_accel_mapped_ = true;
        }
        if (i_map_.gyro_filtered_idx[0] != -1) {
            has_gyro_filtered_mapped_ = true;
        }
        if (i_map_.accel_filtered_idx[0] != -1) {
            has_accel_filtered_mapped_ = true;
        }
        if (i_map_.erpm_idx[0] != -1) {
            has_erpm_mapped_ = true;
        }
        if (i_map_.motor_idx[0] != -1) {
            has_motor_mapped_ = true;
        }
        if (i_map_.rc_command_idx[0] != -1) {
            has_rc_command_mapped_ = true;
        }

        if (i_map_.total_fields > 0 && !headers_locked_) {
            // BF 4.5+ doesn't send "Field P name:" — P frames use the same field layout
            if (p_map_.total_fields == 0) p_map_ = i_map_;
            headers_locked_ = true;
            RCLCPP_INFO(this->get_logger(), "Dictionary locked (I:%d fields, P:%d fields)",
                        i_map_.total_fields, p_map_.total_fields);
            
            if (has_erpm_mapped_) {
                RCLCPP_INFO(this->get_logger(), "eRPM fields mapped (indices: %d,%d,%d,%d)",
                            i_map_.erpm_idx[0], i_map_.erpm_idx[1], i_map_.erpm_idx[2], i_map_.erpm_idx[3]);
            }
            
            if (i_map_.vbat_idx != -1) {
                RCLCPP_INFO(this->get_logger(), "vbat field mapped (index: %d)", i_map_.vbat_idx);
            }

            if (has_rc_command_mapped_) {
                RCLCPP_INFO(this->get_logger(), "rcCommand fields mapped (indices: %d,%d,%d,%d)",
                            i_map_.rc_command_idx[0], i_map_.rc_command_idx[1], i_map_.rc_command_idx[2], i_map_.rc_command_idx[3]);
            }

            if (has_gyro_filtered_mapped_ || has_accel_filtered_mapped_) {
                RCLCPP_INFO(this->get_logger(), "Filtered IMU fields mapped (Gyro: %s, Accel: %s)",
                            has_gyro_filtered_mapped_ ? "YES" : "NO",
                            has_accel_filtered_mapped_ ? "YES" : "NO");
            }

            if (!has_gyro_mapped_) RCLCPP_WARN(this->get_logger(), "Gyro fields not found in header");
            if (!has_accel_mapped_) RCLCPP_WARN(this->get_logger(), "Accel fields not found in header");
            if (!has_erpm_mapped_) {
                RCLCPP_WARN(this->get_logger(), "eRPM fields NOT found in Blackbox header!");
                RCLCPP_WARN(this->get_logger(), "Ensure Bidirectional DShot is enabled in Betaflight Motors tab.");
            }
        }

        // Parse motor_poles from "H motor_poles:14"
        if (line.find("motor_poles:") != std::string::npos) {
            size_t pos = line.find("motor_poles:");
            std::string val = line.substr(pos + 12);
            try {
                int poles = std::stoi(val);
                if (poles >= 2 && poles <= 50) {
                    motor_poles_ = poles;
                    motor_poles_from_header_ = true;
                    RCLCPP_INFO(this->get_logger(), "Auto-detected motor_poles = %d from header", motor_poles_);
                }
            } catch (...) {}
        }
    }

    std::vector<int> parseEncodingHeader(const std::string& line) {
        std::vector<int> result;
        size_t pos = line.find(":");
        if (pos == std::string::npos) return result;
        std::string values = line.substr(pos + 1);
        std::stringstream ss(values);
        std::string token;
        while (std::getline(ss, token, ',')) {
            try { result.push_back(std::stoi(token)); } catch (...) { result.push_back(0); }
        }
        return result;
    }

    void mapFields(const std::string& line, FieldMap& map) {
        size_t pos = line.find(":");
        if (pos == std::string::npos) return;
        std::vector<std::string> fields;
        std::string item;
        std::stringstream ss(line.substr(pos + 1));
        while (std::getline(ss, item, ',')) {
            fields.push_back(item);
        }

        if (line.find("vbat") != std::string::npos) {
            RCLCPP_INFO(this->get_logger(), "Header line containing vbat: %s", line.c_str());
        }
        map.total_fields = 0;
        int start_idx = (line.find("Field I") != std::string::npos) ? 2 : 1;

        for (int i = start_idx; i < (int)fields.size(); ++i) {
            std::string f = fields[i];
            // Poor man's trim (removes leading/trailing spaces/newlines)
            f.erase(0, f.find_first_not_of(" \n\r\t"));
            f.erase(f.find_last_not_of(" \n\r\t") + 1);

            int local_idx = i - start_idx;
            if (f == "gyroUnfilt[0]" || f == "gyroRaw[0]" || f == "gi[0]") map.gyro_idx[0] = local_idx;
            else if (f == "gyroUnfilt[1]" || f == "gyroRaw[1]" || f == "gi[1]") map.gyro_idx[1] = local_idx;
            else if (f == "gyroUnfilt[2]" || f == "gyroRaw[2]" || f == "gi[2]") map.gyro_idx[2] = local_idx;
            else if (f == "gyroADC[0]") { if (map.gyro_idx[0] == -1) map.gyro_idx[0] = local_idx; }
            else if (f == "gyroADC[1]") { if (map.gyro_idx[1] == -1) map.gyro_idx[1] = local_idx; }
            else if (f == "gyroADC[2]") { if (map.gyro_idx[2] == -1) map.gyro_idx[2] = local_idx; }
            else if (f == "accelRaw[0]" || f == "ai[0]") map.accel_idx[0] = local_idx;
            else if (f == "accelRaw[1]" || f == "ai[1]") map.accel_idx[1] = local_idx;
            else if (f == "accelRaw[2]" || f == "ai[2]") map.accel_idx[2] = local_idx;
            else if (f == "accelADC[0]" || f == "accSmooth[0]") { if (map.accel_idx[0] == -1) map.accel_idx[0] = local_idx; }
            else if (f == "accelADC[1]" || f == "accSmooth[1]") { if (map.accel_idx[1] == -1) map.accel_idx[1] = local_idx; }
            else if (f == "accelADC[2]" || f == "accSmooth[2]") { if (map.accel_idx[2] == -1) map.accel_idx[2] = local_idx; }
            else if (f == "motor[0]") map.motor_idx[0] = local_idx;
            else if (f == "motor[1]") map.motor_idx[1] = local_idx;
            else if (f == "motor[2]") map.motor_idx[2] = local_idx;
            else if (f == "motor[3]") map.motor_idx[3] = local_idx;
            else if (f == "eRPM[0]")  map.erpm_idx[0] = local_idx;
            else if (f == "eRPM[1]")  map.erpm_idx[1] = local_idx;
            else if (f == "eRPM[2]")  map.erpm_idx[2] = local_idx;
            else if (f == "eRPM[3]")  map.erpm_idx[3] = local_idx;
            else if (f == "vbatLatest" || f == "vbat") map.vbat_idx = local_idx;
            else if (f == "rcCommand[0]") map.rc_command_idx[0] = local_idx;
            else if (f == "rcCommand[1]") map.rc_command_idx[1] = local_idx;
            else if (f == "rcCommand[2]") map.rc_command_idx[2] = local_idx;
            else if (f == "rcCommand[3]") map.rc_command_idx[3] = local_idx;
            
            map.total_fields++;
        }
    }



    void attitudeCmdCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() < 4) return;
        
        float roll_rate = msg->data[0];
        float pitch_rate = msg->data[1];
        float yaw_rate = msg->data[2];
        float thrust = msg->data[3];

        // Update internal RC state
        rc_channels_[0] = std::max(1000, std::min(2000, (int)(1500 + (roll_rate / max_rate_deg_s_) * 500)));
        rc_channels_[1] = std::max(1000, std::min(2000, (int)(1500 + (pitch_rate / max_rate_deg_s_) * 500)));
        rc_channels_[3] = std::max(1000, std::min(2000, (int)(1500 + (yaw_rate / max_rate_deg_s_) * 500)));
        rc_channels_[2] = std::max(1000, std::min(2000, (int)(1000 + thrust * 1000)));
        
        // Mode logic: Keep Armed and in Override if receiving active commands
        rc_channels_[4] = 2000; // AUX1 (Armed)
        rc_channels_[5] = 1000; // AUX2 (MSP Override)
        
        last_cmd_time_ = this->now();
        sendMspPacket();
    }

    void safetyWatchdog() {
        auto now = this->now();
        if ((now - last_cmd_time_).seconds() > 0.5) {
            // Failsafe: Disarm and disable override
            rc_channels_[2] = 1000; // Throttle low
            rc_channels_[4] = 1000; // AUX1 (Disarmed)
            rc_channels_[5] = 1500; // AUX2 (Override Off)
            sendMspPacket();
        }
    }

    void sendMspPacket() {
        if (!port_) return; // ALLOW TX DURING LOCK TO PERMIT ARMING

        uint8_t frame[22];
        frame[0] = '$'; frame[1] = 'M'; frame[2] = '<';
        frame[3] = 16;   // Size
        frame[4] = 200;  // MSP_SET_RAW_RC
        
        uint8_t checksum = frame[3] ^ frame[4];
        for (int i = 0; i < 8; ++i) {
            uint16_t val = rc_channels_[i];
            frame[5 + i*2] = val & 0xFF;
            frame[5 + i*2 + 1] = (val >> 8) & 0xFF;
            checksum ^= frame[5 + i*2];
            checksum ^= frame[5 + i*2 + 1];
        }
        frame[21] = checksum;

        sp_nonblocking_write(port_, frame, 22);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BlackboxDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
