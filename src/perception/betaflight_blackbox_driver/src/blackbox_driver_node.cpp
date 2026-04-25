#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <cmath>
#include <thread>
#include <sstream>
#include <iomanip>
extern "C" {
#include <libserialport.h>
}
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <unistd.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <fstream>

using namespace std::chrono_literals;

class BlackboxDriverNode : public rclcpp::Node {
public:
    BlackboxDriverNode() : Node("betaflight_blackbox_driver") {
        this->declare_parameter("port", "/dev/ttyTHS1");
        this->declare_parameter("baud", 1500000);
        this->declare_parameter("imu_topic", "/imu0");
        this->declare_parameter("target_hz", 400);
        this->declare_parameter("intra_skip_fields", 13);
        this->declare_parameter("debug_raw", false);
        this->declare_parameter("enable_msp_control", false);
        this->declare_parameter("attitude_command_topic", "/drone_attitude_cmd");
        this->declare_parameter("motor_poles", 14);
        this->declare_parameter("rpm_topic", "/motor_rpm");

        std::string port_name = this->get_parameter("port").as_string();
        int baud = this->get_parameter("baud").as_int();
        std::string imu_topic = this->get_parameter("imu_topic").as_string();
        target_hz_ = this->get_parameter("target_hz").as_int();
        intra_skip_fields_ = this->get_parameter("intra_skip_fields").as_int();
        debug_raw_ = this->get_parameter("debug_raw").as_bool();
        bool enable_msp_control = this->get_parameter("enable_msp_control").as_bool();
        std::string attitude_command_topic = this->get_parameter("attitude_command_topic").as_string();
        motor_poles_ = this->get_parameter("motor_poles").as_int();
        std::string rpm_topic = this->get_parameter("rpm_topic").as_string();

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic, 10);
        rpm_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(rpm_topic, 10);
        
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
        RCLCPP_INFO(this->get_logger(), "Connected to Betaflight Blackbox on %s at %d baud (Debug: %s)", 
                    port_name.c_str(), baud, debug_raw_ ? "ON" : "OFF");
        
        if (debug_raw_) {
            raw_dump_file_ = fopen("/tmp/capture.bbl", "wb");
            RCLCPP_INFO(this->get_logger(), "Raw binary dumping enabled -> /tmp/capture.bbl");
        }
        buffer_.resize(CIRCULAR_SIZE);
        thread_ = std::thread(&BlackboxDriverNode::readLoop, this);
        
        // RPM Health Check Timer (every 10 seconds)
        health_check_timer_ = this->create_wall_timer(std::chrono::seconds(10), std::bind(&BlackboxDriverNode::rpmHealthCheck, this));
        
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
    };

    enum class ParseState { SYNC, HEADER, INTRA, INTER, MSP_V1, MSP_V2 };
    
    // Circular Buffer for high-rate parsing
    static constexpr size_t CIRCULAR_SIZE = 65536;
    std::vector<uint8_t> buffer_;
    size_t head_ = 0; // Write index
    size_t tail_ = 0; // Read index

    struct FieldMap {
        int time_idx = -1;
        int gyro_idx[3] = {-1, -1, -1};
        int accel_idx[3] = {-1, -1, -1};
        int motor_idx[4] = {-1, -1, -1, -1};
        int erpm_idx[4] = {-1, -1, -1, -1};
        int total_fields = 0;
    };

    ParseState state_ = ParseState::SYNC;
    FieldMap i_map_, p_map_;
    bool headers_locked_ = false;
    std::string header_line_buffer_;

    int target_hz_ = 400;
    int intra_skip_fields_ = 13;
    int downsample_factor_ = -1;
    bool debug_raw_ = false;
    std::chrono::steady_clock::time_point last_warning_time_;
    std::chrono::steady_clock::time_point first_data_time_;
    bool saw_first_data_ = false;

    FrameData last_frame_ = {0};
    bool has_first_intra_ = false;
    bool has_gyro_mapped_ = false;
    bool has_accel_mapped_ = false;
    bool time_synced_ = false;
    int64_t time_offset_us_ = 0;

    struct sp_port* port_ = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rpm_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr att_sub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::TimerBase::SharedPtr health_check_timer_;
    
    int motor_poles_ = 14;
    bool motor_poles_from_header_ = false;
    bool has_erpm_mapped_ = false;
    bool has_motor_mapped_ = false;
    bool erpm_valid_ = false;
    std::chrono::steady_clock::time_point last_nonzero_motor_time_;
    std::chrono::steady_clock::time_point last_erpm_check_time_;
    
    float max_rate_deg_s_ = 500.0;
    rclcpp::Time last_cmd_time_;
    uint16_t rc_channels_[8] = {1500, 1500, 1000, 1500, 1000, 1500, 1500, 1500}; // Default Safe State

    std::thread thread_;
    bool running_ = true;
    uint64_t total_bytes_read_ = 0;
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

    bool hasCompleteFrame(ParseState s) const {
        if (s == ParseState::INTRA || s == ParseState::INTER) {
            const std::vector<int>& encodings = (s == ParseState::INTRA) ? i_encoding_ : p_encoding_;
            if (encodings.empty()) return false;
            
            size_t current_peek = 0;
            for (int enc : encodings) {
                if (current_peek >= queueSize()) return false;
                
                if (enc == 10 || enc == 9) { // NULL or G-Zero
                    continue; 
                } else if (enc == 6) { // Tag2_3S32
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
                } else { // Variable Bit (VB) variants
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
            return true;
        } else if (s == ParseState::MSP_V1) {
            // Header: $M[Direction][Size][Cmd]...[Checksum]
            // We've already popped '$M', sopeek(0)=Dir, peek(1)=Size
            if (queueSize() < 2) return false; 
            uint8_t size = peekByte(1);
            return queueSize() >= (size_t)(size + 3); // Dir + Size + Cmd + Payload + Checksum
        } else if (s == ParseState::MSP_V2) {
            // Header: $X[Direction][Flags][CmdLo][CmdHi][SizeLo][SizeHi]...[CRC]
            if (queueSize() < 6) return false;
            uint16_t size = peekByte(4) | (peekByte(5) << 8);
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
        return (int32_t)((u >> 1) ^ -(int32_t)(u & 1));
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

    // Parse encoding headers: "H Field I encoding:1,1,3,0,..." → vector of ints
    std::vector<int> i_encoding_, p_encoding_;

    void readLoop() {
        std::vector<uint8_t> raw_buf(4096);
        while (running_ && rclcpp::ok()) {
            int bytes = sp_blocking_read(port_, raw_buf.data(), raw_buf.size(), 1);
            if (bytes > 0) {
                if (raw_dump_file_) { fwrite(raw_buf.data(), 1, bytes, raw_dump_file_); fflush(raw_dump_file_); }
                total_bytes_read_ += bytes;
                for (int i = 0; i < bytes; ++i) {
                    size_t next_head = (head_ + 1) % CIRCULAR_SIZE;
                    if (next_head == tail_) break; // Buffer full
                    buffer_[head_] = raw_buf[i];
                    head_ = next_head;
                }
                processData();
            }
        }
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

                if (!hasCompleteFrame(state_)) break; 

                if (state_ == ParseState::INTRA) {
                    try {
                        // Decode ALL fields using per-field encoding from I-frame encoding header
                        int total_fields = (int)i_encoding_.size();
                        int32_t frame[64] = {};
                        int fi = 0;
                        while (fi < total_fields && fi < 64) {
                            int enc = i_encoding_[fi];
                            if (enc == 1) { // Unsigned VB
                                frame[fi] = (int32_t)decodeVBUI();
                                fi++;
                            } else if (enc == 0) { // Signed VB
                                frame[fi] = decodeVBSI();
                                fi++;
                            } else if (enc == 3) { // Tag8_8SVB (for single field: same as signed VB)
                                frame[fi] = decodeVBSI();
                                fi++;
                            } else if (enc == 10) { // NULL
                                frame[fi] = 0;
                                fi++;
                            } else { // Unknown encoding, try signed VB
                                frame[fi] = decodeVBSI();
                                fi++;
                            }
                        }
                        // Map fields: indices are relative to start_idx=2 (skip loopIteration, time)
                        int start = 2; // loopIteration=0, time=1
                        last_frame_.time = (uint32_t)frame[1];
                        if (has_gyro_mapped_ || has_accel_mapped_) {
                            if (has_gyro_mapped_) {
                                last_frame_.gyro[0] = frame[start + i_map_.gyro_idx[0]];
                                last_frame_.gyro[1] = frame[start + i_map_.gyro_idx[1]];
                                last_frame_.gyro[2] = frame[start + i_map_.gyro_idx[2]];
                            }
                            if (has_accel_mapped_) {
                                last_frame_.accel[0] = frame[start + i_map_.accel_idx[0]];
                                last_frame_.accel[1] = frame[start + i_map_.accel_idx[1]];
                                last_frame_.accel[2] = frame[start + i_map_.accel_idx[2]];
                            }
                            if (has_erpm_mapped_) {
                                for (int m = 0; m < 4; m++) {
                                    if (i_map_.erpm_idx[m] >= 0) last_frame_.erpm[m] = frame[start + i_map_.erpm_idx[m]];
                                }
                                erpm_valid_ = true;
                            }
                            if (has_motor_mapped_) {
                                for (int m = 0; m < 4; m++) {
                                    if (i_map_.motor_idx[m] >= 0) last_frame_.motor[m] = frame[start + i_map_.motor_idx[m]];
                                }
                            }
                            has_first_intra_ = true;
                            publishIMU();
                            publishRPM();
                        }
                    } catch (...) {}
                    state_ = ParseState::SYNC;
                } else if (state_ == ParseState::INTER) {
                    if (!has_first_intra_) { state_ = ParseState::SYNC; continue; }
                    try {
                        // Decode ALL P-frame fields using per-field encoding
                        int total_fields = (int)p_encoding_.size();
                        int32_t frame[64] = {};
                        int fi = 0;
                        while (fi < total_fields && fi < 64) {
                            int enc = p_encoding_[fi];
                            if (enc == 10 || enc == 9) { // NULL encoding — no bytes
                                frame[fi] = 0;
                                fi++;
                            } else if (enc == 0) { // Signed VB
                                frame[fi] = decodeVBSI();
                                fi++;
                            } else if (enc == 1) { // Unsigned VB
                                frame[fi] = (int32_t)decodeVBUI();
                                fi++;
                            } else if (enc == 6) { // Tag2_3S32 — decodes 3 fields at once
                                int32_t vals[3];
                                decodeTag2_3S32(vals);
                                frame[fi]   = vals[0];
                                frame[fi+1] = (fi+1 < 64) ? vals[1] : 0;
                                frame[fi+2] = (fi+2 < 64) ? vals[2] : 0;
                                fi += 3;
                            } else { // Unknown, try signed VB
                                frame[fi] = decodeVBSI();
                                fi++;
                            }
                        }
                        // Apply deltas: P-frame values are deltas from previous
                        int start = 2;
                        last_frame_.time += (uint32_t)frame[1];
                        if (has_gyro_mapped_) {
                            last_frame_.gyro[0] += frame[start + p_map_.gyro_idx[0]];
                            last_frame_.gyro[1] += frame[start + p_map_.gyro_idx[1]];
                            last_frame_.gyro[2] += frame[start + p_map_.gyro_idx[2]];
                        }
                        if (has_accel_mapped_) {
                            last_frame_.accel[0] += frame[start + p_map_.accel_idx[0]];
                            last_frame_.accel[1] += frame[start + p_map_.accel_idx[1]];
                            last_frame_.accel[2] += frame[start + p_map_.accel_idx[2]];
                        }
                        if (has_erpm_mapped_ && erpm_valid_) {
                            for (int m = 0; m < 4; m++) {
                                if (p_map_.erpm_idx[m] >= 0) last_frame_.erpm[m] += frame[start + p_map_.erpm_idx[m]];
                            }
                        }
                        if (has_motor_mapped_) {
                            for (int m = 0; m < 4; m++) {
                                if (p_map_.motor_idx[m] >= 0) last_frame_.motor[m] += frame[start + p_map_.motor_idx[m]];
                            }
                        }
                        publishIMU();
                        publishRPM();
                    } catch (...) {}
                    state_ = ParseState::SYNC;
                } else if (state_ == ParseState::MSP_V1) {
                    uint8_t size = peekByte(1); // peeByte(0) was Direction
                    popBytes(size + 3); // Consume Direction + Size + Cmd + Payload + Checksum
                    state_ = ParseState::SYNC;
                } else if (state_ == ParseState::MSP_V2) {
                    uint16_t size = peekByte(4) | (peekByte(5) << 8);
                    popBytes(size + 7); // Consume Direction + Flags + Cmd(2) + Size(2) + Payload + CRC
                    state_ = ParseState::SYNC;
                }
            }
        }
    }

    void publishIMU() {
        static int total_frames = 0;
        if (++total_frames % 5000 == 0) {
            RCLCPP_INFO(this->get_logger(), "Parsed 5000 frames. Last Accel Z: %d", last_frame_.accel[2]);
        }

        if (downsample_factor_ <= 0) {
            static auto start_detect = std::chrono::steady_clock::now();
            static int raw_detect_count = 0;
            if (raw_detect_count == 0) start_detect = std::chrono::steady_clock::now();
            raw_detect_count++;
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - start_detect).count();
            if (elapsed >= 1.0) {
                int est_hz = raw_detect_count / elapsed;
                downsample_factor_ = std::max(1, est_hz / target_hz_);
                RCLCPP_INFO(this->get_logger(), "Detected IMU at %d Hz. Downsampling by %d to reach %d Hz", 
                            est_hz, downsample_factor_, target_hz_);
            }
            return;
        }

        static int samples_count = 0;
        if (++samples_count >= downsample_factor_) {
            auto msg = sensor_msgs::msg::Imu();
            
            // Hardware Timestamping Logic
            int64_t current_ros_us = this->now().nanoseconds() / 1000;
            int64_t current_fc_us = last_frame_.time;

            if (!time_synced_) {
                time_offset_us_ = current_ros_us - current_fc_us;
                time_synced_ = true;
                RCLCPP_INFO(this->get_logger(), "Time synced! Offset: %ld us", time_offset_us_);
            }

            int64_t true_capture_time_us = current_fc_us + time_offset_us_;
            msg.header.stamp = rclcpp::Time(true_capture_time_us * 1000); // Back to nanoseconds
            msg.header.frame_id = "imu_link";
            
            const float G_TO_MS2 = 9.80665f;
            const float ACCEL_SCALE = 1.0f / 2048.0f;  // acc_1G=2048 confirmed from header
            const float GYRO_SCALE = M_PI / 180.0f;    // gyroADC is already deg/s, only need deg→rad

            msg.linear_acceleration.x = last_frame_.accel[0] * ACCEL_SCALE * G_TO_MS2;
            msg.linear_acceleration.y = last_frame_.accel[1] * ACCEL_SCALE * G_TO_MS2;
            msg.linear_acceleration.z = last_frame_.accel[2] * ACCEL_SCALE * G_TO_MS2;
            
            msg.angular_velocity.x = last_frame_.gyro[0] * GYRO_SCALE;
            msg.angular_velocity.y = last_frame_.gyro[1] * GYRO_SCALE;
            msg.angular_velocity.z = last_frame_.gyro[2] * GYRO_SCALE;
            
            imu_pub_->publish(msg);
            samples_count = 0;
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

    void rpmHealthCheck() {
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

    void parseHeaderLine(const std::string& line) {
        if (line.find("Field I name:") != std::string::npos) {
            mapFields(line, i_map_);
            RCLCPP_INFO(this->get_logger(), "Parsed Field I name (%d fields)", i_map_.total_fields);
        }
        else if (line.find("Field I encoding:") != std::string::npos) {
            i_encoding_ = parseEncodingHeader(line);
        }
        else if (line.find("Field P name:") != std::string::npos) {
            mapFields(line, p_map_);
            RCLCPP_INFO(this->get_logger(), "Parsed Field P name (%d fields)", p_map_.total_fields);
        }
        else if (line.find("Field P encoding:") != std::string::npos) {
            p_encoding_ = parseEncodingHeader(line);
        }
        
        // Finalize mapping state
        if (i_map_.gyro_idx[0] != -1 && i_map_.accel_idx[0] != -1) {
            has_gyro_mapped_ = true;
            has_accel_mapped_ = true;
        }
        if (i_map_.erpm_idx[0] != -1) {
            has_erpm_mapped_ = true;
            RCLCPP_INFO(this->get_logger(), "eRPM fields mapped (indices: %d,%d,%d,%d)",
                        i_map_.erpm_idx[0], i_map_.erpm_idx[1], i_map_.erpm_idx[2], i_map_.erpm_idx[3]);
        }
        if (i_map_.motor_idx[0] != -1) {
            has_motor_mapped_ = true;
        }

        if (i_map_.total_fields > 0 && !headers_locked_) {
            // BF 4.5+ doesn't send "Field P name:" — P frames use the same field layout
            if (p_map_.total_fields == 0) p_map_ = i_map_;
            headers_locked_ = true;
            RCLCPP_INFO(this->get_logger(), "Dictionary locked (I:%d fields, P:%d fields)",
                        i_map_.total_fields, p_map_.total_fields);
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
        std::string names = line.substr(pos + 1);
        std::vector<std::string> fields;
        std::stringstream ss(names);
        std::string field;
        while (std::getline(ss, field, ',')) fields.push_back(field);

        map.total_fields = 0;
        int start_idx = (line.find("Field I") != std::string::npos) ? 2 : 1;

        for (int i = start_idx; i < (int)fields.size(); ++i) {
            std::string f = fields[i];
            // Poor man's trim (removes leading/trailing spaces/newlines)
            f.erase(0, f.find_first_not_of(" \n\r\t"));
            f.erase(f.find_last_not_of(" \n\r\t") + 1);

            int local_idx = i - start_idx;
            if (f == "gyroADC[0]" || f == "gyroRaw[0]" || f == "gyroUnfilt[0]" || f == "gi[0]") map.gyro_idx[0] = local_idx;
            else if (f == "gyroADC[1]" || f == "gyroRaw[1]" || f == "gyroUnfilt[1]" || f == "gi[1]") map.gyro_idx[1] = local_idx;
            else if (f == "gyroADC[2]" || f == "gyroRaw[2]" || f == "gyroUnfilt[2]" || f == "gi[2]") map.gyro_idx[2] = local_idx;
            else if (f == "accelADC[0]" || f == "accSmooth[0]" || f == "accelRaw[0]" || f == "ai[0]") map.accel_idx[0] = local_idx;
            else if (f == "accelADC[1]" || f == "accSmooth[1]" || f == "accelRaw[1]" || f == "ai[1]") map.accel_idx[1] = local_idx;
            else if (f == "accelADC[2]" || f == "accSmooth[2]" || f == "accelRaw[2]" || f == "ai[2]") map.accel_idx[2] = local_idx;
            else if (f == "motor[0]") map.motor_idx[0] = local_idx;
            else if (f == "motor[1]") map.motor_idx[1] = local_idx;
            else if (f == "motor[2]") map.motor_idx[2] = local_idx;
            else if (f == "motor[3]") map.motor_idx[3] = local_idx;
            else if (f == "eRPM[0]")  map.erpm_idx[0] = local_idx;
            else if (f == "eRPM[1]")  map.erpm_idx[1] = local_idx;
            else if (f == "eRPM[2]")  map.erpm_idx[2] = local_idx;
            else if (f == "eRPM[3]")  map.erpm_idx[3] = local_idx;
            
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
