#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <msp/FlightController.hpp>
#include <msp/msp_msg.hpp>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <limits>

class MspImuDriverNode : public rclcpp::Node {
public:
    MspImuDriverNode() : Node("msp_imu_driver_node") {
        this->declare_parameter("port", "/dev/ttyTHS1");
        this->declare_parameter("baud", 115200);
        this->declare_parameter("imu_topic", "/imu0");
        this->declare_parameter("target_hz", 150);
        this->declare_parameter("rpm_topic", "/motor_rpm");
        this->declare_parameter("battery_topic", "/battery");

        std::string port = this->get_parameter("port").as_string();
        int baud = this->get_parameter("baud").as_int();
        imu_topic_ = this->get_parameter("imu_topic").as_string();
        target_hz_ = this->get_parameter("target_hz").as_int();
        std::string rpm_topic = this->get_parameter("rpm_topic").as_string();
        std::string battery_topic = this->get_parameter("battery_topic").as_string();

        auto qos_profile = rclcpp::SensorDataQoS();
        qos_profile.keep_last(1);

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, qos_profile);
        rpm_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(rpm_topic, qos_profile);
        battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>(battery_topic, qos_profile);

        RCLCPP_INFO(this->get_logger(), "Connecting to Flight Controller on %s at %d baud...", port.c_str(), baud);

        fcu_ = std::make_unique<fcu::FlightController>();
        if (!fcu_->connect(port, baud)) {
            RCLCPP_FATAL(this->get_logger(), "Failed to connect to Flight Controller on %s!", port.c_str());
            throw std::runtime_error("FC Connection Failed");
        }

        RCLCPP_INFO(this->get_logger(), "Connected! Subscribing to telemetry (IMU target: %d Hz)...", target_hz_);

        // Subscriptions via lambda callbacks
        fcu_->subscribe<msp::msg::RawImu>([this](const msp::msg::RawImu &imu) {
            this->onImu(imu);
        }, 1.0 / static_cast<double>(target_hz_));

        fcu_->subscribe<msp::msg::Analog>([this](const msp::msg::Analog &analog) {
            this->onAnalog(analog);
        }, 0.1); // Battery at 10Hz

        fcu_->subscribe<msp::msg::EscSensorData>([this](const msp::msg::EscSensorData &esc) {
            this->onEsc(esc);
        }, 0.05); // ESC / RPM at 20Hz
    }

    ~MspImuDriverNode() {
        if (fcu_) {
            fcu_->disconnect();
        }
    }

private:
    void onImu(const msp::msg::RawImu &imu) {
        auto msg = sensor_msgs::msg::Imu();
        msg.header.stamp = this->now();
        msg.header.frame_id = "imu_link";

        // MultiWii/Betaflight MSP scaling: 
        // 1G acceleration maps to 512 units
        // Gyro is scaled by 1/4.096 to get deg/s
        const float acc_scale = 9.80665f / 512.0f;
        const float gyro_scale = (static_cast<float>(M_PI) / 180.0f) / 4.096f;

        msg.linear_acceleration.x = static_cast<float>(imu.acc[0]()) * acc_scale;
        msg.linear_acceleration.y = static_cast<float>(imu.acc[1]()) * acc_scale;
        msg.linear_acceleration.z = static_cast<float>(imu.acc[2]()) * acc_scale;

        msg.angular_velocity.x = static_cast<float>(imu.gyro[0]()) * gyro_scale;
        msg.angular_velocity.y = static_cast<float>(imu.gyro[1]()) * gyro_scale;
        msg.angular_velocity.z = static_cast<float>(imu.gyro[2]()) * gyro_scale;

        imu_pub_->publish(msg);
    }

    void onAnalog(const msp::msg::Analog &analog) {
        auto msg = sensor_msgs::msg::BatteryState();
        msg.header.stamp = this->now();
        msg.header.frame_id = "imu_link";
        msg.voltage = analog.vbat();
        msg.current = analog.amperage();
        msg.charge = std::numeric_limits<float>::quiet_NaN();
        msg.capacity = std::numeric_limits<float>::quiet_NaN();
        msg.design_capacity = std::numeric_limits<float>::quiet_NaN();
        msg.percentage = std::numeric_limits<float>::quiet_NaN();
        msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
        msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
        msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
        msg.present = true;

        battery_pub_->publish(msg);
    }

    void onEsc(const msp::msg::EscSensorData &esc) {
        if (esc.motor_count() == 0) return;

        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data.resize(esc.motor_count());

        for (size_t i = 0; i < esc.motor_count(); i++) {
            msg.data[i] = static_cast<double>(esc.esc_data[i].rpm());
        }

        rpm_pub_->publish(msg);
    }

    std::unique_ptr<fcu::FlightController> fcu_;
    std::string imu_topic_;
    int target_hz_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rpm_pub_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MspImuDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
