#include <rclcpp/rclcpp.hpp>
#include <mavsdk/mavsdk.h>
#include <mavsdk/system.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <thread>
#include <chrono>

using namespace mavsdk;
using namespace std::chrono_literals;

// For older MAVSDK headers
static std::string connection_result_to_str(ConnectionResult result) {
    switch (result) {
        case ConnectionResult::Success: return "Success";
        case ConnectionResult::Timeout: return "Timeout";
        case ConnectionResult::SocketError: return "SocketError";
        case ConnectionResult::BindError: return "BindError";
        case ConnectionResult::SocketConnectionError: return "SocketConnectionError";
        case ConnectionResult::ConnectionsExhausted: return "ConnectionsExhausted";
        case ConnectionResult::NotImplemented: return "NotImplemented";
        default: return "Unknown";
    }
}

class DroneCommNode : public rclcpp::Node {
public:
    DroneCommNode() :
        Node("mavsdk_node"),
        mavsdk{Mavsdk::Configuration(ComponentType::GroundStation)} 
    {
        RCLCPP_INFO(this->get_logger(), "Starting MAVSDK connection...");

        // ConnectionResult connection_result = mavsdk.add_any_connection("udpin://:14540");
        ConnectionResult connection_result = mavsdk.add_any_connection("udp://0.0.0.0:14540");
        if (connection_result != ConnectionResult::Success) {
            RCLCPP_ERROR(this->get_logger(), "Connection failed: %s",
                         connection_result_to_str(connection_result).c_str());
            return;
        }

        // Wait for PX4 system to connect
        std::this_thread::sleep_for(2s);
        auto systems = mavsdk.systems();
        if (systems.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No system found!");
            return;
        }

        system = systems.at(0);
        RCLCPP_INFO(this->get_logger(), "System connected");

        // Initialize plugins
        telemetry = std::make_shared<Telemetry>(system);
        action = std::make_shared<Action>(system);
        offboard = std::make_shared<Offboard>(system);

        // Monitor position continuously
        telemetry->subscribe_position([this](Telemetry::Position pos) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Lat: %.6f | Lon: %.6f | Alt: %.2f",
                                 pos.latitude_deg, pos.longitude_deg, pos.relative_altitude_m);
        });

        // Launch the flight sequence in a background thread
        control_thread_ = std::thread(&DroneCommNode::flight_sequence, this);
    }

    ~DroneCommNode() override {
        if (control_thread_.joinable())
            control_thread_.join();
    }

private:
    Mavsdk mavsdk;
    std::shared_ptr<System> system;
    std::shared_ptr<Telemetry> telemetry;
    std::shared_ptr<Action> action;
    std::shared_ptr<Offboard> offboard;
    std::thread control_thread_;

    void flight_sequence() {
        // Wait until the vehicle reports healthy sensors
        while (!telemetry->health_all_ok()) {
            RCLCPP_INFO(this->get_logger(), "Waiting for vehicle to be ready...");
            std::this_thread::sleep_for(1s);
        }
        RCLCPP_INFO(this->get_logger(), "Vehicle ready");

        // Arm
        if (action->arm() != Action::Result::Success) {
            RCLCPP_ERROR(this->get_logger(), "Arming failed");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Armed");

        // Takeoff
        if (action->takeoff() != Action::Result::Success) {
            RCLCPP_ERROR(this->get_logger(), "Takeoff failed");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Taking off...");
        std::this_thread::sleep_for(5s);

        // Start Offboard
        Offboard::VelocityNedYaw hover{};
        hover.north_m_s = 0.0f;
        hover.east_m_s = 0.0f;
        hover.down_m_s = 0.0f;
        hover.yaw_deg = 0.0f;
        offboard->set_velocity_ned(hover);

        // Send some setpoints before starting offboard mode
        RCLCPP_INFO(this->get_logger(), "Priming Offboard with setpoints...");
        for (int i = 0; i < 20; ++i) {
            offboard->set_velocity_ned(hover);
            std::this_thread::sleep_for(100ms);
        }

        auto result = offboard->start();
        if (result != Offboard::Result::Success) {
            RCLCPP_ERROR(this->get_logger(), "Offboard start failed (%d)", (int)result);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Offboard started");

        // Move north 2m/s for 5 seconds
        Offboard::VelocityNedYaw move{};
        move.north_m_s = 0.5f;
        move.east_m_s = 0.0f;
        move.down_m_s = 0.0f;
        move.yaw_deg = 0.0f;

        RCLCPP_INFO(this->get_logger(), "Moving north for 5s...");
        for (int i = 0; i < 50; ++i) {
            offboard->set_velocity_ned(move);
            std::this_thread::sleep_for(100ms);
        }

        // Hover for a few seconds
        RCLCPP_INFO(this->get_logger(), "Hovering...");
        for (int i = 0; i < 50; ++i) {
            offboard->set_velocity_ned(hover);
            std::this_thread::sleep_for(100ms);
        }

        // Stop offboard and land
        RCLCPP_INFO(this->get_logger(), "Landing...");
        offboard->stop();
        action->land();
        std::this_thread::sleep_for(5s);

        RCLCPP_INFO(this->get_logger(), "Mission complete");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DroneCommNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
