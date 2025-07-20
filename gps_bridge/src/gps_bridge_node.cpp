#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "px4_msgs/msg/sensor_gps.hpp"

using namespace std::chrono;

class GpsBridgeNode : public rclcpp::Node
{
public:
    GpsBridgeNode()
        : Node("gps_bridge"),
          first_msg_(true),
          last_velocity_time_(0),
          velocity_filter_size_(5),
          gps_device_id_(1),
          debug_logging_(false)
    {
        this->declare_parameter("velocity_filter_size", 5);
        this->declare_parameter("gps_device_id", 1);
        this->declare_parameter("debug_logging", false);

        this->get_parameter("velocity_filter_size", velocity_filter_size_);
        this->get_parameter("gps_device_id", gps_device_id_);
        this->get_parameter("debug_logging", debug_logging_);

        gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/drone/gps",
            rclcpp::SensorDataQoS(),
            std::bind(&GpsBridgeNode::gps_callback, this, std::placeholders::_1));

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                       .best_effort()
                       .durability_volatile(); // Recommended for XRCE-DDS

        px4_gps_pub_ = this->create_publisher<px4_msgs::msg::SensorGps>(
            "/drone/fmu/in/sensor_gps", qos);

        RCLCPP_INFO(this->get_logger(), "GPS Bridge Node initialized.");
    }

private:
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        const uint64_t time_us = duration_cast<microseconds>(
                                     this->now().time_since_epoch())
                                     .count();

        if (first_msg_) {
            first_msg_ = false;
        }

        double vel_n_mps = 0.0;
        double vel_e_mps = 0.0;
        double vel_d_mps = 0.0;

        if (!last_msg_time_.nanoseconds() == 0) {
            double dt = (this->now() - last_msg_time_).seconds();
            if (dt > 0.0) {
                vel_n_mps = (msg->latitude - last_lat_) * 1e5 / dt;
                vel_e_mps = (msg->longitude - last_lon_) * 1e5 / dt;
                vel_d_mps = (msg->altitude - last_alt_) / dt;

                velocity_buffer_n_.push_back(vel_n_mps);
                velocity_buffer_e_.push_back(vel_e_mps);
                velocity_buffer_d_.push_back(vel_d_mps);

                while (velocity_buffer_n_.size() > velocity_filter_size_) {
                    velocity_buffer_n_.pop_front();
                    velocity_buffer_e_.pop_front();
                    velocity_buffer_d_.pop_front();
                }

                vel_n_mps = mean(velocity_buffer_n_);
                vel_e_mps = mean(velocity_buffer_e_);
                vel_d_mps = mean(velocity_buffer_d_);
            }
        }

        px4_msgs::msg::SensorGps px4_gps;
        px4_gps.timestamp = time_us;
        px4_gps.device_id = gps_device_id_;
        px4_gps.lat = static_cast<int32_t>(msg->latitude * 1e7);
        px4_gps.lon = static_cast<int32_t>(msg->longitude * 1e7);
        px4_gps.alt = static_cast<int32_t>(msg->altitude * 1e3);

        px4_gps.timestamp_time_relative = 0;

        px4_gps.fix_type = 3; // 3D fix
        px4_gps.satellites_used = 10;

        // Velocity in cm/s
        px4_gps.vel_m_s = static_cast<float>(std::sqrt(vel_n_mps * vel_n_mps + vel_e_mps * vel_e_mps));
        px4_gps.vel_n_m_s = static_cast<float>(vel_n_mps);
        px4_gps.vel_e_m_s = static_cast<float>(vel_e_mps);
        px4_gps.vel_d_m_s = static_cast<float>(-vel_d_mps); // NED: z is down

        // Yaw not available from GPS, leave at 0
        px4_gps.cog_rad = 0.0;
        px4_gps.heading = UINT16_MAX;
        px4_gps.heading_offset = 0.0;

        if (msg->position_covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
            px4_gps.eph = 0.5f;
            px4_gps.epv = 1.0f;
        } else {
            px4_gps.eph = std::sqrt(msg->position_covariance[0]); // east std dev
            px4_gps.epv = std::sqrt(msg->position_covariance[8]); // vertical std dev
        }

        px4_gps.noise_per_ms = 0;
        px4_gps.jamming_indicator = 0;
        px4_gps.automatic_gain_control = 0;
        px4_gps.jamming_state = 0;
        px4_gps.gps_noise = 0;
        px4_gps.gps_jamming = 0;

        px4_gps_pub_->publish(px4_gps);

        if (debug_logging_) {
            RCLCPP_INFO(this->get_logger(), "Published PX4 GPS msg at time_us=%lu", time_us);
        }

        last_lat_ = msg->latitude;
        last_lon_ = msg->longitude;
        last_alt_ = msg->altitude;
        last_msg_time_ = this->now();
    }

    double mean(const std::deque<double> &buffer)
    {
        if (buffer.empty())
            return 0.0;
        double sum = 0.0;
        for (auto val : buffer)
            sum += val;
        return sum / buffer.size();
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
    rclcpp::Publisher<px4_msgs::msg::SensorGps>::SharedPtr px4_gps_pub_;

    std::deque<double> velocity_buffer_n_;
    std::deque<double> velocity_buffer_e_;
    std::deque<double> velocity_buffer_d_;
    int velocity_filter_size_;
    bool first_msg_;
    int gps_device_id_;
    bool debug_logging_;

    rclcpp::Time last_msg_time_;
    double last_lat_;
    double last_lon_;
    double last_alt_;
    uint64_t last_velocity_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsBridgeNode>());
    rclcpp::shutdown();
    return 0;
}