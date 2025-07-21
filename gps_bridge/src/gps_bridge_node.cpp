#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <chrono>
#include <cmath>
#include <deque>

const double EARTH_RADIUS = 6371000.0; 

class GPSBridge : public rclcpp::Node
{
public:
    GPSBridge() : Node("gps_bridge")
    {
        this->declare_parameter("gps_device_id", 77777); 
        device_id_ = this->get_parameter("gps_device_id").as_int();
        
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/drone/gps", 10,
            [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
                this->gps_callback(msg);
            });

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos.durability(rclcpp::DurabilityPolicy::Volatile);
        qos.history(rclcpp::HistoryPolicy::KeepLast);

        px4_gps_pub_ = this->create_publisher<px4_msgs::msg::SensorGps>(
            "/drone/fmu/in/sensor_gps", qos);

        prev_time_ = this->get_clock()->now();
        first_msg_ = true;
        
        RCLCPP_INFO(this->get_logger(), "GPS Bridge Initialized - Final Version. Device ID: %d", device_id_);
    }

private:
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        px4_msgs::msg::SensorGps px4_gps;
        
        // ======================== CRITICAL FIX HERE ========================
        // The timestamp MUST be 0. The uXRCE-DDS client inside PX4 will
        // automatically timestamp the message upon receipt, ensuring proper
        // synchronization with the EKF2.
        px4_gps.timestamp = 0;
        
        // Use the unique device ID
        px4_gps.device_id = device_id_;
        
        // Position data
        px4_gps.latitude_deg = msg->latitude;
        px4_gps.longitude_deg = msg->longitude;
        px4_gps.altitude_msl_m = msg->altitude;
        px4_gps.altitude_ellipsoid_m = msg->altitude;

        // Calculate velocities using the node's clock for dt calculation
        calculate_velocities(msg, px4_gps, this->get_clock()->now());
        
        // Fixed GPS quality parameters
        px4_gps.fix_type = 3;
        px4_gps.satellites_used = 12;
        px4_gps.eph = 1.0f;
        px4_gps.epv = 1.5f;
        px4_gps.hdop = 0.8f;
        px4_gps.vdop = 1.0f;
        px4_gps.s_variance_m_s = 0.2f;
        px4_gps.c_variance_rad = 0.1f;
        px4_gps.vel_ned_valid = true;
        
        px4_gps.heading = NAN;
        
        px4_gps_pub_->publish(px4_gps);
        
        prev_lat_ = msg->latitude;
        prev_lon_ = msg->longitude;
        prev_alt_ = msg->altitude;
    }
    
    void calculate_velocities(const sensor_msgs::msg::NavSatFix::SharedPtr msg,
                              px4_msgs::msg::SensorGps& px4_gps,
                              const rclcpp::Time& current_time)
    {
        if (!first_msg_) {
            double dt = (current_time - prev_time_).seconds();
            if (dt > 0.005 && dt < 1.0) {
                double lat_rad = msg->latitude * M_PI / 180.0;
                double delta_lat_m = (msg->latitude - prev_lat_) * M_PI / 180.0 * EARTH_RADIUS;
                double delta_lon_m = (msg->longitude - prev_lon_) * M_PI / 180.0 * EARTH_RADIUS * cos(lat_rad);
                double delta_alt_m = msg->altitude - prev_alt_;
                px4_gps.vel_n_m_s = delta_lat_m / dt;
                px4_gps.vel_e_m_s = delta_lon_m / dt;
                px4_gps.vel_d_m_s = -delta_alt_m / dt;
            }
        }
        first_msg_ = false;
        prev_time_ = current_time;
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Publisher<px4_msgs::msg::SensorGps>::SharedPtr px4_gps_pub_;
    double prev_lat_, prev_lon_, prev_alt_;
    rclcpp::Time prev_time_;
    bool first_msg_;
    int device_id_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSBridge>());
    rclcpp::shutdown();
    return 0;
}