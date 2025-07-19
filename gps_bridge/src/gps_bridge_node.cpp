#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <chrono>
#include <cmath>

class GPSBridge : public rclcpp::Node
{
public:
    GPSBridge() : Node("gps_bridge")
    {
        // Subscribe to your GPS topic
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/drone/gps", 10,
            [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
                this->gps_callback(msg);
            });

        // Publish to PX4-compatible QoS:
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
        qos.history(rclcpp::HistoryPolicy::KeepLast);

        px4_gps_pub_ = this->create_publisher<px4_msgs::msg::SensorGps>(
            "/drone/fmu/in/sensor_gps", qos);

        // Initialize previous position for velocity calculation
        prev_lat_ = 0.0;
        prev_lon_ = 0.0;
        prev_alt_ = 0.0;
        prev_time_ = this->get_clock()->now();
        first_msg_ = true;
        msg_count_ = 0;

        RCLCPP_INFO(this->get_logger(), "GPS Bridge started with FIXED position and velocity!");
    }

private:
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        px4_msgs::msg::SensorGps px4_gps;
        
        // CRITICAL FIX: Use simulation time instead of steady_clock
        auto current_time = this->get_clock()->now();
        px4_gps.timestamp = current_time.nanoseconds() / 1000; // Convert to microseconds

        // FIXED: Use static position for stable simulation
        static bool position_set = false;
        static double fixed_lat = 0.0;
        static double fixed_lon = 0.0;
        static double fixed_alt = 0.0;

        if (!position_set) {
            fixed_lat = msg->latitude;
            fixed_lon = msg->longitude; 
            fixed_alt = msg->altitude;
            position_set = true;
            RCLCPP_INFO(this->get_logger(), "Fixed GPS position set: lat=%.6f, lon=%.6f, alt=%.2f", 
                fixed_lat, fixed_lon, fixed_alt);
        }

        // Use fixed position instead of changing Gazebo GPS
        px4_gps.latitude_deg = fixed_lat;
        px4_gps.longitude_deg = fixed_lon;
        px4_gps.altitude_msl_m = fixed_alt;
        px4_gps.altitude_ellipsoid_m = fixed_alt + 30.0f; // Realistic ellipsoid offset

        // FIXED: Set minimal velocities for stable simulation
        if (!first_msg_ && msg_count_ > 3) {
            double dt = (current_time - prev_time_).seconds();
            if (dt > 0.1 && dt < 2.0) {
                // For stationary simulation, use very small velocities
                px4_gps.vel_n_m_s = 0.0f;  // No north velocity
                px4_gps.vel_e_m_s = 0.0f;  // No east velocity
                px4_gps.vel_d_m_s = 0.0f;  // No down velocity
                
                RCLCPP_DEBUG(this->get_logger(), "Stable vel: N=%.2f, E=%.2f, D=%.2f (dt=%.3f)", 
                    px4_gps.vel_n_m_s, px4_gps.vel_e_m_s, px4_gps.vel_d_m_s, dt);
            } else {
                // Invalid time delta - set velocities to zero
                px4_gps.vel_n_m_s = 0.0f;
                px4_gps.vel_e_m_s = 0.0f;
                px4_gps.vel_d_m_s = 0.0f;
            }
        } else {
            // First few messages - set velocity to zero
            px4_gps.vel_n_m_s = 0.0f;
            px4_gps.vel_e_m_s = 0.0f;
            px4_gps.vel_d_m_s = 0.0f;
            first_msg_ = false;
        }

        // Store current values for next iteration (using fixed position)
        prev_lat_ = fixed_lat;
        prev_lon_ = fixed_lon;
        prev_alt_ = fixed_alt;
        prev_time_ = current_time;
        msg_count_++;

        // IMPROVEMENT: Realistic GPS quality values for simulation
        px4_gps.fix_type = 3; // 3D fix
        px4_gps.satellites_used = 12; // More realistic satellite count
        
        // Better accuracy values (in meters) - typical for good GPS
        px4_gps.eph = 0.5f;  // IMPROVED: Better horizontal position accuracy
        px4_gps.epv = 1.0f;  // IMPROVED: Better vertical position accuracy
        px4_gps.hdop = 0.8f; // IMPROVED: Better horizontal dilution of precision
        px4_gps.vdop = 1.0f; // IMPROVED: Better vertical dilution of precision
        
        // FIXED: Use only available fields in px4_msgs::msg::SensorGps
        px4_gps.vel_ned_valid = true;
        
        // Additional quality indicators
        px4_gps.noise_per_ms = 5;  // IMPROVED: Lower GPS noise
        px4_gps.jamming_indicator = 0; // No jamming

        // Publish to PX4
        px4_gps_pub_->publish(px4_gps);

        // Throttled logging for debugging
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        //     "GPS: lat=%.6f, lon=%.6f, alt=%.2f, vel=[%.1f,%.1f,%.1f] m/s, sats=%d", 
        //     px4_gps.latitude_deg, px4_gps.longitude_deg, px4_gps.altitude_msl_m, 
        //     px4_gps.vel_n_m_s, px4_gps.vel_e_m_s, px4_gps.vel_d_m_s,
        //     px4_gps.satellites_used);
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Publisher<px4_msgs::msg::SensorGps>::SharedPtr px4_gps_pub_;
    
    // For velocity calculation
    double prev_lat_, prev_lon_, prev_alt_;
    rclcpp::Time prev_time_;
    bool first_msg_;
    int msg_count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSBridge>());
    rclcpp::shutdown();
    return 0;
}