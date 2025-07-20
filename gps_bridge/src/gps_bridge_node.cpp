#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <chrono>
#include <cmath>
#include <deque>

class GPSBridge : public rclcpp::Node
{
public:
    GPSBridge() : Node("gps_bridge")
    {
        // Declare parameters
        this->declare_parameter("gps_device_id", 1310988);
        this->declare_parameter("velocity_filter_size", 5);
        this->declare_parameter("debug_logging", true);
        
        device_id_ = this->get_parameter("gps_device_id").as_int();
        velocity_filter_size_ = this->get_parameter("velocity_filter_size").as_int();
        debug_logging_ = this->get_parameter("debug_logging").as_bool();
        
        // Subscribe to Gazebo GPS topic
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/drone/gps", 10,
            [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
                this->gps_callback(msg);
            });

        // Publish to uXRCE-DDS topic
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos.durability(rclcpp::DurabilityPolicy::Volatile);
        qos.history(rclcpp::HistoryPolicy::KeepLast);

        px4_gps_pub_ = this->create_publisher<px4_msgs::msg::SensorGps>(
            "/drone/fmu/in/sensor_gps", qos);

        // Initialize state
        prev_lat_ = 0.0;
        prev_lon_ = 0.0;
        prev_alt_ = 0.0;
        prev_time_ = this->get_clock()->now();
        first_msg_ = true;
        msg_count_ = 0;
        
        RCLCPP_INFO(this->get_logger(), "GPS Bridge initialized - Device ID: %d", device_id_);
    }

private:
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        // ALWAYS print when GPS received - for debugging
        if (debug_logging_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "GPS RECEIVED: lat=%.6f, lon=%.6f, alt=%.2f", 
                msg->latitude, msg->longitude, msg->altitude);
        }
        
        px4_msgs::msg::SensorGps px4_gps;
        
        // Simple timestamp
        auto current_time = this->get_clock()->now();
        uint64_t time_us = current_time.nanoseconds() / 1000;
        px4_gps.timestamp = time_us;
        
        // Device ID
        px4_gps.device_id = device_id_;
        
        // Position data - NO VALIDATION, just copy
        px4_gps.latitude_deg = msg->latitude;
        px4_gps.longitude_deg = msg->longitude;
        px4_gps.altitude_msl_m = msg->altitude;
        px4_gps.altitude_ellipsoid_m = msg->altitude;
        
        // Calculate simple velocities
        calculate_velocities(msg, px4_gps, current_time);
        
        // Fixed GPS quality parameters
        px4_gps.fix_type = 3;                    // 3D fix
        px4_gps.satellites_used = 12;            // Good satellite count
        px4_gps.eph = 2.0f;                     // 2m horizontal accuracy
        px4_gps.epv = 3.0f;                     // 3m vertical accuracy
        px4_gps.hdop = 1.0f;                    // Good HDOP
        px4_gps.vdop = 1.2f;                    // Good VDOP
        px4_gps.s_variance_m_s = 0.5f;          // Speed accuracy
        px4_gps.c_variance_rad = 0.3f;          // Course accuracy
        px4_gps.vel_ned_valid = true;           // Always enable velocity
        px4_gps.noise_per_ms = 10;              // Low noise
        px4_gps.jamming_indicator = 0;          // No jamming
        px4_gps.spoofing_state = px4_msgs::msg::SensorGps::SPOOFING_STATE_NONE;
        
        // No GPS heading
        px4_gps.heading = NAN;
        px4_gps.heading_offset = NAN;
        px4_gps.heading_accuracy = NAN;
        
        // Simple time fields
        px4_gps.time_utc_usec = 0;
        px4_gps.timestamp_time_relative = 0;
        
        // RTK fields (unused)
        px4_gps.rtcm_injection_rate = 0.0f;
        px4_gps.selected_rtcm_instance = 0;
        px4_gps.rtcm_crc_failed = false;
        px4_gps.rtcm_msg_used = px4_msgs::msg::SensorGps::RTCM_MSG_USED_NOT_USED;
        
        // ALWAYS publish
        px4_gps_pub_->publish(px4_gps);
        
        // Debug output
        if (debug_logging_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "GPS PUBLISHED: lat=%.6f, lon=%.6f, alt=%.2f, vel=[%.2f,%.2f,%.2f], fix=%d", 
                px4_gps.latitude_deg, px4_gps.longitude_deg, px4_gps.altitude_msl_m,
                px4_gps.vel_n_m_s, px4_gps.vel_e_m_s, px4_gps.vel_d_m_s,
                px4_gps.fix_type);
        }
        
        // Update state
        prev_lat_ = msg->latitude;
        prev_lon_ = msg->longitude;
        prev_alt_ = msg->altitude;
        prev_time_ = current_time;
        msg_count_++;
    }
    
    void calculate_velocities(const sensor_msgs::msg::NavSatFix::SharedPtr msg,
                            px4_msgs::msg::SensorGps& px4_gps,
                            const rclcpp::Time& current_time)
    {
        if (!first_msg_ && msg_count_ > 2) {
            double dt = (current_time - prev_time_).seconds();
            
            if (dt > 0.02 && dt < 0.5) {
                // Calculate velocities
                const double EARTH_RADIUS = 6371000.0;
                double lat_rad = msg->latitude * M_PI / 180.0;
                
                double lat_dist = (msg->latitude - prev_lat_) * M_PI / 180.0 * EARTH_RADIUS;
                double lon_dist = (msg->longitude - prev_lon_) * M_PI / 180.0 * EARTH_RADIUS * cos(lat_rad);
                double alt_dist = msg->altitude - prev_alt_;
                
                px4_gps.vel_n_m_s = lat_dist / dt;
                px4_gps.vel_e_m_s = lon_dist / dt;
                px4_gps.vel_d_m_s = -alt_dist / dt; // NED: down positive
                
                // Limit velocities
                px4_gps.vel_n_m_s = std::clamp(px4_gps.vel_n_m_s, -10.0f, 10.0f);
                px4_gps.vel_e_m_s = std::clamp(px4_gps.vel_e_m_s, -10.0f, 10.0f);
                px4_gps.vel_d_m_s = std::clamp(px4_gps.vel_d_m_s, -5.0f, 5.0f);
                
                // Ground speed and course
                px4_gps.vel_m_s = sqrt(px4_gps.vel_n_m_s * px4_gps.vel_n_m_s + 
                                      px4_gps.vel_e_m_s * px4_gps.vel_e_m_s);
                px4_gps.cog_rad = atan2(px4_gps.vel_e_m_s, px4_gps.vel_n_m_s);
            } else {
                // Zero velocity for invalid dt
                px4_gps.vel_n_m_s = 0.0f;
                px4_gps.vel_e_m_s = 0.0f;
                px4_gps.vel_d_m_s = 0.0f;
                px4_gps.vel_m_s = 0.0f;
                px4_gps.cog_rad = 0.0f;
            }
        } else {
            // First messages - zero velocity
            px4_gps.vel_n_m_s = 0.0f;
            px4_gps.vel_e_m_s = 0.0f;
            px4_gps.vel_d_m_s = 0.0f;
            px4_gps.vel_m_s = 0.0f;
            px4_gps.cog_rad = 0.0f;
            first_msg_ = false;
        }
    }

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Publisher<px4_msgs::msg::SensorGps>::SharedPtr px4_gps_pub_;
    
    // State
    double prev_lat_, prev_lon_, prev_alt_;
    rclcpp::Time prev_time_;
    bool first_msg_;
    int msg_count_;
    
    // Parameters
    int device_id_;
    int velocity_filter_size_;
    bool debug_logging_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSBridge>());
    rclcpp::shutdown();
    return 0;
}