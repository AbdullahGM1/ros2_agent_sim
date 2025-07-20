#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <chrono>
#include <cmath>
#include <deque>

// Define Earth's radius for geographic calculations
const double EARTH_RADIUS = 6371000.0; // meters

class GPSBridge : public rclcpp::Node
{
public:
    GPSBridge() : Node("gps_bridge")
    {
        // Declare parameters
        this->declare_parameter("gps_device_id", 1310988);
        this->declare_parameter("debug_logging", true);
        
        device_id_ = this->get_parameter("gps_device_id").as_int();
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
        
        // Use the timestamp from the incoming ROS message for PX4 timestamp
        // This is crucial for EKF2 synchronization.
        auto current_time = msg->header.stamp;
        // Corrected: Use 'nanosec' member for nanoseconds, then convert to microseconds
        px4_gps.timestamp = current_time.sec * 1000000ULL + current_time.nanosec / 1000ULL;
        
        // Device ID
        px4_gps.device_id = device_id_;
        
        // Position data - NO VALIDATION, just copy
        px4_gps.latitude_deg = msg->latitude;
        px4_gps.longitude_deg = msg->longitude;
        px4_gps.altitude_msl_m = msg->altitude;
        px4_gps.altitude_ellipsoid_m = msg->altitude; // Assuming MSL and Ellipsoid are the same for simulation

        // Calculate velocities
        calculate_velocities(msg, px4_gps, current_time);
        
        // Fixed GPS quality parameters - these are what EKF2 uses to decide if data is good enough
        px4_gps.fix_type = 3;                       // 3D fix
        px4_gps.satellites_used = 12;               // Good satellite count
        px4_gps.eph = 1.0f;                         // Horizontal accuracy (m) - Set to a very good value for simulation
        px4_gps.epv = 1.5f;                         // Vertical accuracy (m) - Set to a very good value for simulation
        px4_gps.hdop = 0.8f;                        // Good HDOP
        px4_gps.vdop = 1.0f;                        // Good VDOP
        px4_gps.s_variance_m_s = 0.2f;              // Speed accuracy (m/s) - Set to a very good value
        px4_gps.c_variance_rad = 0.1f;              // Course accuracy (rad) - Set to a very good value
        px4_gps.vel_ned_valid = true;               // Always enable velocity
        px4_gps.noise_per_ms = 10;                  // Low noise
        px4_gps.jamming_indicator = 0;              // No jamming
        // CRITICAL FIX: Explicitly set spoofing_state to 0 to prevent misinterpretation by the bridge.
        px4_gps.spoofing_state = 0; 
        
        // No GPS heading (if not provided by Gazebo)
        px4_gps.heading = NAN;
        px4_gps.heading_offset = NAN;
        px4_gps.heading_accuracy = NAN;
        
        // Simple time fields (can be left as 0 for simulation if not used by EKF2)
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
                "GPS PUBLISHED: lat=%.6f, lon=%.6f, alt=%.2f, vel=[%.2f,%.2f,%.2f], fix=%d, eph=%.2f, epv=%.2f, s_variance=%.2f, spoofing_state=%d", 
                px4_gps.latitude_deg, px4_gps.longitude_deg, px4_gps.altitude_msl_m,
                px4_gps.vel_n_m_s, px4_gps.vel_e_m_s, px4_gps.vel_d_m_s,
                px4_gps.fix_type, px4_gps.eph, px4_gps.epv, px4_gps.s_variance_m_s,
                px4_gps.spoofing_state); // Added spoofing_state to debug output
        }
        
        // Update state
        prev_lat_ = msg->latitude;
        prev_lon_ = msg->longitude;
        prev_alt_ = msg->altitude;
        prev_time_ = current_time;
    }
    
    void calculate_velocities(const sensor_msgs::msg::NavSatFix::SharedPtr msg,
                              px4_msgs::msg::SensorGps& px4_gps,
                              const rclcpp::Time& current_time)
    {
        // Only calculate velocity if we have a previous message and a valid time difference
        if (!first_msg_) {
            double dt = (current_time - prev_time_).seconds();
            
            // Ensure a reasonable time step to avoid division by zero or extreme velocities
            // A minimum dt of 0.01s (10ms) corresponds to a 100Hz update rate.
            // A maximum dt prevents old data from causing huge velocity spikes.
            if (dt > 0.005 && dt < 1.0) { // Relaxed dt bounds slightly
                // Calculate velocities
                double lat_rad = msg->latitude * M_PI / 180.0;
                
                // Convert latitude/longitude differences to meters
                double delta_lat_m = (msg->latitude - prev_lat_) * M_PI / 180.0 * EARTH_RADIUS;
                double delta_lon_m = (msg->longitude - prev_lon_) * M_PI / 180.0 * EARTH_RADIUS * cos(lat_rad);
                double delta_alt_m = msg->altitude - prev_alt_;
                
                px4_gps.vel_n_m_s = delta_lat_m / dt;
                px4_gps.vel_e_m_s = delta_lon_m / dt;
                px4_gps.vel_d_m_s = -delta_alt_m / dt; // NED: down positive
                
                // Apply more aggressive clamping to velocities to prevent large spikes
                // EKF2 is sensitive to large, sudden velocity changes.
                px4_gps.vel_n_m_s = std::clamp(px4_gps.vel_n_m_s, -5.0f, 5.0f); // Max 5 m/s (approx 18 km/h)
                px4_gps.vel_e_m_s = std::clamp(px4_gps.vel_e_m_s, -5.0f, 5.0f);
                px4_gps.vel_d_m_s = std::clamp(px4_gps.vel_d_m_s, -2.0f, 2.0f); // Max 2 m/s vertical
                
                // Ground speed and course
                px4_gps.vel_m_s = sqrt(px4_gps.vel_n_m_s * px4_gps.vel_n_m_s + 
                                       px4_gps.vel_e_m_s * px4_gps.vel_e_m_s);
                px4_gps.cog_rad = atan2(px4_gps.vel_e_m_s, px4_gps.vel_n_m_s);
            } else {
                // If dt is invalid, zero out velocities to avoid issues
                px4_gps.vel_n_m_s = 0.001f;
                px4_gps.vel_e_m_s = 0.001f;
                px4_gps.vel_d_m_s = 0.001f;
                px4_gps.vel_m_s = 0.002f;
                px4_gps.cog_rad = 0.0f;
                if (debug_logging_) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                        "Invalid dt for velocity calculation: %.4f seconds. Setting velocities to small values.", dt);
                }
            }
        } else {
            // First message - zero velocity
            px4_gps.vel_n_m_s = 0.001f;  // 1mm/s instead of 0
            px4_gps.vel_e_m_s = 0.001f;  // 1mm/s instead of 0  
            px4_gps.vel_d_m_s = 0.001f;  // 1mm/s instead of 0
            px4_gps.vel_m_s = 0.002f;    // Small ground speed
            px4_gps.cog_rad = 0.0f;      // Course over ground can be 0
            first_msg_ = false; // Mark first message processed
        }
    }

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Publisher<px4_msgs::msg::SensorGps>::SharedPtr px4_gps_pub_;
    
    // State
    double prev_lat_, prev_lon_, prev_alt_;
    rclcpp::Time prev_time_;
    bool first_msg_;
    
    // Parameters
    int device_id_;
    bool debug_logging_;
};

int main(int argc, char *argv[])
{
    // Corrected: Pass both argc and argv to rclcpp::init
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSBridge>());
    rclcpp::shutdown();
    return 0;
}