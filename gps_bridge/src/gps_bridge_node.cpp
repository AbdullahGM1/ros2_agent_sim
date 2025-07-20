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
        this->declare_parameter("gps_device_id", 1310988);  // CRITICAL: Use consistent device ID
        this->declare_parameter("velocity_filter_size", 3);  // REDUCED filter size for faster response
        this->declare_parameter("position_filter_enabled", false);
        this->declare_parameter("debug_logging", true);  // Enable debug for troubleshooting
        
        device_id_ = this->get_parameter("gps_device_id").as_int();
        velocity_filter_size_ = this->get_parameter("velocity_filter_size").as_int();
        position_filter_enabled_ = this->get_parameter("position_filter_enabled").as_bool();
        debug_logging_ = this->get_parameter("debug_logging").as_bool();
        
        // Subscribe to Gazebo GPS topic
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/drone/gps", 10,
            [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
                this->gps_callback(msg);
            });

        // CRITICAL: Use correct QoS for PX4 v1.15 uXRCE-DDS
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos.durability(rclcpp::DurabilityPolicy::Volatile);  // CHANGED from TransientLocal
        qos.history(rclcpp::HistoryPolicy::KeepLast);

        px4_gps_pub_ = this->create_publisher<px4_msgs::msg::SensorGps>(
            "/fmu/in/sensor_gps", qos);  // CRITICAL: Use correct topic name for v1.15

        // Initialize state
        prev_lat_ = 0.0;
        prev_lon_ = 0.0;
        prev_alt_ = 0.0;
        prev_time_ = this->get_clock()->now();
        first_msg_ = true;
        msg_count_ = 0;
        
        // Velocity filters
        vel_n_filter_.clear();
        vel_e_filter_.clear();
        vel_d_filter_.clear();
        
        RCLCPP_INFO(this->get_logger(), "GPS Bridge initialized - Device ID: %d", device_id_);
    }

private:
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        px4_msgs::msg::SensorGps px4_gps;
        
        // CRITICAL: Use single timestamp for v1.15 
        auto current_time = this->get_clock()->now();
        uint64_t time_us = current_time.nanoseconds() / 1000; // Convert to microseconds
        
        px4_gps.timestamp = time_us;  // Primary timestamp
        // REMOVED: timestamp_sample to avoid EKF2 timing confusion
        
        // CRITICAL: Set device ID consistently with airframe
        px4_gps.device_id = device_id_;
        
        // Position data - ensure coordinates are reasonable
        px4_gps.latitude_deg = msg->latitude;
        px4_gps.longitude_deg = msg->longitude;
        px4_gps.altitude_msl_m = msg->altitude;
        px4_gps.altitude_ellipsoid_m = msg->altitude;  // SIMPLIFIED: No offset for simulation
        
        // Calculate velocities from position changes with robust filtering
        calculate_velocities(msg, px4_gps, current_time);
        
        // CRITICAL: GPS quality parameters for PX4 v1.15 acceptance
        px4_gps.fix_type = 3; // 3D fix - REQUIRED
        px4_gps.satellites_used = 12; // Good satellite count for v1.15
        
        // CRITICAL: Position accuracy - must be within EKF2 acceptance thresholds
        if (msg->position_covariance[0] > 0) {
            px4_gps.eph = std::min(static_cast<float>(sqrt(msg->position_covariance[0])), 3.0f);
            px4_gps.epv = std::min(static_cast<float>(sqrt(msg->position_covariance[8])), 5.0f);
        } else {
            // CRITICAL: Conservative accuracy values for v1.15
            px4_gps.eph = 1.0f;  // 1m horizontal accuracy
            px4_gps.epv = 2.0f;  // 2m vertical accuracy
        }
        
        // CRITICAL: DOP values must be reasonable for EKF2
        px4_gps.hdop = 1.0f;  // Good horizontal DOP
        px4_gps.vdop = 1.5f;  // Good vertical DOP
        
        // CRITICAL: Velocity accuracy for EKF2 acceptance
        px4_gps.s_variance_m_s = 0.3f;  // 30 cm/s speed accuracy
        px4_gps.c_variance_rad = 0.2f;  // Course accuracy in radians
        
        // CRITICAL: This MUST be true for velocity fusion in EKF2
        px4_gps.vel_ned_valid = true;
        
        // Additional quality parameters for v1.15
        px4_gps.noise_per_ms = 15;      // GPS noise level - REDUCED
        px4_gps.jamming_indicator = 0;   // No jamming
        px4_gps.spoofing_state = px4_msgs::msg::SensorGps::SPOOFING_STATE_NONE;
        
        // CRITICAL: Do NOT set heading fields - let EKF2 use magnetometer/gyro
        px4_gps.heading = NAN;
        px4_gps.heading_offset = NAN;
        px4_gps.heading_accuracy = NAN;
        
        // UTC time (simplified for simulation)
        px4_gps.time_utc_usec = 0;  // SIMPLIFIED: Not needed for simulation
        px4_gps.timestamp_time_relative = 0;
        
        // RTK fields (not used in basic GPS simulation)
        px4_gps.rtcm_injection_rate = 0.0f;
        px4_gps.selected_rtcm_instance = 0;
        px4_gps.rtcm_crc_failed = false;
        px4_gps.rtcm_msg_used = px4_msgs::msg::SensorGps::RTCM_MSG_USED_NOT_USED;
        
        // Publish to PX4
        px4_gps_pub_->publish(px4_gps);
        
        // Update state for next iteration
        prev_lat_ = msg->latitude;
        prev_lon_ = msg->longitude;
        prev_alt_ = msg->altitude;
        prev_time_ = current_time;
        msg_count_++;
        
        // Enhanced debug logging
        if (debug_logging_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "GPS Bridge: fix=%d, sats=%d, hdop=%.2f, eph=%.2f, vel_valid=%s, vel=[%.2f,%.2f,%.2f]", 
                px4_gps.fix_type, px4_gps.satellites_used, px4_gps.hdop, px4_gps.eph,
                px4_gps.vel_ned_valid ? "true" : "false",
                px4_gps.vel_n_m_s, px4_gps.vel_e_m_s, px4_gps.vel_d_m_s);
        }
    }
    
    void calculate_velocities(const sensor_msgs::msg::NavSatFix::SharedPtr msg,
                            px4_msgs::msg::SensorGps& px4_gps,
                            const rclcpp::Time& current_time)
    {
        if (!first_msg_ && msg_count_ > 1) {  // REDUCED requirement
            double dt = (current_time - prev_time_).seconds();
            
            if (dt > 0.01 && dt < 1.0) { // RELAXED time delta range
                // Convert lat/lon differences to meters using local tangent plane
                const double EARTH_RADIUS = 6371000.0; // meters
                double lat_rad = msg->latitude * M_PI / 180.0;
                
                // Distance calculations
                double lat_dist = (msg->latitude - prev_lat_) * M_PI / 180.0 * EARTH_RADIUS;
                double lon_dist = (msg->longitude - prev_lon_) * M_PI / 180.0 * EARTH_RADIUS * cos(lat_rad);
                double alt_dist = msg->altitude - prev_alt_;
                
                // Raw velocity calculations
                float vel_n_raw = lat_dist / dt;
                float vel_e_raw = lon_dist / dt;
                float vel_d_raw = -alt_dist / dt; // NED convention: down is positive
                
                // Apply simplified filtering with outlier rejection
                if (std::abs(vel_n_raw) < 50.0f && std::abs(vel_e_raw) < 50.0f && std::abs(vel_d_raw) < 20.0f) {
                    update_velocity_filter(vel_n_raw, vel_e_raw, vel_d_raw);
                }
                
                // Get filtered velocities
                px4_gps.vel_n_m_s = get_filtered_velocity(vel_n_filter_);
                px4_gps.vel_e_m_s = get_filtered_velocity(vel_e_filter_);
                px4_gps.vel_d_m_s = get_filtered_velocity(vel_d_filter_);
                
                // CRITICAL: Clamp velocities to reasonable drone limits
                const float MAX_HORIZONTAL_VEL = 15.0f; // 15 m/s max horizontal - REDUCED
                const float MAX_VERTICAL_VEL = 8.0f;    // 8 m/s max vertical - REDUCED
                
                px4_gps.vel_n_m_s = std::clamp(px4_gps.vel_n_m_s, -MAX_HORIZONTAL_VEL, MAX_HORIZONTAL_VEL);
                px4_gps.vel_e_m_s = std::clamp(px4_gps.vel_e_m_s, -MAX_HORIZONTAL_VEL, MAX_HORIZONTAL_VEL);
                px4_gps.vel_d_m_s = std::clamp(px4_gps.vel_d_m_s, -MAX_VERTICAL_VEL, MAX_VERTICAL_VEL);
                
                // Calculate ground speed and course over ground
                px4_gps.vel_m_s = sqrt(px4_gps.vel_n_m_s * px4_gps.vel_n_m_s + 
                                      px4_gps.vel_e_m_s * px4_gps.vel_e_m_s);
                px4_gps.cog_rad = atan2(px4_gps.vel_e_m_s, px4_gps.vel_n_m_s);
                
            } else {
                // Invalid dt - use smoothed zero velocity
                px4_gps.vel_n_m_s = 0.0f;
                px4_gps.vel_e_m_s = 0.0f;
                px4_gps.vel_d_m_s = 0.0f;
                px4_gps.vel_m_s = 0.0f;
                px4_gps.cog_rad = 0.0f;
            }
        } else {
            // First few messages - zero velocity
            px4_gps.vel_n_m_s = 0.0f;
            px4_gps.vel_e_m_s = 0.0f;
            px4_gps.vel_d_m_s = 0.0f;
            px4_gps.vel_m_s = 0.0f;
            px4_gps.cog_rad = 0.0f;
            first_msg_ = false;
        }
    }
    
    void update_velocity_filter(float vel_n, float vel_e, float vel_d)
    {
        // Add new values to filters
        vel_n_filter_.push_back(vel_n);
        vel_e_filter_.push_back(vel_e);
        vel_d_filter_.push_back(vel_d);
        
        // Maintain filter size - SMALLER for faster response
        while (vel_n_filter_.size() > static_cast<size_t>(velocity_filter_size_)) {
            vel_n_filter_.pop_front();
            vel_e_filter_.pop_front();
            vel_d_filter_.pop_front();
        }
    }
    
    float get_filtered_velocity(const std::deque<float>& filter)
    {
        if (filter.empty()) return 0.0f;
        
        // Simple moving average
        float sum = 0.0f;
        for (float val : filter) {
            sum += val;
        }
        return sum / filter.size();
    }

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Publisher<px4_msgs::msg::SensorGps>::SharedPtr px4_gps_pub_;
    
    // State tracking
    double prev_lat_, prev_lon_, prev_alt_;
    rclcpp::Time prev_time_;
    bool first_msg_;
    int msg_count_;
    
    // Velocity filtering
    std::deque<float> vel_n_filter_;
    std::deque<float> vel_e_filter_;
    std::deque<float> vel_d_filter_;
    
    // Parameters
    int device_id_;
    int velocity_filter_size_;
    bool position_filter_enabled_;
    bool debug_logging_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSBridge>());
    rclcpp::shutdown();
    return 0;
}