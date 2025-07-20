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
        this->declare_parameter("gps_device_id", 1);
        this->declare_parameter("velocity_filter_size", 5);
        this->declare_parameter("position_filter_enabled", false);
        this->declare_parameter("debug_logging", false);
        
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

        // Publish to PX4 with proper QoS settings for uXRCE-DDS
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
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
        
        // CRITICAL: Set both timestamp fields properly
        auto current_time = this->get_clock()->now();
        uint64_t time_us = current_time.nanoseconds() / 1000; // Convert to microseconds
        
        px4_gps.timestamp = time_us;  // Time of this message
        px4_gps.timestamp_sample = time_us - 10000; // GPS sample taken 10ms earlier (typical GPS delay)
        
        // CRITICAL: Set device ID (required for sensor selection)
        px4_gps.device_id = device_id_;
        
        // Position data - use actual GPS data from Gazebo
        px4_gps.latitude_deg = msg->latitude;
        px4_gps.longitude_deg = msg->longitude;
        px4_gps.altitude_msl_m = msg->altitude;
        px4_gps.altitude_ellipsoid_m = msg->altitude + 30.0f; // WGS84 ellipsoid offset (location dependent)
        
        // Calculate velocities from position changes with robust filtering
        calculate_velocities(msg, px4_gps, current_time);
        
        // GPS quality parameters based on Gazebo simulation characteristics
        px4_gps.fix_type = 3; // 3D fix
        px4_gps.satellites_used = 10; // Typical for good GPS
        
        // Position accuracy (meters) - based on covariance if available
        if (msg->position_covariance[0] > 0) {
            px4_gps.eph = sqrt(msg->position_covariance[0]); // Horizontal accuracy from covariance
            px4_gps.epv = sqrt(msg->position_covariance[8]); // Vertical accuracy from covariance
        } else {
            // Default simulation GPS accuracy
            px4_gps.eph = 0.5f;  // 50cm horizontal accuracy
            px4_gps.epv = 1.0f;  // 1m vertical accuracy
        }
        
        // Dilution of Precision (DOP) values
        px4_gps.hdop = 0.9f;  // Good horizontal DOP
        px4_gps.vdop = 1.2f;  // Good vertical DOP
        
        // Velocity accuracy
        px4_gps.s_variance_m_s = 0.2f;  // 20 cm/s speed accuracy
        px4_gps.c_variance_rad = 0.1f;  // Course accuracy in radians
        
        // CRITICAL: This flag must be true for velocity to be used
        px4_gps.vel_ned_valid = true;
        
        // Additional quality parameters
        px4_gps.noise_per_ms = 20;      // GPS noise level
        px4_gps.jamming_indicator = 0;   // No jamming
        px4_gps.spoofing_state = px4_msgs::msg::SensorGps::SPOOFING_STATE_NONE;
        
        // UTC time (optional, but good for simulation)
        px4_gps.time_utc_usec = time_us + 1609459200000000ULL; // Add seconds from 1970 to 2021
        px4_gps.timestamp_time_relative = 0; // We're providing absolute UTC time
        
        // Heading accuracy (only valid when moving)
        if (px4_gps.vel_m_s > 0.5f) {
            px4_gps.heading = atan2(px4_gps.vel_e_m_s, px4_gps.vel_n_m_s); // Heading from velocity
            px4_gps.heading_offset = 0.0f; // No dual antenna offset
            px4_gps.heading_accuracy = 0.1f; // 0.1 rad accuracy when moving
        } else {
            px4_gps.heading = NAN;
            px4_gps.heading_offset = NAN;
            px4_gps.heading_accuracy = NAN;
        }
        
        // RTK fields (not used in basic GPS)
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
        
        // Debug logging
        if (debug_logging_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "GPS: lat=%.7f, lon=%.7f, alt=%.2f, vel=[%.2f,%.2f,%.2f] m/s, hdg=%.2f°, sats=%d", 
                px4_gps.latitude_deg, px4_gps.longitude_deg, px4_gps.altitude_msl_m, 
                px4_gps.vel_n_m_s, px4_gps.vel_e_m_s, px4_gps.vel_d_m_s,
                std::isnan(px4_gps.heading) ? 0.0 : px4_gps.heading * 180.0 / M_PI,
                px4_gps.satellites_used);
        }
    }
    
    void calculate_velocities(const sensor_msgs::msg::NavSatFix::SharedPtr msg,
                            px4_msgs::msg::SensorGps& px4_gps,
                            const rclcpp::Time& current_time)
    {
        if (!first_msg_ && msg_count_ > 2) {
            double dt = (current_time - prev_time_).seconds();
            
            if (dt > 0.01 && dt < 0.5) { // Valid time delta (10ms to 500ms)
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
                
                // Apply moving average filter
                update_velocity_filter(vel_n_raw, vel_e_raw, vel_d_raw);
                
                // Get filtered velocities
                px4_gps.vel_n_m_s = get_filtered_velocity(vel_n_filter_);
                px4_gps.vel_e_m_s = get_filtered_velocity(vel_e_filter_);
                px4_gps.vel_d_m_s = get_filtered_velocity(vel_d_filter_);
                
                // Limit velocities to reasonable values for a drone
                const float MAX_HORIZONTAL_VEL = 20.0f; // 20 m/s max horizontal
                const float MAX_VERTICAL_VEL = 10.0f;   // 10 m/s max vertical
                
                px4_gps.vel_n_m_s = std::clamp(px4_gps.vel_n_m_s, -MAX_HORIZONTAL_VEL, MAX_HORIZONTAL_VEL);
                px4_gps.vel_e_m_s = std::clamp(px4_gps.vel_e_m_s, -MAX_HORIZONTAL_VEL, MAX_HORIZONTAL_VEL);
                px4_gps.vel_d_m_s = std::clamp(px4_gps.vel_d_m_s, -MAX_VERTICAL_VEL, MAX_VERTICAL_VEL);
                
                // Calculate ground speed and course over ground
                px4_gps.vel_m_s = sqrt(px4_gps.vel_n_m_s * px4_gps.vel_n_m_s + 
                                      px4_gps.vel_e_m_s * px4_gps.vel_e_m_s);
                px4_gps.cog_rad = atan2(px4_gps.vel_e_m_s, px4_gps.vel_n_m_s);
                
            } else {
                // Invalid dt - use last filtered values with decay
                px4_gps.vel_n_m_s = get_filtered_velocity(vel_n_filter_) * 0.95f;
                px4_gps.vel_e_m_s = get_filtered_velocity(vel_e_filter_) * 0.95f;
                px4_gps.vel_d_m_s = get_filtered_velocity(vel_d_filter_) * 0.95f;
                px4_gps.vel_m_s = sqrt(px4_gps.vel_n_m_s * px4_gps.vel_n_m_s + 
                                      px4_gps.vel_e_m_s * px4_gps.vel_e_m_s);
                px4_gps.cog_rad = atan2(px4_gps.vel_e_m_s, px4_gps.vel_n_m_s);
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
        
        // Maintain filter size
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