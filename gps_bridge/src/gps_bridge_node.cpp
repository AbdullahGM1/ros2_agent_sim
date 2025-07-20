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
        this->declare_parameter("position_filter_enabled", true);  // ENABLE position filtering
        this->declare_parameter("debug_logging", true);
        
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

        // Publish to correct namespaced topic
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
        gps_origin_set_ = false;
        
        // Velocity filters
        vel_n_filter_.clear();
        vel_e_filter_.clear();
        vel_d_filter_.clear();
        
        // Position filters
        lat_filter_.clear();
        lon_filter_.clear();
        alt_filter_.clear();
        
        // Expected GPS coordinates for Palo Alto (from your world file)
        expected_lat_ = 37.4223;  // degrees
        expected_lon_ = -122.0844; // degrees
        expected_alt_ = 0.0;      // meters
        
        RCLCPP_INFO(this->get_logger(), "GPS Bridge initialized - Device ID: %d", device_id_);
        RCLCPP_INFO(this->get_logger(), "Expected GPS: lat=%.6f, lon=%.6f, alt=%.1f", 
                   expected_lat_, expected_lon_, expected_alt_);
    }

private:
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        // CRITICAL: Validate GPS coordinates are reasonable
        if (!is_gps_valid(msg)) {
            if (debug_logging_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Invalid GPS data: lat=%.6f, lon=%.6f, alt=%.2f", 
                    msg->latitude, msg->longitude, msg->altitude);
            }
            return;
        }
        
        px4_msgs::msg::SensorGps px4_gps;
        
        // CRITICAL: Stable timestamp
        auto current_time = this->get_clock()->now();
        uint64_t time_us = current_time.nanoseconds() / 1000;
        px4_gps.timestamp = time_us;
        
        // CRITICAL: Consistent device ID
        px4_gps.device_id = device_id_;
        
        // CRITICAL: Filter GPS position for stability
        double filtered_lat, filtered_lon, filtered_alt;
        if (position_filter_enabled_) {
            filter_gps_position(msg, filtered_lat, filtered_lon, filtered_alt);
        } else {
            filtered_lat = msg->latitude;
            filtered_lon = msg->longitude;
            filtered_alt = msg->altitude;
        }
        
        px4_gps.latitude_deg = filtered_lat;
        px4_gps.longitude_deg = filtered_lon;
        px4_gps.altitude_msl_m = filtered_alt;
        px4_gps.altitude_ellipsoid_m = filtered_alt;
        
        // Calculate stable velocities
        calculate_stable_velocities(filtered_lat, filtered_lon, filtered_alt, px4_gps, current_time);
        
        // CRITICAL: Conservative GPS quality for stability
        px4_gps.fix_type = 3;                    // 3D fix
        px4_gps.satellites_used = 12;            // Good satellite count
        
        // CRITICAL: Consistent accuracy values for EKF2 acceptance
        px4_gps.eph = 2.0f;                     // 2m horizontal accuracy (conservative)
        px4_gps.epv = 3.0f;                     // 3m vertical accuracy (conservative)
        px4_gps.hdop = 1.0f;                    // Good HDOP
        px4_gps.vdop = 1.2f;                    // Good VDOP
        
        // Velocity accuracy
        px4_gps.s_variance_m_s = 0.5f;          // 50cm/s speed accuracy
        px4_gps.c_variance_rad = 0.3f;          // Course accuracy
        
        // CRITICAL: Always enable velocity
        px4_gps.vel_ned_valid = true;
        
        // Quality parameters
        px4_gps.noise_per_ms = 10;              // Low noise
        px4_gps.jamming_indicator = 0;          // No jamming
        px4_gps.spoofing_state = px4_msgs::msg::SensorGps::SPOOFING_STATE_NONE;
        
        // CRITICAL: No GPS heading - let magnetometer handle it
        px4_gps.heading = NAN;
        px4_gps.heading_offset = NAN;
        px4_gps.heading_accuracy = NAN;
        
        // Simplified time fields
        px4_gps.time_utc_usec = 0;
        px4_gps.timestamp_time_relative = 0;
        
        // RTK fields (unused)
        px4_gps.rtcm_injection_rate = 0.0f;
        px4_gps.selected_rtcm_instance = 0;
        px4_gps.rtcm_crc_failed = false;
        px4_gps.rtcm_msg_used = px4_msgs::msg::SensorGps::RTCM_MSG_USED_NOT_USED;
        
        // Publish stable GPS data
        px4_gps_pub_->publish(px4_gps);
        
        // Update state
        prev_lat_ = filtered_lat;
        prev_lon_ = filtered_lon;
        prev_alt_ = filtered_alt;
        prev_time_ = current_time;
        msg_count_++;
        
        if (debug_logging_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "GPS: lat=%.6f, lon=%.6f, alt=%.2f | vel=[%.2f,%.2f,%.2f] | eph=%.1f, sats=%d", 
                px4_gps.latitude_deg, px4_gps.longitude_deg, px4_gps.altitude_msl_m,
                px4_gps.vel_n_m_s, px4_gps.vel_e_m_s, px4_gps.vel_d_m_s,
                px4_gps.eph, px4_gps.satellites_used);
        }
    }
    
    bool is_gps_valid(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        // Check for NaN or inf values
        if (!std::isfinite(msg->latitude) || !std::isfinite(msg->longitude) || !std::isfinite(msg->altitude)) {
            return false;
        }
        
        // Check GPS coordinates are within reasonable bounds
        if (std::abs(msg->latitude) > 90.0 || std::abs(msg->longitude) > 180.0) {
            return false;
        }
        
        // Check altitude is reasonable (-500m to +10000m)
        if (msg->altitude < -500.0 || msg->altitude > 10000.0) {
            return false;
        }
        
        // For first message, just validate basic bounds
        if (!gps_origin_set_) {
            gps_origin_set_ = true;
            origin_lat_ = msg->latitude;
            origin_lon_ = msg->longitude;
            origin_alt_ = msg->altitude;
            return true;
        }
        
        // Check GPS hasn't jumped too far from origin (within 1000m should be fine for simulation)
        const double EARTH_RADIUS = 6371000.0;
        double lat_dist = (msg->latitude - origin_lat_) * M_PI / 180.0 * EARTH_RADIUS;
        double lon_dist = (msg->longitude - origin_lon_) * M_PI / 180.0 * EARTH_RADIUS * cos(origin_lat_ * M_PI / 180.0);
        double distance = sqrt(lat_dist*lat_dist + lon_dist*lon_dist);
        
        if (distance > 1000.0) {  // 1km max jump
            return false;
        }
        
        return true;
    }
    
    void filter_gps_position(const sensor_msgs::msg::NavSatFix::SharedPtr msg,
                           double& filtered_lat, double& filtered_lon, double& filtered_alt)
    {
        // Add to position filters
        lat_filter_.push_back(msg->latitude);
        lon_filter_.push_back(msg->longitude);
        alt_filter_.push_back(msg->altitude);
        
        // Maintain filter size
        const size_t POSITION_FILTER_SIZE = 3;  // Small filter for responsiveness
        while (lat_filter_.size() > POSITION_FILTER_SIZE) {
            lat_filter_.pop_front();
            lon_filter_.pop_front();
            alt_filter_.pop_front();
        }
        
        // Calculate filtered positions (simple average)
        filtered_lat = 0.0;
        filtered_lon = 0.0;
        filtered_alt = 0.0;
        
        for (size_t i = 0; i < lat_filter_.size(); ++i) {
            filtered_lat += lat_filter_[i];
            filtered_lon += lon_filter_[i];
            filtered_alt += alt_filter_[i];
        }
        
        filtered_lat /= lat_filter_.size();
        filtered_lon /= lon_filter_.size();
        filtered_alt /= alt_filter_.size();
    }
    
    void calculate_stable_velocities(double lat, double lon, double alt,
                                   px4_msgs::msg::SensorGps& px4_gps,
                                   const rclcpp::Time& current_time)
    {
        if (!first_msg_ && msg_count_ > 2) {
            double dt = (current_time - prev_time_).seconds();
            
            if (dt > 0.02 && dt < 0.5) {  // Valid time delta (20ms to 500ms)
                // Calculate position changes in meters
                const double EARTH_RADIUS = 6371000.0;
                double lat_rad = lat * M_PI / 180.0;
                
                double lat_dist = (lat - prev_lat_) * M_PI / 180.0 * EARTH_RADIUS;
                double lon_dist = (lon - prev_lon_) * M_PI / 180.0 * EARTH_RADIUS * cos(lat_rad);
                double alt_dist = alt - prev_alt_;
                
                // Calculate raw velocities
                float vel_n_raw = lat_dist / dt;
                float vel_e_raw = lon_dist / dt;
                float vel_d_raw = -alt_dist / dt; // NED: down positive
                
                // CRITICAL: Limit velocity jumps for stability
                const float MAX_VEL_CHANGE = 5.0f; // 5 m/s max change per update
                if (std::abs(vel_n_raw) < MAX_VEL_CHANGE && 
                    std::abs(vel_e_raw) < MAX_VEL_CHANGE && 
                    std::abs(vel_d_raw) < MAX_VEL_CHANGE) {
                    
                    update_velocity_filter(vel_n_raw, vel_e_raw, vel_d_raw);
                }
                
                // Get smoothed velocities
                px4_gps.vel_n_m_s = get_filtered_velocity(vel_n_filter_);
                px4_gps.vel_e_m_s = get_filtered_velocity(vel_e_filter_);
                px4_gps.vel_d_m_s = get_filtered_velocity(vel_d_filter_);
                
                // CRITICAL: Conservative velocity limits for stability
                const float MAX_HORIZONTAL_VEL = 10.0f;
                const float MAX_VERTICAL_VEL = 5.0f;
                
                px4_gps.vel_n_m_s = std::clamp(px4_gps.vel_n_m_s, -MAX_HORIZONTAL_VEL, MAX_HORIZONTAL_VEL);
                px4_gps.vel_e_m_s = std::clamp(px4_gps.vel_e_m_s, -MAX_HORIZONTAL_VEL, MAX_HORIZONTAL_VEL);
                px4_gps.vel_d_m_s = std::clamp(px4_gps.vel_d_m_s, -MAX_VERTICAL_VEL, MAX_VERTICAL_VEL);
                
                // Calculate ground speed and course
                px4_gps.vel_m_s = sqrt(px4_gps.vel_n_m_s * px4_gps.vel_n_m_s + 
                                      px4_gps.vel_e_m_s * px4_gps.vel_e_m_s);
                px4_gps.cog_rad = atan2(px4_gps.vel_e_m_s, px4_gps.vel_n_m_s);
                
            } else {
                // Invalid dt - use last known velocities with decay
                px4_gps.vel_n_m_s = get_filtered_velocity(vel_n_filter_) * 0.9f;
                px4_gps.vel_e_m_s = get_filtered_velocity(vel_e_filter_) * 0.9f;
                px4_gps.vel_d_m_s = get_filtered_velocity(vel_d_filter_) * 0.9f;
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
        vel_n_filter_.push_back(vel_n);
        vel_e_filter_.push_back(vel_e);
        vel_d_filter_.push_back(vel_d);
        
        while (vel_n_filter_.size() > static_cast<size_t>(velocity_filter_size_)) {
            vel_n_filter_.pop_front();
            vel_e_filter_.pop_front();
            vel_d_filter_.pop_front();
        }
    }
    
    float get_filtered_velocity(const std::deque<float>& filter)
    {
        if (filter.empty()) return 0.0f;
        
        // Use median filter for better outlier rejection
        std::vector<float> values(filter.begin(), filter.end());
        std::sort(values.begin(), values.end());
        
        if (values.size() % 2 == 0) {
            return (values[values.size()/2 - 1] + values[values.size()/2]) / 2.0f;
        } else {
            return values[values.size()/2];
        }
    }

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Publisher<px4_msgs::msg::SensorGps>::SharedPtr px4_gps_pub_;
    
    // State tracking
    double prev_lat_, prev_lon_, prev_alt_;
    double origin_lat_, origin_lon_, origin_alt_;
    double expected_lat_, expected_lon_, expected_alt_;
    rclcpp::Time prev_time_;
    bool first_msg_;
    bool gps_origin_set_;
    int msg_count_;
    
    // Filtering
    std::deque<float> vel_n_filter_;
    std::deque<float> vel_e_filter_;
    std::deque<float> vel_d_filter_;
    std::deque<double> lat_filter_;
    std::deque<double> lon_filter_;
    std::deque<double> alt_filter_;
    
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