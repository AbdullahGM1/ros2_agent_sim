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

        // Time synchronization offset (to be calibrated)
        time_offset_us_ = 0;
        
        RCLCPP_INFO(this->get_logger(), "GPS Bridge started - Providing DYNAMIC position updates!");
    }

private:
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        px4_msgs::msg::SensorGps px4_gps;
        
        // Use simulation time with proper offset
        auto current_time = this->get_clock()->now();
        px4_gps.timestamp = current_time.nanoseconds() / 1000; // Convert to microseconds
        px4_gps.timestamp_sample = px4_gps.timestamp - 20000; // Sample taken 20ms before
        
        // CRITICAL FIX: Use ACTUAL GPS position from Gazebo (not fixed!)
        px4_gps.latitude_deg = msg->latitude;
        px4_gps.longitude_deg = msg->longitude;
        px4_gps.altitude_msl_m = msg->altitude;
        px4_gps.altitude_ellipsoid_m = msg->altitude + 30.0f; // Typical ellipsoid offset
        
        // Calculate velocities from position changes
        if (!first_msg_ && msg_count_ > 3) {
            double dt = (current_time - prev_time_).seconds();
            
            if (dt > 0.05 && dt < 1.0) { // Valid time delta
                // Convert lat/lon differences to meters
                double lat_diff = msg->latitude - prev_lat_;
                double lon_diff = msg->longitude - prev_lon_;
                double alt_diff = msg->altitude - prev_alt_;
                
                // Earth radius in meters
                const double EARTH_RADIUS = 6371000.0;
                
                // Convert to NED velocities
                // North velocity (latitude change)
                double lat_dist = lat_diff * M_PI / 180.0 * EARTH_RADIUS;
                px4_gps.vel_n_m_s = lat_dist / dt;
                
                // East velocity (longitude change, accounting for latitude)
                double lon_dist = lon_diff * M_PI / 180.0 * EARTH_RADIUS * cos(msg->latitude * M_PI / 180.0);
                px4_gps.vel_e_m_s = lon_dist / dt;
                
                // Down velocity (negative of altitude change)
                px4_gps.vel_d_m_s = -alt_diff / dt;
                
                // Apply low-pass filter to reduce noise
                const double alpha = 0.3; // Filter coefficient
                static float filtered_vel_n = 0.0f;
                static float filtered_vel_e = 0.0f;
                static float filtered_vel_d = 0.0f;
                
                filtered_vel_n = alpha * px4_gps.vel_n_m_s + (1.0 - alpha) * filtered_vel_n;
                filtered_vel_e = alpha * px4_gps.vel_e_m_s + (1.0 - alpha) * filtered_vel_e;
                filtered_vel_d = alpha * px4_gps.vel_d_m_s + (1.0 - alpha) * filtered_vel_d;
                
                px4_gps.vel_n_m_s = filtered_vel_n;
                px4_gps.vel_e_m_s = filtered_vel_e;
                px4_gps.vel_d_m_s = filtered_vel_d;
                
                // Calculate ground speed and course
                px4_gps.vel_m_s = sqrt(px4_gps.vel_n_m_s * px4_gps.vel_n_m_s + 
                                      px4_gps.vel_e_m_s * px4_gps.vel_e_m_s);
                px4_gps.cog_rad = atan2(px4_gps.vel_e_m_s, px4_gps.vel_n_m_s);
                
            } else {
                // Invalid time delta - use previous velocities with decay
                static float last_vel_n = 0.0f;
                static float last_vel_e = 0.0f;
                static float last_vel_d = 0.0f;
                
                px4_gps.vel_n_m_s = last_vel_n * 0.95f; // Decay factor
                px4_gps.vel_e_m_s = last_vel_e * 0.95f;
                px4_gps.vel_d_m_s = last_vel_d * 0.95f;
                
                last_vel_n = px4_gps.vel_n_m_s;
                last_vel_e = px4_gps.vel_e_m_s;
                last_vel_d = px4_gps.vel_d_m_s;
            }
        } else {
            // First few messages - set velocity to zero
            px4_gps.vel_n_m_s = 0.0f;
            px4_gps.vel_e_m_s = 0.0f;
            px4_gps.vel_d_m_s = 0.0f;
            px4_gps.vel_m_s = 0.0f;
            px4_gps.cog_rad = 0.0f;
            first_msg_ = false;
        }

        // Store current values for next iteration
        prev_lat_ = msg->latitude;
        prev_lon_ = msg->longitude;
        prev_alt_ = msg->altitude;
        prev_time_ = current_time;
        msg_count_++;

        // GPS quality parameters - realistic for good GPS
        px4_gps.fix_type = 3; // 3D fix
        px4_gps.satellites_used = 12;
        
        // Dynamic accuracy based on velocity (higher speed = slightly worse accuracy)
        float velocity_factor = 1.0f + (px4_gps.vel_m_s / 10.0f) * 0.5f;
        px4_gps.eph = 0.5f * velocity_factor;  // Horizontal accuracy
        px4_gps.epv = 1.0f * velocity_factor;  // Vertical accuracy
        px4_gps.hdop = 0.8f * velocity_factor; // Horizontal DOP
        px4_gps.vdop = 1.0f * velocity_factor; // Vertical DOP
        
        // Velocity accuracy
        px4_gps.s_variance_m_s = 0.2f; // 20 cm/s velocity accuracy
        
        // Additional fields
        px4_gps.vel_ned_valid = true;
        px4_gps.noise_per_ms = 5;
        px4_gps.jamming_indicator = 0;
        
        // Heading accuracy (if moving)
        if (px4_gps.vel_m_s > 0.5f) {
            px4_gps.heading_accuracy = 5.0f * M_PI / 180.0f; // 5 degrees in radians
        } else {
            px4_gps.heading_accuracy = NAN; // No heading when stationary
        }

        // Publish to PX4
        px4_gps_pub_->publish(px4_gps);

        // Debug logging (throttled)
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "GPS: lat=%.6f, lon=%.6f, alt=%.2f, vel=[%.2f,%.2f,%.2f] m/s, speed=%.2f m/s", 
            px4_gps.latitude_deg, px4_gps.longitude_deg, px4_gps.altitude_msl_m, 
            px4_gps.vel_n_m_s, px4_gps.vel_e_m_s, px4_gps.vel_d_m_s,
            px4_gps.vel_m_s);
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Publisher<px4_msgs::msg::SensorGps>::SharedPtr px4_gps_pub_;
    
    // For velocity calculation
    double prev_lat_, prev_lon_, prev_alt_;
    rclcpp::Time prev_time_;
    bool first_msg_;
    int msg_count_;
    int64_t time_offset_us_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSBridge>());
    rclcpp::shutdown();
    return 0;
}