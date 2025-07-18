#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <chrono>

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

        // Publish to PX4 
        px4_gps_pub_ = this->create_publisher<px4_msgs::msg::SensorGps>(
            "/fmu/in/sensor_gps", 10);

        RCLCPP_INFO(this->get_logger(), "GPS Bridge started!");
    }

private:
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        px4_msgs::msg::SensorGps px4_gps;
        
        // Set timestamp
        px4_gps.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now()).time_since_epoch().count();

        // GPS position
        px4_gps.latitude_deg = msg->latitude;
        px4_gps.longitude_deg = msg->longitude;
        px4_gps.altitude_msl_m = msg->altitude;
        px4_gps.altitude_ellipsoid_m = msg->altitude;

        // GPS status
        px4_gps.fix_type = 3; // 3D fix
        px4_gps.satellites_used = 8;
        px4_gps.eph = 1.0f;
        px4_gps.epv = 1.0f;
        px4_gps.hdop = 1.0f;
        px4_gps.vdop = 1.0f;

        // Publish to PX4
        px4_gps_pub_->publish(px4_gps);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "GPS: lat=%.6f, lon=%.6f, alt=%.2f", 
            msg->latitude, msg->longitude, msg->altitude);
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Publisher<px4_msgs::msg::SensorGps>::SharedPtr px4_gps_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSBridge>());
    rclcpp::shutdown();
    return 0;
}