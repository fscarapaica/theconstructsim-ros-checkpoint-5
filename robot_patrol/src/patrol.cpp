#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <cmath>
#include <cstddef>

using namespace std::chrono_literals;
using std::placeholders::_1;

class RobotPatrol : public rclcpp::Node {
public:
  RobotPatrol() : Node("topics_project") {
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&RobotPatrol::scan_callback, this, _1));

    timer_ = this->create_wall_timer(
        100ms, std::bind(&RobotPatrol::timer_callback, this));
  }

  float angle = 0;
  float collisionThreshold = 0.3;

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Store the laser scan data using a shared pointer
    laser_scan_ = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);

    if (laser_scan_ != nullptr && !laser_scan_->ranges.empty()) {
      // Call the function to print the distance to the closest obstacle
      angle = calculateSafeAreaAngle();
    }
  }

  void timer_callback() {
    if (laser_scan_ != nullptr && !laser_scan_->ranges.empty()) {
      twist_msg.linear.x = 0.1;
      twist_msg.angular.z = angle / 2;
      publisher_->publish(twist_msg);
    }
  }

  float calculateSafeAreaAngle() {
    float max_valid_range = laser_scan_->range_max;
    float min_valid_range = laser_scan_->range_min;
    int startLaserReadIndex = 186;
    int endLaserReadIndex = 546;

    float max_range = min_valid_range;
    float range_index = -1;

    // Find the index of the minimum range in the laser scan data
    for (int i = startLaserReadIndex; i < endLaserReadIndex; i++) {
      if (laser_scan_->ranges[i] <= max_valid_range &&
          laser_scan_->ranges[i] > max_range &&
          !std::isinf(laser_scan_->ranges[i])) {

        max_range = laser_scan_->ranges[i];
        range_index = i;
      }
    }

    // No good read
    if (range_index == -1)
      return 0;

    return laser_scan_->angle_min + range_index * laser_scan_->angle_increment;
  }

  // cmd_vel control
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  geometry_msgs::msg::Twist twist_msg;

  // Laser data subscriber
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  std::shared_ptr<sensor_msgs::msg::LaserScan> laser_scan_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotPatrol>());
  rclcpp::shutdown();
  return 0;
}