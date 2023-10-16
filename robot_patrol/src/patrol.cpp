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
  float collisionThreshold = 0.4;

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Store the laser scan data using a shared pointer
    laser_scan_ = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);

    if (laser_scan_ != nullptr && !laser_scan_->ranges.empty()) {
      if (calculateCollisionOnAngle(360, 60)) {
        angle = calculateSafeAreaAngle();
      } else {
        angle = 0;
      }
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
    int startLaserReadIndex = 180;
    int endLaserReadIndex = 540;

    max_range = laser_scan_->range_min;
    range_index = -1;

    // Find the index of the minimum range in the laser scan data
    for (int i = 0; i <= 180; i++) {
      calculateAngleIndex(startLaserReadIndex + i);
      calculateAngleIndex(endLaserReadIndex - i);
    }

    // No good read
    if (range_index == -1)
      return 0;

    return laser_scan_->angle_min + range_index * laser_scan_->angle_increment;
  }

  void calculateAngleIndex(int i) {
    if (laser_scan_->ranges[i] < laser_scan_->range_max &&
        laser_scan_->ranges[i] >= max_range &&
        !std::isinf(laser_scan_->ranges[i])) {

      // Check greater range obstacle direction
      if (calculateCollisionOnAngle(i, 60)) {
        return;
      }

      max_range = laser_scan_->ranges[i];
      if (range_index >= 360) {
        range_index = i + 30;
      } else {
        range_index = i - 30;
      }
    }
  }

  bool calculateCollisionOnAngle(int angleIndex, int range) {
    for (int y = angleIndex - range; y <= angleIndex + range; y++) {
      if (((laser_scan_->ranges[y] < collisionThreshold &&
            laser_scan_->ranges[y] > laser_scan_->range_min))) {
        return true;
      }
    }
    return false;
  }

  // cmd_vel control
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  geometry_msgs::msg::Twist twist_msg;

  // Laser data subscriber
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  std::shared_ptr<sensor_msgs::msg::LaserScan> laser_scan_;

  float max_range = 0;
  float range_index = 0;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotPatrol>());
  rclcpp::shutdown();
  return 0;
}
