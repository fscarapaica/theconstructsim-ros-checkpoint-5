#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

class RobotPatrol : public rclcpp::Node {
public:
  RobotPatrol() : Node("topics_project") {
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS(), std::bind(&RobotPatrol::scan_callback, this, _1));

    timer_ = this->create_wall_timer(1000ms, std::bind(&RobotPatrol::timer_callback, this));
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

  }

  void timer_callback() {

  }

  // cmd_vel control
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  geometry_msgs::msg::Twist twist_msg;

  // Laser data subscriber
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotPatrol>());
  rclcpp::shutdown();
  return 0;
}