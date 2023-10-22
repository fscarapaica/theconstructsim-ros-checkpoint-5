#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <cmath>
#include <cstddef>

using GetDirection = robot_patrol::srv::GetDirection;
using namespace std::chrono_literals;
using std::placeholders::_1;

class RobotPatrol : public rclcpp::Node {
public:
  RobotPatrol() : Node("topics_project") { init_node_callback(); }

  std::string direction = "forward";
  float collisionThreshold = 0.4;

private:
  void init_node_callback() {
    client = this->create_client<GetDirection>("direction_service");
    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                  "service not available, waiting again...");
    }

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&RobotPatrol::scan_callback, this, _1));

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    timer_ = this->create_wall_timer(
        100ms, std::bind(&RobotPatrol::timer_callback, this));
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Store the laser scan data using a shared pointer
    laser_scan_ = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);

    if (laser_scan_ != nullptr && !laser_scan_->ranges.empty()) {
      auto request = std::make_shared<GetDirection::Request>();
      request->laser_data = *msg;
      auto result_future = client->async_send_request(
          request, std::bind(&RobotPatrol::get_direction_service_callback, this,
                             std::placeholders::_1));
    }
  }

  void get_direction_service_callback(
      rclcpp::Client<GetDirection>::SharedFuture result_future) {
    auto status = result_future.wait_for(1s);
    if (status == std::future_status::ready) {
      direction = result_future.get()->direction.c_str();
    } else {
      RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
  }

  void timer_callback() {
    if (laser_scan_ != nullptr && !laser_scan_->ranges.empty()) {
      twist_msg.linear.x = 0.1;
      if (direction == "left") {
        twist_msg.angular.z = 0.5;
      } else if (direction == "right") {
        twist_msg.angular.z = -0.5;
      } else {
        twist_msg.angular.z = 0.0;
      }
      publisher_->publish(twist_msg);
    }
  }

  // cmd_vel control
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  geometry_msgs::msg::Twist twist_msg;

  // Laser data subscriber
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  std::shared_ptr<sensor_msgs::msg::LaserScan> laser_scan_;

  // Get Direction service
  rclcpp::Client<GetDirection>::SharedPtr client;
  float max_range = 0;
  float range_index = 0;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotPatrol>());
  rclcpp::shutdown();
  return 0;
}
