#include "rclcpp/rclcpp.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <future>

using GetDirection = robot_patrol::srv::GetDirection;
using std::placeholders::_1;
using namespace std::chrono_literals;

class TestDirectionService : public rclcpp::Node {
public:
  TestDirectionService() : Node("test_direction_service") {
    timer_ = this->create_wall_timer(
        100ms, std::bind(&TestDirectionService::init_node_callback, this));
  }

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
        std::bind(&TestDirectionService::scan_callback, this, _1));
    timer_->cancel();
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Store the laser scan data using a shared pointer
    laser_scan_ = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);

    if (laser_scan_ != nullptr && !laser_scan_->ranges.empty()) {
      auto request = std::make_shared<GetDirection::Request>();
      request->laser_data = *msg;
      auto result_future = client->async_send_request(
          request, std::bind(&TestDirectionService::response_callback, this,
                             std::placeholders::_1));
    }
  }

  void
  response_callback(rclcpp::Client<GetDirection>::SharedFuture result_future) {
    auto status = result_future.wait_for(1s);
    if (status == std::future_status::ready) {
      auto result = result_future.get();
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service returned: %s",
                  result->direction.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
    rclcpp::shutdown();
  }

  // Laser data subscriber
  rclcpp::Client<GetDirection>::SharedPtr client;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  std::shared_ptr<sensor_msgs::msg::LaserScan> laser_scan_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestDirectionService>());
  rclcpp::shutdown();
  return 0;
}
