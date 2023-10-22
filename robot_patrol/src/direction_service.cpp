#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "robot_patrol/srv/get_direction.hpp"

#include <chrono>
#include <memory>

using GetDirection = robot_patrol::srv::GetDirection;
using std::placeholders::_1;
using std::placeholders::_2;

class ServerNode : public rclcpp::Node {
public:
  ServerNode() : Node("get_direction_service") {

    srv_ = create_service<GetDirection>(
        "direction_service", std::bind(&ServerNode::moving_callback, this, _1, _2));
  }

private:
  rclcpp::Service<GetDirection>::SharedPtr srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  void moving_callback(const std::shared_ptr<GetDirection::Request> request,
                       const std::shared_ptr<GetDirection::Response> response) {

  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServerNode>());
  rclcpp::shutdown();
  return 0;
}