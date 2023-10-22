#include "rclcpp/rclcpp.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <memory>

using GetDirection = robot_patrol::srv::GetDirection;
using Laser = sensor_msgs::msg::LaserScan;
using std::placeholders::_1;
using std::placeholders::_2;

class ServerNode : public rclcpp::Node {
public:
  ServerNode() : Node("direction_service_node") {

    srv_ = create_service<GetDirection>(
        "direction_service",
        std::bind(&ServerNode::moving_callback, this, _1, _2));
  }

private:
  rclcpp::Service<GetDirection>::SharedPtr srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<GetDirection::Request> request_;
  float collisionThreshold = 0.4;
  int average_ranges = 0;

  int angle_range = 60;

  int left_angle_index = 240;
  int forward_angle_index = 360;
  int right_angle_index = 480;

  void moving_callback(const std::shared_ptr<GetDirection::Request> request,
                       const std::shared_ptr<GetDirection::Response> response) {
    if (request == nullptr || request->laser_data.ranges.empty()) {
      response->direction = "error";
      return;
    }

    request_ = request;

    if (calculateCollisionOnAngle(forward_angle_index, angle_range)) {
      response->direction = "forward";
      return;
    }
    float total_dist_sec_front = average_ranges;

    if (calculateCollisionOnAngle(forward_angle_index, angle_range)) {
      response->direction = "left";
      return;
    }
    float total_dist_sec_left = average_ranges;

    if (calculateCollisionOnAngle(right_angle_index, angle_range)) {
      response->direction = "right";
      return;
    }
    float total_dist_sec_right = average_ranges;

    response->direction = "forward";
    if (total_dist_sec_left < total_dist_sec_front) {
      response->direction = "left";
    } else if (total_dist_sec_right < total_dist_sec_left) {
      response->direction = "right";
    }
  }

  bool calculateCollisionOnAngle(int angleIndex, int range) {
    average_ranges = 0;
    for (int y = angleIndex - range; y < angleIndex + range; y++) {
      if (((request_->laser_data.ranges[y] < collisionThreshold &&
            request_->laser_data.ranges[y] > request_->laser_data.range_min))) {
        return true;
      }
      average_ranges = average_ranges + request_->laser_data.ranges[y];
    }
    average_ranges = average_ranges / (range * 2);
    return false;
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServerNode>());
  rclcpp::shutdown();
  return 0;
}