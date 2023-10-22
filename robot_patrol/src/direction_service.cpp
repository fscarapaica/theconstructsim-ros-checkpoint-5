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
  float lateral_collision_threshold = 0.3;
  float front_collision_threshold = 0.4;
  float average_ranges = 0;

  int angle_range = 60;

  int left_angle_index = 480;
  int forward_angle_index = 360;
  int right_angle_index = 240;

  void moving_callback(const std::shared_ptr<GetDirection::Request> request,
                       const std::shared_ptr<GetDirection::Response> response) {
    if (request == nullptr || request->laser_data.ranges.empty()) {
      response->direction = "error";
      return;
    }

    request_ = request;
    response->direction = "forward";

    if (!isThereCollisionOnAngle(forward_angle_index, angle_range, front_collision_threshold)) {
      response->direction = "forward";
      return;
    }
    float total_dist_sec_front = average_ranges;

    bool isThereCollisionOnAngleRight =
        isThereCollisionOnAngle(right_angle_index, angle_range, lateral_collision_threshold);
    float total_dist_sec_right = average_ranges;

    bool isThereCollisionOnAngleLeft =
        isThereCollisionOnAngle(left_angle_index, angle_range, lateral_collision_threshold);
    float total_dist_sec_left = average_ranges;

    if (!isThereCollisionOnAngleLeft && !isThereCollisionOnAngleRight) {
      if (total_dist_sec_right > total_dist_sec_left) {
        response->direction = "right";
      } else {
        response->direction = "left";
      }
      return;
    } else if (!isThereCollisionOnAngleRight) {
      response->direction = "right";
      return;
    } else if (!isThereCollisionOnAngleLeft) {
      response->direction = "left";
      return;
    }

    if (total_dist_sec_left > total_dist_sec_front &&
        total_dist_sec_left > total_dist_sec_right) {
      response->direction = "left";
    } else if (total_dist_sec_right > total_dist_sec_front &&
               total_dist_sec_right > total_dist_sec_left) {
      response->direction = "right";
    }
  }

  bool isThereCollisionOnAngle(int angleIndex, int range, float collisionThreshold) {
    average_ranges = 0;
    bool isThereCollisionOnAngle = false;
    for (int y = angleIndex - range; y < angleIndex + range; y++) {
      if (((request_->laser_data.ranges[y] < collisionThreshold &&
            request_->laser_data.ranges[y] > request_->laser_data.range_min))) {
        isThereCollisionOnAngle = true;
      }
      average_ranges = average_ranges + request_->laser_data.ranges[y];
    }
    average_ranges = average_ranges / (range * 2);
    return isThereCollisionOnAngle;
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServerNode>());
  rclcpp::shutdown();
  return 0;
}