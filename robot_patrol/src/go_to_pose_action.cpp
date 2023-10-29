#include <cmath>
#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "robot_patrol/action/go_to_pose.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

const double TWO_PI = 2.0 * M_PI;

class GoToPose : public rclcpp::Node {
public:
  using GoToPoseAction = robot_patrol::action::GoToPose;
  using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPoseAction>;

  explicit GoToPose(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("go_to_pose_server_node", options) {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<GoToPoseAction>(
        this, "go_to_pose", std::bind(&GoToPose::handle_goal, this, _1, _2),
        std::bind(&GoToPose::handle_cancel, this, _1),
        std::bind(&GoToPose::handle_accepted, this, _1));

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", rclcpp::SensorDataQoS(),
        std::bind(&GoToPose::odom_callback, this, _1));
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Extract the quaternion
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    // Convert the quaternion to Euler angles
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    current_pos.set__x(msg->pose.pose.position.x);
    current_pos.set__y(msg->pose.pose.position.y);
    current_pos.set__theta(yaw);
  }

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const GoToPoseAction::Goal> goal) {
    (void)uuid;
    destination_pos_ = goal->goal_pos;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    // RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spinup a
    // new thread
    std::thread{std::bind(&GoToPose::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting service going to pose");
    auto feedback = std::make_shared<GoToPoseAction::Feedback>();

    // message = "Starting movement...";
    auto result = std::make_shared<GoToPoseAction::Result>();
    auto twist = geometry_msgs::msg::Twist();

    float destinationAngle = calculateDestinationDirectionAngle();
    destinationPositionYawNormalize = yawRadNormalize(destinationAngle);

    rclcpp::Rate loop_rate(5);
    bool goalReached = false;
    while (rclcpp::ok() && !goalReached) {
      currentPositionYawNormalize = yawRadNormalize(current_pos.theta);
      // we need to rotate more
      calculateRemainingAngleToTarget();
      goalReached = areAlmostEqual(rotation, 0, 2);
      if (goalReached) {
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.angular.z = 0;
      } else {
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.angular.z = rotation;
      }
      publisher_->publish(twist);
      feedback->current_pos = current_pos;
      goal_handle->publish_feedback(feedback);
      loop_rate.sleep();
    }

    // Move to point
    goalReached = false;
    while (rclcpp::ok() && !goalReached) {
      currentPositionYawNormalize = yawRadNormalize(current_pos.theta);
      calculateRemainingAngleToTarget();
      goalReached = areAlmostEqual(distanceBetweenPose(), 0, 5);

      if (goalReached) {
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.angular.z = 0;
      } else {
        twist.linear.x = 0.1;
        twist.linear.y = 0;
        twist.angular.z = rotation;
      }
      publisher_->publish(twist);
      feedback->current_pos = current_pos;
      goal_handle->publish_feedback(feedback);
      loop_rate.sleep();
    }

    // Check last rotation
    float destinationRadAngle = destination_pos_.theta * (M_PI / 180.0);
    destinationPositionYawNormalize = yawRadNormalize(destinationRadAngle);
    goalReached = false;
    while (rclcpp::ok() && !goalReached) {
      currentPositionYawNormalize = yawRadNormalize(current_pos.theta);

      // we need to rotate more
      calculateRemainingAngleToTarget();
      goalReached = areAlmostEqual(rotation, 0, 1);

      if (goalReached) {
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.angular.z = 0;
      } else {
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.angular.z = rotation;
      }
      publisher_->publish(twist);
      feedback->current_pos = current_pos;
      goal_handle->publish_feedback(feedback);
      loop_rate.sleep();
    }

    result->status = true;
    goal_handle->succeed(result);
  }

  void calculateRemainingAngleToTarget() {
    double clockwise_diff = fmod((destinationPositionYawNormalize -
                                  currentPositionYawNormalize + TWO_PI),
                                 TWO_PI);
    double counter_clockwise_diff =
        fmod((currentPositionYawNormalize - destinationPositionYawNormalize +
              TWO_PI),
             TWO_PI);

    if (clockwise_diff < counter_clockwise_diff) {
      rotation = clockwise_diff;
    } else {
      rotation = -counter_clockwise_diff;
    }
  }

  double calculateDestinationDirectionAngle() {
    return atan2(destination_pos_.y - current_pos.y,
                                destination_pos_.x - current_pos.x);
  }

  float yawRadNormalize(float yawNormalize) {
    yawNormalize = fmod(yawNormalize, TWO_PI);
    if (yawNormalize < 0)
      yawNormalize += TWO_PI;
    return yawNormalize;
  }

  bool areAlmostEqual(float a, float b, float margin) {
    int a_int = static_cast<int>(round(a * 100));
    int b_int = static_cast<int>(round(b * 100));

    return b_int - margin <= a_int && a_int <= b_int + margin;
  }

  double distanceBetweenPose() {
    double dx = current_pos.x - destination_pos_.x;
    double dy = current_pos.y - destination_pos_.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  rclcpp_action::Server<GoToPoseAction>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;

  geometry_msgs::msg::Pose2D current_pos;
  geometry_msgs::msg::Pose2D destination_pos_;

  float currentPositionYawNormalize = 0;
  float destinationPositionYawNormalize = 0;
  float rotation = 0;
}; // class GoToPose

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<GoToPose>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}