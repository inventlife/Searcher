#include <memory>
#include <cmath>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

class ServoTrajectoryActionServer : public rclcpp::Node
{
public:
  ServoTrajectoryActionServer()
  : Node("custom_servo_trajectory_server"),
    confirmation_received_(false)
  {
    using namespace std::placeholders;

    action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
      this,
      "/custom_trajectory_controller",
      std::bind(&ServoTrajectoryActionServer::handle_goal, this, _1, _2),
      std::bind(&ServoTrajectoryActionServer::handle_cancel, this, _1),
      std::bind(&ServoTrajectoryActionServer::handle_accepted, this, _1)
    );

    goal_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/searcher_controller/goal_position", 10);

    confirmation_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/servo_goal_reached", 10,
      std::bind(&ServoTrajectoryActionServer::on_confirmation_received, this, _1));

    RCLCPP_INFO(this->get_logger(), "Custom servo trajectory action server is ready.");
  }

private:
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr goal_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr confirmation_sub_;

  bool confirmation_received_;
  float expected_goal_ = 0.0f;

  using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;
  std::shared_ptr<GoalHandle> active_goal_;

  rclcpp_action::GoalUUID current_goal_id_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received trajectory goal.");
    current_goal_id_ = uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    RCLCPP_INFO(this->get_logger(), "Cancel request received.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    active_goal_ = goal_handle;
    std::thread([this, goal_handle]() {
      this->execute(goal_handle);
    }).detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing trajectory...");

    const auto & trajectory = goal_handle->get_goal()->trajectory;

    if (trajectory.points.empty() || trajectory.joint_names.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Trajectory is empty.");
      auto result = std::make_shared<FollowJointTrajectory::Result>();
      result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
      goal_handle->abort(result);
      return;
    }

    expected_goal_ = static_cast<float>(trajectory.points.back().positions[0]);
    confirmation_received_ = false;

    std_msgs::msg::Float32 goal_msg;
    goal_msg.data = expected_goal_;
    goal_pub_->publish(goal_msg);

    RCLCPP_INFO(this->get_logger(), "Published goal: %.3f", expected_goal_);

    // Wait for confirmation with timeout
    rclcpp::Time start_time = this->now();
    rclcpp::Duration timeout = rclcpp::Duration(5s);

    while (!confirmation_received_) {
      if (this->now() - start_time > timeout) {
        RCLCPP_WARN(this->get_logger(), "Timeout waiting for servo confirmation.");
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
        goal_handle->abort(result);
        return;
      }
      rclcpp::sleep_for(100ms);
    }

    RCLCPP_INFO(this->get_logger(), "Servo goal confirmed.");
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
    goal_handle->succeed(result);
  }

  void on_confirmation_received(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      confirmation_received_ = true;
      RCLCPP_INFO(this->get_logger(), "Confirmation received for goal: %.3f", expected_goal_);
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ServoTrajectoryActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
