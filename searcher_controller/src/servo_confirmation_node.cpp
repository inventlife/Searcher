#include <memory>
#include <cmath>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class ServoConfirmationNode : public rclcpp::Node
{
public:
  ServoConfirmationNode()
  : Node("servo_confirmation_node"),
    latest_goal_(0.0f),
    confirmation_sent_(false)
  {
    using namespace std::placeholders;

    goal_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/searcher_controller/goal_position", 10,
      std::bind(&ServoConfirmationNode::on_goal_received, this, _1));

    position_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/servo_position_estimated", 10,
      std::bind(&ServoConfirmationNode::on_position_received, this, _1));

    goal_reached_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/servo_goal_reached", 10);

    actual_position_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/searcher_controller/controller_states/actual_position", 10);

    RCLCPP_INFO(this->get_logger(), "Servo confirmation node is ready.");
  }

private:
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr position_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr actual_position_pub_;

  float latest_goal_;
  bool confirmation_sent_;

  void on_goal_received(const std_msgs::msg::Float32::SharedPtr msg)
  {
    latest_goal_ = msg->data * (180.0f / static_cast<float>(M_PI));  // Convert radians to degrees
    confirmation_sent_ = false;
    RCLCPP_INFO(this->get_logger(), "New goal received: %.3f°", latest_goal_);
  }

  void on_position_received(const std_msgs::msg::Float32::SharedPtr msg)
  {
    float current_pos_deg = msg->data;
    float current_pos_rad = current_pos_deg * static_cast<float>(M_PI) / 180.0f;

    // Publish actual position in radians
    std_msgs::msg::Float32 actual_msg;
    actual_msg.data = current_pos_rad;
    actual_position_pub_->publish(actual_msg);

    float delta = std::abs(current_pos_deg - latest_goal_);
    const float tolerance = 1.5f;  // Acceptable error in degrees

    RCLCPP_INFO(this->get_logger(),
      "Goal: %.3f°, Position: %.3f° → Δ = %.3f° → Match: %s",
      latest_goal_, current_pos_deg, delta,
      (delta < tolerance ? "TRUE" : "FALSE"));

    if (!confirmation_sent_ && delta < tolerance) {
      std_msgs::msg::Bool confirm_msg;
      confirm_msg.data = true;
      goal_reached_pub_->publish(confirm_msg);
      confirmation_sent_ = true;

      RCLCPP_INFO(this->get_logger(), "Servo goal confirmed.");
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ServoConfirmationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
