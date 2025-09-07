#include "searcher_system.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <array>

namespace searcher_ros2_hardware
{

const std::array<std::string, 4> joint_names = {
  "Shoulder", "Shoulder_Upper_Arm", "Upper_Lower_Arm", "Lower_Arm_Wrist"
};

hardware_interface::CallbackReturn SearcherSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  node_ = rclcpp::Node::make_shared("searcher_hw_interface");
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  spin_thread_ = std::thread([this]() { executor_->spin(); });

  joint_command_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
    "/searcher_controller/goal_positions", rclcpp::QoS(10));

  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/searcher_controller/controller_states/joint_states", rclcpp::QoS(10),
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      for (size_t i = 0; i < joint_names.size(); ++i) {
        for (size_t j = 0; j < msg->name.size(); ++j) {
          if (msg->name[j] == joint_names[i]) {
            joint_positions_[i] = msg->position[j];
            break;
          }
        }
      }
    });

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SearcherSystemHardware::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(node_->get_logger(), "SearcherSystemHardware configured.");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SearcherSystemHardware::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(node_->get_logger(), "SearcherSystemHardware activated.");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SearcherSystemHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(node_->get_logger(), "SearcherSystemHardware deactivated.");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type SearcherSystemHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SearcherSystemHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  sensor_msgs::msg::JointState cmd_msg;
  cmd_msg.name = {joint_names.begin(), joint_names.end()};
  cmd_msg.position.resize(joint_names.size());
  bool need_publish = false;

  for (size_t i = 0; i < joint_names.size(); ++i) {
    cmd_msg.position[i] = joint_commands_[i];
    if (std::fabs(joint_commands_[i] - last_published_commands_[i]) > 0.001) {
      need_publish = true;
      last_published_commands_[i] = joint_commands_[i];
    }
  }

  if (need_publish) {
    joint_command_pub_->publish(cmd_msg);
  }

  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface>
SearcherSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_names.size(); ++i) {
    state_interfaces.emplace_back(joint_names[i], "position", &joint_positions_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
SearcherSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> cmd_interfaces;
  for (size_t i = 0; i < joint_names.size(); ++i) {
    cmd_interfaces.emplace_back(joint_names[i], "position", &joint_commands_[i]);
  }
  return cmd_interfaces;
}

// Declare joint arrays
std::array<double, 4> joint_positions_ = {0.0, 0.0, 0.0, 0.0};
std::array<double, 4> joint_commands_ = {0.0, 0.0, 0.0, 0.0};
std::array<double, 4> last_published_commands_ = {0.0, 0.0, 0.0, 0.0};

}  // namespace searcher_ros2_hardware

PLUGINLIB_EXPORT_CLASS(searcher_ros2_hardware::SearcherSystemHardware, hardware_interface::SystemInterface)
