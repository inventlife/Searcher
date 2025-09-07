#ifndef SEARCHER_ROS2_HARDWARE__SEARCHER_SYSTEM_HPP_
#define SEARCHER_ROS2_HARDWARE__SEARCHER_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <array>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include <sensor_msgs/msg/joint_state.hpp>

namespace searcher_ros2_hardware
{

class SearcherSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SearcherSystemHardware)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
  // ROS node and executor
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;

  // Publisher and subscriber for joint communication
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // Joint data: position, command, and last published values
  std::array<double, 4> joint_positions_{};
  std::array<double, 4> joint_commands_{};
  std::array<double, 4> last_published_commands_{};
};

}  // namespace searcher_ros2_hardware

#endif  // SEARCHER_ROS2_HARDWARE__SEARCHER_SYSTEM_HPP_
