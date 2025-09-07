#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

class SystemClockPublisher : public rclcpp::Node
{
public:
  SystemClockPublisher()
  : Node("system_clock_publisher")
  {
    using namespace std::chrono_literals;

    // Create publisher
    clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::QoS(10));

    // Create timer that triggers every 100ms
    timer_ = this->create_wall_timer(100ms, [this]() {
      rosgraph_msgs::msg::Clock clock_msg;
      clock_msg.clock = this->now();  // Uses system time
      clock_pub_->publish(clock_msg);
    });

    RCLCPP_INFO(this->get_logger(), "SystemClockPublisher node started.");
  }

private:
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SystemClockPublisher>());
  rclcpp::shutdown();
  return 0;
}
