#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

class MotorSpeedsPub : public rclcpp::Node {
public:
  MotorSpeedsPub()
  : Node("motor_speeds_pub_cpp")
  {
    pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/my_drone/motor_speeds", 10);
    timer_ = this->create_wall_timer(
      100ms, std::bind(&MotorSpeedsPub::on_timer, this));
  }

private:
  void on_timer() {
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data = {400.0f, 400.0f, 400.0f, 400.0f};
    pub_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorSpeedsPub>());
  rclcpp::shutdown();
  return 0;
}