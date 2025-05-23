// motor_speeds_pub.cpp
#include <chrono>
#include <memory>
#include <vector>
#include <map>
#include <array>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class MotorSpeedsPub : public rclcpp::Node {
public:
  MotorSpeedsPub() : Node("motor_speeds_pub") {
    pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      "/my_drone/motor_speeds", 10);
    sub_cmd_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/motor_wab", 10,
      std::bind(&MotorSpeedsPub::cmdCallback, this, std::placeholders::_1));
    sub_js_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS().keep_last(50),
      std::bind(&MotorSpeedsPub::jointStateCb, this, std::placeholders::_1));
    kp_ = declare_parameter("flare_kp", 2.0f);
    kd_ = declare_parameter("flare_kd", 0.5f);
    target_flare_.fill(0.0f);
    prev_err_.fill(0.0f);
    timer_ = create_wall_timer(100ms, std::bind(&MotorSpeedsPub::onTimer, this));
  }

private:
  void cmdCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    cmd_speeds_ = msg->data;
  }
  void jointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg) {
    for (size_t i = 0; i < msg->name.size(); ++i)
      last_pos_[msg->name[i]] = msg->position[i];
  }
  void onTimer() {
    std::array<float,4> main_s, flare_s, tilt_s;
    // 실제 명령이 12채널 들어오면 적용
    if (cmd_speeds_.size() >= 13) {
      for (int i = 0; i < 4; ++i) main_s[i] = cmd_speeds_[i];
      for (int i = 0; i < 4; ++i) target_flare_[i] = cmd_speeds_[4 + i];
      for (int i = 0; i < 4; ++i) tilt_s[i] = cmd_speeds_[8 + i];
    } else {
      // 디폴트 실험용 값
      main_s      = {400.0f, -400.0f, 400.0f, -400.0f};
      target_flare_ = {0.0f, 0.0f, 0.0f, 0.0f};
      tilt_s      = {0.0f, 0.0f, 0.0f, 0.0f};
    }

    // flare PD 제어
    const float dt = 0.1f;
    static const std::array<std::string,4> fj = {
      "flare1_link_joint","flare2_link_joint",
      "flare3_link_joint","flare4_link_joint"
    };
    for (int i = 0; i < 4; ++i) {
      float cur  = last_pos_[fj[i]];
      float err  = target_flare_[i] - cur;
      float derr = (err - prev_err_[i]) / dt;
      float u    = kp_ * err + kd_ * derr;
      flare_s[i] = std::clamp(u, -10.0f, 10.0f);
      prev_err_[i] = err;
    }

    std_msgs::msg::Float32MultiArray out;
    out.data.resize(12);
    for (int i = 0; i < 4; ++i) out.data[i]       = main_s[i];
    for (int i = 0; i < 4; ++i) out.data[4 + i]   = flare_s[i];
    for (int i = 0; i < 4; ++i) out.data[8 + i]   = tilt_s[i];
    pub_->publish(out);
  }

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_cmd_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr       sub_js_;
  rclcpp::TimerBase::SharedPtr                                       timer_;

  std::vector<float>               cmd_speeds_;
  std::map<std::string,float>      last_pos_;
  std::array<float,4>              target_flare_, prev_err_;
  float                             kp_, kd_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorSpeedsPub>());
  rclcpp::shutdown();
  return 0;
}