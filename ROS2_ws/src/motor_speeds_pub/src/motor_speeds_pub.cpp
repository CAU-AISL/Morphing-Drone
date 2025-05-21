#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include <map>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class MotorSpeedsPub : public rclcpp::Node {
public:
  MotorSpeedsPub()
  : Node("motor_speeds_pub")
  {
    // 1) 퍼블리셔: 메인+틸트+플레어 총 12채널 속도 토픽
    pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/my_drone/motor_speeds", 10);

    // 2) 조인트 상태 구독 (현재 플레어 각도 읽기용)
    sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",
      rclcpp::SensorDataQoS().keep_last(50),
      std::bind(&MotorSpeedsPub::jointStateCb, this, std::placeholders::_1)
    );

    // 2-b) 플레어 목표 각도 토픽 구독
    sub_target_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/flare_target_angles", 10,
      std::bind(&MotorSpeedsPub::targetAnglesCb, this, std::placeholders::_1)
    );

    // 3) 플레어 초기 목표 각도 (예시)
    target_flare_angles_ = {0.0f, 0.0f, 0.0f, 0.0f};

    // 4) P, D 이득 파라미터 선언 및 초기화
    kp_ = this->declare_parameter("flare_kp", 2.0f);
    kd_ = this->declare_parameter("flare_kd", 0.5f);

    // 4-b) prev_errors_ 초기화
    prev_errors_.assign(4, 0.0f);

    // 5) 주기 타이머 (100ms)
    timer_ = this->create_wall_timer(
      100ms, std::bind(&MotorSpeedsPub::onTimer, this));
  }

private:
  // joint_states 콜백
  void jointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "[joint_cb] got %zu joints", msg->name.size());
    for (size_t i = 0; i < msg->name.size(); ++i) {
      last_positions_[ msg->name[i] ] = msg->position[i];
      float saved = last_positions_[ msg->name[i] ];
      RCLCPP_INFO(this->get_logger(),
        "  saved last_positions_[%s] = %.4f",
        msg->name[i].c_str(), saved);

      if (msg->name[i].rfind("flare", 0) == 0) {
        RCLCPP_INFO(this->get_logger(),
          "    flare angle: %s = %.3f", 
          msg->name[i].c_str(), msg->position[i]);
      }
    }

    RCLCPP_INFO(this->get_logger(),
      "  total stored joints: %zu", last_positions_.size());
  }

  // flare 목표 각도 콜백
  void targetAnglesCb(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 4) {
      target_flare_angles_ = { msg->data[0], msg->data[1], msg->data[2], msg->data[3] };
      RCLCPP_INFO(this->get_logger(),
        "New flare targets: [%.2f, %.2f, %.2f, %.2f]",
        target_flare_angles_[0], target_flare_angles_[1],
        target_flare_angles_[2], target_flare_angles_[3]);
    } else {
      RCLCPP_WARN(this->get_logger(),
        "Received incomplete flare target angles (size=%zu)", msg->data.size());
    }
  }

  void onTimer() {
    // — 메인 프로펠러 4개 속도
    const std::array<float,4> main_speeds = {450.0f, 0.0f, 0.0f, 450.0f};
    // — 틸트 모터 4개 속도
    const std::array<float,4> tilt_speeds = {0.0f, 0.0f, 0.0f, 0.0f};

    // — 플레어 PD 제어 속도 산출
    std::vector<float> flare_speeds(4, 0.0f);
    static const std::vector<std::string> flare_joints = {
      "flare1_link_joint",
      "flare2_link_joint",
      "flare3_link_joint",
      "flare4_link_joint"
    };
    const float dt = 0.1f;  // 타이머 주기(100ms)

    for (size_t i = 0; i < 4; ++i) {
      float cur = 0.0f;
      auto it = last_positions_.find(flare_joints[i]);
      if (it != last_positions_.end()) {
        cur = it->second;
      }

      float err  = target_flare_angles_[i] - cur;
      float derr = (err - prev_errors_[i]) / dt;

      float cmd = kp_ * err + kd_ * derr;
      const float vmax = 10.0f;
      if      (cmd >  vmax) cmd =  vmax;
      else if (cmd < -vmax) cmd = -vmax;

      flare_speeds[i]   = cmd;
      prev_errors_[i]   = err;
    }

    // — 전체 12채널에 담아서 publish
    std_msgs::msg::Float32MultiArray msg;
    msg.data.resize(12);
    for (int i = 0; i < 4; ++i) msg.data[i]       = main_speeds[i];
    for (int i = 0; i < 4; ++i) msg.data[4 + i]   = flare_speeds[i];
    for (int i = 0; i < 4; ++i) msg.data[8 + i]   = tilt_speeds[i];

    pub_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_target_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::map<std::string, float> last_positions_;
  std::vector<float> target_flare_angles_;

  float kp_;
  float kd_;
  std::vector<float> prev_errors_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorSpeedsPub>());
  rclcpp::shutdown();
  return 0;
}
