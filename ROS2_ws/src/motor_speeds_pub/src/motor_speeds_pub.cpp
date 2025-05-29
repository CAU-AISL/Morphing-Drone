#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include <map>
#include <array>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class MotorSpeedsPub : public rclcpp::Node {
public:
  MotorSpeedsPub()
  : Node("motor_speeds_pub")
  {
    // 1) 퍼블리셔 (Gazebo 모터 플러그인에 인가)
    pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/my_drone/motor_speeds", 10);

    // 2) 컨트롤러 명령 12채널 구독
    sub_cmd_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/motor_wab", 10,
      std::bind(&MotorSpeedsPub::cmdCallback, this, std::placeholders::_1));

    // 3) joint_states 구독 (flare 각도 읽기용)
    sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",
      rclcpp::SensorDataQoS().keep_last(50),
      std::bind(&MotorSpeedsPub::jointStateCb, this, std::placeholders::_1));
<<<<<<< Updated upstream

    // 4) PD 제어용 파라미터
    kp_ = this->declare_parameter("flare_kp", 2.0f);
    kd_ = this->declare_parameter("flare_kd", 0.5f);
    target_flare_angles_ = {0.0f, 0.0f, 0.0f, 0.0f};
    prev_errors_        = {0.0f, 0.0f, 0.0f, 0.0f};

    // 5) 주기 타이머 (100ms)
    timer_ = this->create_wall_timer(
      100ms, std::bind(&MotorSpeedsPub::onTimer, this));
=======
    kp_ = declare_parameter("flare_kp", 2.0f);
    kd_ = declare_parameter("flare_kd", 0.5f);
    target_flare_.fill(0.0f);
    prev_err_.fill(0.0f);
    timer_ = create_wall_timer(10ms, std::bind(&MotorSpeedsPub::onTimer, this));
>>>>>>> Stashed changes
  }

private:
  // 컨트롤러가 보낸 12채널 명령 콜백
  void cmdCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    cmd_speeds_ = msg->data;
  }

  // joint_states 콜백: flare 조인트 각도 저장
  void jointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg) {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      last_positions_[ msg->name[i] ] = msg->position[i];
    }
  }

  // 타이머 콜백: cmd_speeds_ → main/flare/tilt 분리 → PD 제어 → 퍼블리시
  void onTimer() {
<<<<<<< Updated upstream
    std::array<float,4> main_speeds;
    std::array<float,4> tilt_speeds;

    // 1) cmd_speeds_ 가 유효하면 반영, 아니면 디폴트
    if (cmd_speeds_.size() >= 12) {
      for (int i = 0; i < 4; ++i) {
        main_speeds[i]            = cmd_speeds_[i];
        target_flare_angles_[i]   = cmd_speeds_[4 + i];
        tilt_speeds[i]            = cmd_speeds_[8 + i];
      }
    } else {
      main_speeds          = {0.0f, 0.0f, 0.0f, 0.0f};
      target_flare_angles_ = {0.0f,   0.0f, 0.0f, 0.0f};
      tilt_speeds          = {0.0f,   0.0f, 0.0f, 0.0f};
=======
    std::array<float,4> main_s, flare_s, tilt_s;
    // 실제 명령이 12채널 들어오면 적용
    // 여기 변경해서 추력, 반토크 확인
    if (cmd_speeds_.size() >= 12) {
      for (int i = 0; i < 4; ++i) main_s[i] = cmd_speeds_[i];
      for (int i = 0; i < 4; ++i) target_flare_[i] = cmd_speeds_[4 + i];
      for (int i = 0; i < 4; ++i) tilt_s[i] = cmd_speeds_[8 + i];
    } else {
      // 디폴트 실험용 값
      main_s      = {-330.0f, 330.0f, -330.0f, 330.0f};
      target_flare_ = {0.0f, 0.0f, 0.0f, 0.0f};
      tilt_s      = {0.0f, 0.0f, 0.0f, 0.0f};
>>>>>>> Stashed changes
    }

    // 2) flare PD 제어
    std::vector<float> flare_speeds(4, 0.0f);
    static const std::vector<std::string> flare_joints = {
      "flare1_link_joint","flare2_link_joint",
      "flare3_link_joint","flare4_link_joint"
    };
    const float dt = 0.1f; // 100ms

    for (int i = 0; i < 4; ++i) {
      float current = 0.0f;
      auto it = last_positions_.find(flare_joints[i]);
      if (it != last_positions_.end()) current = it->second;

      float err  = target_flare_angles_[i] - current;
      float derr = (err - prev_errors_[i]) / dt;
      float cmd  = kp_ * err + kd_ * derr;
      cmd = std::clamp(cmd, -10.0f, 10.0f);

      flare_speeds[i] = cmd;
      prev_errors_[i] = err;
    }

    // 3) 12채널 메시지 생성 & 퍼블리시
    std_msgs::msg::Float32MultiArray out;
    out.data.resize(12);
    for (int i = 0; i < 4; ++i) out.data[i]       = main_speeds[i];
    for (int i = 0; i < 4; ++i) out.data[4 + i]   = flare_speeds[i];
    for (int i = 0; i < 4; ++i) out.data[8 + i]   = tilt_speeds[i];

    pub_->publish(out);
  }

  // --- 멤버 변수 ---
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr           pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr        sub_cmd_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr            sub_js_;
  rclcpp::TimerBase::SharedPtr                                             timer_;

  std::vector<float>          cmd_speeds_;
  std::map<std::string,float> last_positions_;
  std::array<float,4>         target_flare_angles_;
  std::vector<float>          prev_errors_;
  float                       kp_, kd_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorSpeedsPub>());
  rclcpp::shutdown();
  return 0;
}
