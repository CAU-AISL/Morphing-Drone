import rclpy
from rclpy.node import Node

# 아래 3개의 파일은 각각 유도 시스템, 제어기, 모터 명령 인터페이스를 담당하는 모듈입니다.
from .guidance_manager import GuidanceManager
from .mode_controller import ModeController
from .motor_controller import MotorController
from .kalman_filter import KalmanFilter
from .fault_detection import FaultDetection

class DroneState:
    def __init__(self):
        self.x_hat = 0.0
        self.y_hat = 0.0
        self.z_hat = 0.0

        self.x_dot_hat = 0.0
        self.y_dot_hat = 0.0
        self.z_dot_hat = 0.0

        self.phi_hat   = 0.0
        self.theta_hat = 0.0
        self.psi_hat   = 0.0

        self.phi_dot_hat   = 0.0
        self.theta_dot_hat = 0.0
        self.psi_dot_hat   = 0.0

        self.alpha = 0.0
        self.beta  = 0.0
        
        self.alpha_dot = 0.0
        self.beta_dot  = 0.0
        self.w_d = 0.0

        self.mode = 'X' 


class MorphingDroneController(Node):
    def __init__(self):
        super().__init__('morphing_drone_controller')  # ROS 2 노드 초기화

        self.guidance = GuidanceManager(self)

        self.mode_controller = ModeController(self.state)

        self.motor_controller = MotorController(self.state)
        
        self.state = DroneState() 

        # 주기적인 제어 루프 실행 (10ms마다 실행)
        self.timer = self.create_timer(0.01, self.control_loop)

    def control_loop(self):
        # 드론 상태 업데이트 (IMU 센서 데이터 읽기)
        
        # Navigation - Kalman Filter
        
        # Navigation - Fault Detection
        # Navigation - Mode Classification

        # ---------------------------이 위 까지 아직 구현 안됨 ----------------------------


        # 제어기에서 제어 출력 계산(w_d², α̇_d, β̇_d) 및 state에 업데이트
        self.mode_controller.update_abw()

        # 충돌 체크 

        # 모터 명령어 생성 및 PWM 신호 전송
        self.motor_controller.send_pwm(self.state)

def main(args=None):
    rclpy.init(args=args)               # ROS 2 초기화
    node = MorphingDroneController()    # 노드 객체 생성
    rclpy.spin(node)                    # 노드 실행
    rclpy.shutdown()                    # 종료 시 ROS 종료
