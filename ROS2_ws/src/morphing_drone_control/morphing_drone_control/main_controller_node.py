import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Float32MultiArray

from .guidance_manager import GuidanceManager
from .mode_controller import ModeController
from .motor_controller import MotorController
from .kalman_filter import KalmanFilter
from .fault_detection import FaultDetection
from .drone_model import DroneModel

import numpy as np

class DroneState:
    def __init__(self):
        self.x_hat = 0.0
        self.y_hat = 0.0
        self.z_hat = 0.0

        self.x_dot_hat = 0.0
        self.y_dot_hat = 0.0
        self.z_dot_hat = 0.0
        
        self.x_ddot_hat = 0.0
        self.y_ddot_hat = 0.0
        self.z_ddot_hat = 0.0

        self.phi_hat   = 0.0
        self.theta_hat = 0.0
        self.psi_hat   = 0.0

        self.phi_dot_hat   = 0.0
        self.theta_dot_hat = 0.0
        self.psi_dot_hat   = 0.0
        
        self.phi_ddot_hat   = 0.0
        self.theta_ddot_hat = 0.0
        self.psi_ddot_hat   = 0.0

        self.alpha = 0.0
        self.beta  = 0.0
        
        self.alpha_dot = 0.0
        self.beta_dot  = 0.0
        self.w_d = np.array([
            [0,0,0,0]
        ]).T

        self.mode = 'X' 
        
        # 센서


class MorphingDroneController(Node):
    def __init__(self):
        super().__init__('morphing_drone_controller')  # ROS 2 노드 초기화
        
        # 1) 파라미터 선언 -- 수정필요
        param_defaults = {
            'bodyMass':      1.00,
            'armMass':       0.10,
            'armcmLength':   0.05,
            'armLength':     0.15,
            'bodyLength':    0.10,
            'Ixxa':          0.002,
            'Iyya':          0.002,
            'Izza':          0.004,
            'Ixza':          0.0001,
            'Ixxb':          0.005,
            'Iyyb':          0.005,
            'Izzb':          0.008,
            'ThrustCoeff':   1e-5,
            'DragCoeff':     1e-6
        }
        for name, default in param_defaults.items():
            self.declare_parameter(name, default)
        params = {name: self.get_parameter(name).value for name in param_defaults}

        # 2) 상태, 필터 등 class 초기화
        self.drone_model = DroneModel(params)
        self.state = DroneState() 
        self.kf = KalmanFilter()
        
        self.guidance = GuidanceManager(self.state)
        self.fault_detection = FaultDetection(self.state)
        self.mode_controller = ModeController(self.state,self.guidance,self.drone_model,self.fault_detection)
        
        self.motor_controller = MotorController(self, self.state)

        # 3) 센서 데이터 저장 변수
        self.imu_data = None
        self.gps_data = None
        self.mag_data = None

        # 4) 센서 구독
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.create_subscription(MagneticField, '/magnetometer/data', self.mag_callback, 10)

        # 주기적인 제어 루프 실행 (10ms마다 실행) -- gazebo에 맞춰 수정 필요
        self.timer = self.create_timer(0.01, self.control_loop)
        

    def imu_callback(self, msg: Imu):
        self.imu_data = msg

    def gps_callback(self, msg: NavSatFix):
        self.gps_data = msg

    def mag_callback(self, msg: MagneticField):
        self.mag_data = msg

    def control_loop(self):
        # 모든 센서 데이터가 준비되었는지 확인
        if self.imu_data is None or self.gps_data is None or self.mag_data is None:
            return
        
        # 1) 동역학 파라미터 업데이트 -- 실제 식에 맞게 수정 필요
        # DroneState 클래스에서 w_d를 numpy (4,1) 형태로 저장해야 함
        self.drone_model.update(
            state=self.state,
            w_m=self.state.w_d,
            beta=self.state.beta,
            alpha=self.state.alpha
        )
        self.kf.m_t    = self.drone_model.m_t
        self.kf.F_ab   = self.drone_model.F_ab
        self.kf.Tau_ab = self.drone_model.Tau_ab
        self.kf.I_tot  = self.drone_model.I_tot
        self.kf.w_m    = self.drone_model.w_m
        
        # 2) Kalman Filter 추청 및 현재 상태에 반영
        # 예측, 갱신
        self.kf.predict()
        self.kf.update(self.imu_data, self.gps_data, self.mag_data)
        # state에 반영
        est = self.kf.x_est  # 18×1 추정 상태 벡터
        
        self.state.x_hat = est[0]
        self.state.y_hat = est[1]
        self.state.z_hat = est[2]
        self.state.x_dot_hat = est[3]
        self.state.y_dot_hat = est[4]
        self.state.z_dot_hat = est[5]
        self.state.x_ddot_hat = est[6]
        self.state.y_ddot_hat = est[7]
        self.state.z_ddot_hat = est[8]
        self.state.phi_hat = est[9]
        self.state.theta_hat = est[10]
        self.state.psi_hat = est[11]
        self.state.phi_dot_hat = est[12]
        self.state.theta_dot_hat = est[13]
        self.state.psi_dot_hat = est[14]
        self.state.phi_ddot_hat = est[15]
        self.state.theta_ddot_hat = est[16]
        self.state.psi_ddot_hat = est[17]
        
        # TODO: 3) Navigation - Fault Detection
        # TODO: 4) Navigation - Mode Classification 
        # TODO: 5) Guidance(Ros2 와 Controller 좌표축 감안할것)

        # 6) Controller - 제어기에서 제어 출력 계산(w_d², α̇_d, β̇_d) 및 state에 업데이트
        self.mode_controller.update_abw()

        # sub - 충돌 체크 

        # 7) 모터 명령어 생성 및 PWM 신호 전송
        self.motor_controller.send_commands(self.state.w_d, self.state.alpha, self.state.beta_dot)
        

def main(args=None):
    rclpy.init(args=args)               # ROS 2 초기화
    node = MorphingDroneController()    # 노드 객체 생성
    rclpy.spin(node)                    # 노드 실행
    node.destroy_node()
    rclpy.shutdown()                    # 종료 시 ROS 종료

if __name__ == '__main__':
    main()