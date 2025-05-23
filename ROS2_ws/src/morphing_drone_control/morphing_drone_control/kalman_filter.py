import numpy as np
from .rpy2rot import rpy2rot
from .rpy2rot_derivative import dRpy2rot_dphi, dRpy2rot_dtheta, dRpy2rot_dpsi
def skew(v):
    return np.array([[    0, -v[2],  v[1]],
                     [ v[2],     0, -v[0]],
                     [-v[1],  v[0],     0]])
class KalmanFilter:
    def __init__(self, drone_model, state, dt: float = 0.01):
        self.dt = dt
        # EKF 상태 벡터, 공분산, 노이즈
        self.x_est = np.zeros((18,1))
        self.P     = np.eye(18)*1e-4
        self.Q     = np.eye(18)*1e-8
        # 측정 노이즈
        sigma_acc  = 0.1
        sigma_gyro = 0.001
        sigma_gps  = 0.003
        sigma_mag  = 0.003
        R_gps  = sigma_gps**2 * np.eye(3)
        R_mag  = sigma_mag**2 * np.eye(1)
        R_gyro = sigma_gyro**2 * np.eye(3)
        self.R_imu = np.block([
            [R_gps,           np.zeros((3,1)), np.zeros((3,3))],
            [np.zeros((1,3)), R_mag,           np.zeros((1,3))],
            [np.zeros((3,3)), np.zeros((3,1)), R_gyro         ]
        ])
        # # 동역학 파라미터 (외부에서 설정 필요)  18-state estimation 에서는 feedback linearization 에 사용된 모델 이용
        # self.m_t = None      # 총 질량
        # self.F_ab = None     # 3×4 힘 매핑 행렬
        # self.Tau_ab = None   # 3×4 토크 매핑 행렬
        # self.I_tot = None    # 3×3 관성 모멘트 행렬
        # self.w_m = None      # 입력 모터 속도 벡터 (4×1)
        self.drone_model = drone_model
        self.state = state


    def euler_acc(self, imu_msg, u: np.ndarray = None):  ## create roll,pitch reading from accelerometer
        acc = np.array([[imu_msg.linear_acceleration.x],
                        [-imu_msg.linear_acceleration.y],
                        [-imu_msg.linear_acceleration.z]])
        acc_by_grav = acc - (1/self.drone_model.m_t)*self.drone_model.F_ab @ self.state.w_d
        phi_acc = np.arctan2(acc_by_grav[1], acc_by_grav[2])
        theta_acc = np.arctan2(
            -acc_by_grav[0],
            np.sqrt(acc_by_grav[1]**2 + acc_by_grav[2]**2)
        )
        euler_acc = np.array([phi_acc, theta_acc])
        return euler_acc
    def predict(self, v: np.ndarray = None):
        A = np.block([
            [np.eye(6), self.dt*np.eye(6), 0.5*self.dt**2*np.eye(6)],
            [np.zeros((6,6)),np.zeros((6,6)),self.dt*np.eye(6)],
            [np.zeros((6,6)), np.zeros((6,6)), np.eye(6)]
        ])
        B = np.vstack([
            (self.dt**3/6)*np.eye(6),
            (self.dt**2/2)*np.eye(6),
            self.dt*np.eye(6)
        ])
        if v is None:
            v = np.zeros((6,1))
        self.x_est = A.dot(self.x_est) + B.dot(v) ## + np.concatenate((np.zeros((5,1)), [[self.g*self.dt]], np.zeros((6,1)))) 여기도 필요 없어짐
        self.P     = A.dot(self.P).dot(A.T) + self.Q
    def update(self, imu_msg, gps_msg, mag_msg, u: np.ndarray = None):
        phi = self.state.phi_hat
        theta = self.state.theta_hat
        psi = self.state.psi_hat
        R = rpy2rot(phi,theta,psi)
        R_T = R.T
        acc = R_T @ np.array([[imu_msg.linear_acceleration.x],
                        [imu_msg.linear_acceleration.y],
                        [imu_msg.linear_acceleration.z-9.81]])
        gyro = np.array([[-imu_msg.angular_velocity.x],
                         [imu_msg.angular_velocity.y],
                         [-imu_msg.angular_velocity.z]])
        gps = np.array([[-gps_msg.latitude],
                        [gps_msg.longitude],
                        [-gps_msg.altitude]])
        mag = np.array([[-mag_msg.magnetic_field.x]])    ## 확인필요 z yaw 를 측정해야함
        euler_acc = self.euler_acc(imu_msg, u)
        z_k = np.vstack((gps, euler_acc, mag, gyro, acc))
        H = np.block({
            [np.eye(3), np.zeros((3,15))],
            [np.zeros((3,3)), np.eye(3), np.zeros((3,12))],
            [np.zeros((3,9)), np.eye(3), np.zeros((3,6))],
            [np.zeros((3,12)), np.eye(3), np.zeros((3,3))]
        })
        S = H.dot(self.P).dot(H.T) + self.R_imu
        K = self.P.dot(H.T).dot(np.linalg.inv(S))
        y = z_k - H.dot(self.x_est)
        self.x_est = self.x_est + K.dot(y)
        self.P     = (np.eye(12)-K.dot(H)).dot(self.P)
        # print("x_est", self.x_est)
    # for 18 - state estimation we don't need jacobian calculation, just use the linear model from feedback linearization
    # def _compute_jacobian(self):
    #     phi, theta, psi = self.x_est[6,0], self.x_est[7,0], self.x_est[8,0]
    #     omega = self.x_est[9:12,0]
    #     wm    = self.w_m.flatten()
    #     dt    = self.dt
    #     m_t   = self.m_t
    #     F_ab  = self.F_ab  # shape 3×4
    #     Tau_ab= self.Tau_ab
    #     I_tot = self.I_tot
    #     R = rpy2rot([phi,theta,psi]).T
    #     T = np.array([
    #         [1, np.sin(phi)*np.tan(theta),  np.cos(phi)*np.tan(theta)],
    #         [0, np.cos(phi),               -np.sin(phi)],
    #         [0, np.sin(phi)/np.cos(theta),  np.cos(phi)/np.cos(theta)]
    #     ])
    #     A = np.eye(12)
    #     B = np.zeros((12,4))
    #     # 1) 위치-속도 블록
    #     A[0:3,3:6] = dt * np.eye(3)
    #     # 2) 속도-자세 블록
    #     dR_dphi   = dRpy2rot_dphi(phi,theta,psi)
    #     dR_dtheta = dRpy2rot_dtheta(phi,theta,psi)
    #     dR_dpsi   = dRpy2rot_dpsi(phi,theta,psi)
    #     J_R = (1/m_t) * np.column_stack((dR_dphi.dot(F_ab).dot(wm),
    #                                      dR_dtheta.dot(F_ab).dot(wm),
    #                                      dR_dpsi.dot(F_ab).dot(wm)))
    #     A[3:6,6:9] = dt * J_R
    #     # 3) 자세-자세, 자세-속도 블록
    #     dT_dphi = np.array([
    #         [0, np.cos(phi)*np.tan(theta), -np.sin(phi)*np.tan(theta)],
    #         [0, -np.sin(phi),             -np.cos(phi)],
    #         [0, np.cos(phi)/np.cos(theta), -np.sin(phi)/np.cos(theta)]
    #     ])
    #     dT_dtheta = np.array([
    #         [0, np.sin(phi)*(1/np.cos(theta))**2,  np.cos(phi)*(1/np.cos(theta))**2],
    #         [0, 0,                                   0],
    #         [0, np.sin(phi)*(1/np.cos(theta))*np.tan(theta), np.cos(phi)*(1/np.cos(theta))*np.tan(theta)]
    #     ])
    #     J_T = np.column_stack((dT_dphi.dot(omega), dT_dtheta.dot(omega), np.zeros((3,1))))
    #     A[6:9,6:9]   = np.eye(3) + dt * J_T
    #     A[6:9,9:12]  = dt * T
    #     # 4) 각속도-각속도 블록
    #     Jg = -np.linalg.solve(I_tot, skew(omega).dot(I_tot) + skew(I_tot.dot(omega)))
    #     A[9:12,9:12] = np.eye(3) + dt * Jg
    #     # B blocks
    #     B[3:6,:]   = dt * (1/m_t) * R.dot(F_ab)
    #     B[9:12,:] = dt * np.linalg.solve(I_tot, Tau_ab)
    #     return A, B
"""
상태 예측: predict()에서 Jacobian 기반 예측과 중력 가속도 보정
측정 갱신: update()에서 IMU, GPS, 자기계 데이터로 칼만 이득 계산 및 상태/공분산 업데이트
측정 노이즈: MATLAB 값(sigma_acc=0.1, sigma_gyro=0.001, sigma_gps=0.003, sigma_mag=0.003) 그대로 사용
보조 IMU 융합: MATLAB 알고리즘의 두 IMU 보조 융합 구조를 단일 IMU 반복 사용으로 대체
self.x_est로부터 φ, θ, ψ, ω를 추출
수정 필요
블라블라
self.w_m, self.m_t, self.F_ab, self.Tau_ab, self.I_tot은 외부에서 설정 필요
드론 고유 모델(질량 분포, 링크 길이 등)에 맞춰 dRpy2rot_*, skew, T 행렬 정의
만약 w_m 입력 차원이 다르거나, 모터 개수가 다르면 B 행렬 크기 조정
"""