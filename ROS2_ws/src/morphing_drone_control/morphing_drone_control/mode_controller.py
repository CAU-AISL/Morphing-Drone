import numpy as np
from .rpy2rot import rpy2rot
from .rpy2rot_derivative import RPY2Rot_derivative

class ModeController:
    def __init__(self, state, guidance, drone_model,fault_detection):
        self.state = state
        self.guidance = guidance
        self.drone_model = drone_model
        self.fault_detection = fault_detection

    def update_abw(self):
    # 변수정의
    # State 관련
        x = self.state.x_hat
        y = self.state.y_hat
        z = self.state.z_hat
        
        x_dot = self.state.x_dot_hat
        y_dot = self.state.y_dot_hat
        z_dot = self.state.z_dot_hat
        
        x_ddot = self.state.x_ddot_hat
        y_ddot = self.state.y_ddot_hat
        z_ddot = self.state.z_ddot_hat
        
        phi = self.state.phi_hat
        theta = self.state.theta_hat
        psi = self.state.psi_hat
        
        phi_dot = self.state.phi_dot_hat
        theta_dot = self.state.theta_dot_hat
        psi_dot = self.state.psi_dot_hat
        
        phi_ddot = self.state.phi_ddot_hat
        theta_ddot = self.state.theta_ddot_hat
        psi_ddot = self.state.psi_ddot_hat
        
        # 팔 각도 관련
        alpha = self.state.alpha
        beta = self.state.beta
        
        #여기는 아직 없음 따로 추가해줘야 함
        self.drone_model.update(alpha)
        I_prev = self.drone_model.prev_I_total
        I_cur = self.drone_model.cur_I_total
        dt = self.drone_model.current_time-self.drone_model.prev_time
        
        kf = self.drone_model.kf
        km = self.drone_model.km 
        al = self.drone_model.al
        bl = self.drone_model.bl
        m_t = self.drone_model.m_t
        
        w_m = self.state.w_d**2
        
    # Guidance 관련 (update 되면 해야 함)
        x_d = self.guidance.x_d #maybe..?
        y_d = self.guidance.y_d
        z_d = self.guidance.z_d
        phi_d = self.guidance.phi_d
        theta_d = self.guidance.theta_d
        psi_d = self.guidance.psi_d
        
        if self.state.mode == 'X':
            
            #Z-domain에서 v 값 계산
            z_current = np.array([
                [x,x_dot,x_ddot,y,y_dot,y_ddot,z,z_dot,z_ddot,phi,phi_dot,phi_ddot,theta,theta_dot,theta_ddot,psi, psi_dot, psi_ddot]
            ]).T
            z_ref = np.array([
                [x_d,0,0,y_d,0,0,z_d,0,0,phi_d,0,0,theta_d,0,0,psi_d,0,0]
            ]).T
            error = z_ref-z_current
                    #이런식으로 K_lqr 계산, 미리 offline 계산
            '''
            A = np.array([
                [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            ])
            
            B = np.array([
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [1, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 1]
            ])
            
            Q = np.array([
                [10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]
            ])
            
            R = np.array([
                [1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1],
            ])
            K_lqr,S,E = control.lqr(A,B,Q,R)
            '''
            K_lqr = np.array([
                [1.00000000e+01,  9.51325204e+00,  4.47509822e+00, -2.09825237e-17,
                -4.81629836e-16, -3.26249510e-16, -9.36288349e-16,  1.24528323e-15,
                6.52280064e-16, -1.10683002e-17, -1.59522132e-17, -4.88391287e-18,
                -2.19944578e-17, -8.87434893e-17, -9.70178991e-17,  7.32124969e-17,
                1.90203171e-16,  1.81084519e-16],
                [-6.16819195e-16, -1.41889108e-15, -3.26249510e-16,  1.00000000e+01,
                9.51325204e+00,  4.47509822e+00,  1.36890538e-14,  2.17775278e-14,
                5.36420138e-15,  4.89624223e-16,  6.93929591e-16,  2.70475164e-16,
                -7.38479446e-17, -2.80061890e-16, -1.10057195e-15,  3.09555861e-17,
                6.48768443e-16,  1.22667787e-16],
                [5.14245690e-15,  3.00562395e-15,  6.52280064e-16,  4.20456157e-14,
                4.18675683e-14,  5.36420138e-15,  1.00000000e+01,  9.51325204e+00,
                4.47509822e+00,  2.60423763e-16,  3.58741050e-16,  2.20428578e-16,
                -1.36981312e-16,  6.17842699e-16,  1.03637321e-16, -5.34558188e-16,
                -1.17012414e-15, -1.17353127e-15],
                [2.07516351e-17,  1.51057540e-18, -4.88391287e-18, -2.10094667e-15,
                -1.54936117e-15,  2.70475164e-16, -6.89246928e-16, -4.01573516e-16,
                2.20428578e-16,  1.00000000e+00,  2.41421356e+00,  2.41421356e+00,
                -1.79189186e-15, -1.38984181e-15, -2.03910436e-15, -3.38369976e-15,
                -2.03166292e-15, -2.18936409e-15],
                [-1.39906336e-15, -5.96781518e-16, -9.70178991e-17, -5.22419576e-15,
                -2.75685534e-15, -1.10057195e-15, -4.34521952e-15, -2.12592344e-15,
                1.03637321e-16, -1.46911516e-16, -1.03718173e-15, -2.03910436e-15,
                1.00000000e+00,  2.41421356e+00,  2.41421356e+00,  1.00526731e-15,
                3.55415968e-15,  2.50385769e-15],
                [1.34157716e-15,  7.98929591e-16,  1.81084519e-16,  4.81178165e-15,
                3.18927220e-15,  1.22667787e-16, -5.94289565e-15, -2.71978671e-15,
                -1.17353127e-15, -1.73643522e-15, -2.71023382e-15, -2.18936409e-15,
                2.22947161e-15,  6.83307426e-15,  2.50385769e-15,  1.00000000e+00,
                2.41421356e+00,  2.41421356e+00]
            ])
               
               
            
            v_lqr = K_lqr@error
            self.state.v = v_lqr
            #I_d 값 계산(Numerical method)
            I_inv_cur = np.linalg.inv(I_cur)
            I_inv_prev = np.linalg.inv(I_prev)
            I_d = (I_inv_cur-I_inv_prev)/dt
            
            #F_a_b 값 계산
            p2x = np.array([
                [np.cos(np.pi/4), -np.sin(np.pi/4),0],
                [np.sin(np.pi/4), np.cos(np.pi/4),0],
                [0,0,1]
            ])
            F_a_b = p2x@(np.array([
                [0, kf*np.sin(beta[0][0]), -kf*np.cos(beta[0][0])],
                [-kf*np.cos(alpha[1][0])*np.sin(beta[1][0]), -kf*np.sin(alpha[1][0])*np.sin(beta[1][0]), -kf*np.cos(beta[1][0])],
                [0, -kf*np.sin(beta[2][0]), -kf*np.cos(beta[2][0])],
                [kf*np.cos(alpha[3][0])*np.sin(beta[3][0]), kf*np.sin(alpha[3][0])*np.sin(beta[3][0]), -kf*np.cos(beta[3][0])]
            ]).T)
            
            #Tau_a_b 값 계산 #bl에 1/sqrt(2) 안곱하는 이유 궁금
            Tau_a_b = p2x@(np.array([
                [0, kf * (bl+al) * np.cos(beta[0][0]) + km*np.sin(beta[0][0]), kf * (bl+al) * np.sin(beta[0][0]) - km * np.cos(beta[0][0])],
                [-kf* (bl+al*np.cos(alpha[1][0]))*np.cos(beta[1][0]) + km*np.cos(alpha[1][0])*np.sin(beta[1][0]), -kf*al*np.sin(alpha[1][0])*np.cos(beta[1][0])+km*np.sin(alpha[1][0])*np.sin(beta[1][0]),kf*al*(np.sin(alpha[1][0]))**2*np.sin(beta[1][0])+kf*(bl+al*np.cos(alpha[1][0]))*np.cos(alpha[1][0])*np.sin(beta[1][0])+ km*np.cos(beta[1][0])],
                [0,  -kf*(bl+al)*np.cos(beta[2][0]) - km*np.sin(beta[2][0]),kf*(bl+al)*np.sin(beta[2][0])-km*np.cos(beta[2][0])],
                [kf*(bl+al*np.cos(alpha[3][0]))*np.cos(beta[3][0])-km*np.cos(alpha[3][0])*np.sin(beta[3][0]), kf*al*np.sin(alpha[3][0])*np.cos(beta[3][0])-km*np.sin(alpha[3][0])*np.sin(beta[3][0]),kf*al*(np.sin(alpha[3][0]))**2*np.sin(beta[3][0])+kf*(bl+al*np.cos(alpha[3][0]))*np.cos(alpha[3][0])*np.sin(beta[3][0])+km*np.cos(beta[3][0])]
            ]).T)
            
            #JR matrix 
            R = rpy2rot(phi,theta,psi)         # 3×3 회전 행렬
            R_T = R.T                      # RPY2Rot(obj.euler)'에 해당

            top_left = (1 / m_t) * R_T
            top_right = np.zeros((3, 3))
            bottom_left = np.zeros((3, 3))
            bottom_right = np.linalg.inv(I_cur)
            
            JR = np.block([
                [top_left,     top_right],
                [bottom_left,  bottom_right]
            ])
            
            #JR_dot matrix 
            R = RPY2Rot_derivative(phi,theta,psi,phi_dot,theta_dot,psi_dot)
            top_left = (1/m_t)*R
            top_right = np.zeros((3,3))
            bottom_left = np.zeros((3,3))
            #inv인지 zeros인지 확인
            bottom_right = np.zeros((3,3))
            
            JRdot = np.block([
                [top_left, top_right],
                [bottom_left,bottom_right]
            ])
            
            #J_beta, J_betadot 계산후 B matrix 
            J_beta = np.vstack((F_a_b,Tau_a_b)) # 6*4 matrix
            
            top = p2x@np.array([
                [0,  -kf*np.cos(alpha[1][0])*np.cos(beta[1][0])*w_m[1][0], 0, kf*np.cos(alpha[3][0])*np.cos(beta[3][0])*w_m[3][0]],
                [kf*np.cos(beta[0][0])*w_m[0][0],  -kf*np.sin(alpha[1][0])*np.cos(beta[1][0])*w_m[1][0], -kf*np.cos(beta[2][0])*w_m[2][0], kf*np.sin(alpha[3][0])*np.cos(beta[3][0])*w_m[3][0]],
                [kf*np.sin(beta[0][0])*w_m[0][0],kf*np.sin(beta[1][0])*w_m[1][0], kf*np.sin(beta[2][0])*w_m[2][0],kf*np.sin(beta[3][0])*w_m[3][0]]
            ])
            bottom = p2x@np.array([
                [0,kf*(bl+al*np.cos(alpha[1][0]))*np.sin(beta[1][0])*w_m[1][0] + km*np.cos(alpha[1][0])*np.cos(beta[1][0])*w_m[1][0],0,-kf*(bl+al*np.cos(alpha[3][0]))*np.sin(beta[3][0])*w_m[3][0] - km*np.cos(alpha[3][0])*np.cos(beta[3][0])*w_m[3][0]],
                [(-kf*(bl+al)*np.sin(beta[0][0]) + km*np.cos(beta[0][0]))*w_m[0][0], (kf*al*np.sin(alpha[1][0])*np.sin(beta[1][0])*w_m[1][0] + km*np.sin(alpha[1][0])*np.cos(beta[1][0]))*w_m[1][0],(kf*(bl+al)*np.sin(beta[2][0]) - km*np.cos(beta[2][0]))*w_m[2][0], -(kf*al*np.sin(alpha[3][0])*np.sin(beta[3][0]) + km*np.sin(alpha[3][0])*np.cos(beta[3][0]))*w_m[3][0]],
                [(kf*(bl+al)*np.cos(beta[0][0]) + km*np.sin(beta[0][0]))*w_m[0][0], (kf*al*(np.sin(alpha[1][0]))**2*np.cos(beta[1][0]) + kf*(bl+al*np.cos(alpha[1][0]))*np.cos(alpha[1][0])*np.cos(beta[1][0]) - km*np.sin(beta[1][0]))*w_m[1][0], (kf*(bl+al)*np.cos(beta[2][0]) + km*np.sin(beta[2][0]))*w_m[2][0], (kf*al*(np.sin(alpha[3][0]))**2*np.cos(beta[3][0]) + kf*(bl+al*np.cos(alpha[3][0]))*np.cos(alpha[3][0])*np.cos(beta[3][0]) - km*np.sin(beta[3][0]))*w_m[3][0]]
            ])
            J_betadot = np.vstack((top,bottom)) #6*4 matrix
            
            left = JR@J_beta
            right = JR@J_betadot
            
            B = np.hstack((left,right))
            
            #드디어 Control input 계산 명령을 주는건데 state class의 변수를 변화시키는 것이 맞나?
            B_pinv = np.linalg.pinv(B)
            inner = v_lqr - JRdot @ J_beta @ w_m
            u_control = B_pinv@inner
            
            #Control input 만듬 state class에 있는 것이 맞을지 검토해봐야 할 듯, dt가 처음엔 0일텐데 흠 -> 프로펠러 안돌텐데 그 후엔 dt바뀌니까 되겠네
            self.state.w_d = w_m + u_control[0:4]*dt
            self.state.w_d = np.sqrt(np.maximum(self.state.w_d, 0.0))
            self.state.w_d[0][0] *= -1
            self.state.w_d[2][0] *= -1
            
            self.state.alpha = np.array([
                [0,0,0,0]
            ]).T # 팔 각도 고정
            self.state.beta_dot = u_control[4:]
            
            #F_ab,Tau_ab 업데이트
            self.drone_model.F_ab = F_a_b
            self.drone_model.Tau_ab = Tau_a_b
            
        elif self.state.mode == 'Y':
            #Z-domain에서 v 값 계산
            z_current = np.array([
                x,x_dot,x_ddot,y,y_dot,y_ddot,z,z_dot,z_ddot,phi,phi_dot,phi_ddot,theta,theta_dot,theta_ddot,psi, psi_dot, psi_ddot
            ]).T
            z_ref = np.array([
                x_d,0,0,y_d,0,0,z_d,0,0,phi_d,0,0,theta_d,0,0,psi_d,0,0
            ]).T
            error = z_ref-z_current
                    #이런식으로 K_lqr 계산, 미리 offline 계산
            '''
            A = np.array([
                [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            ])
            
            B = np.array([
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [1, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 1]
            ])
            
            Q = np.array([
                [10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]
            ])
            
            R = np.array([
                [1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1],
            ])
            K_lqr,S,E = control.lqr(A,B,Q,R)
            '''
            K_lqr = np.array([
                [ 3.16227766e+00,  4.60539882e+00,  3.19543387e+00, -3.96321481e-15,
                -1.10947117e-15, -3.95937993e-16, -1.44318141e-15, -1.16117604e-15,
                1.61070316e-16,  1.20254798e-17,  7.98022220e-17,  6.86662535e-17,
                4.10716902e-17,  6.19100687e-17,  1.15493727e-16,  2.93083316e-16,
                5.03738354e-16,  4.44815272e-16],
    
                [ 1.96652683e-15,  6.06446660e-16, -3.95937993e-16,  3.16227766e+00,
                4.60539882e+00,  3.19543387e+00, -6.30018545e-15,  2.96990927e-15,
                3.31060276e-15,  1.03651812e-16,  7.35615984e-16,  1.54723064e-15,
                7.48174089e-17, -1.79554044e-15, -4.44853537e-16, -2.83378512e-16,
                -1.93839691e-16,  1.63389072e-16],

                [ 1.30302090e-15, -5.32161871e-16,  1.61070316e-16,  4.65700913e-15,
                7.29014491e-15,  3.31060276e-15,  3.16227766e+00,  4.60539882e+00,
                3.19543387e+00, -3.51894636e-16, -1.85544397e-17,  2.64460596e-16,
                -1.57376167e-16, -3.43068099e-16,  4.16531675e-17,  9.51648775e-17,
                1.26593610e-15,  9.30125439e-16],

                [ 1.77946870e-16,  2.44369438e-16,  6.86662535e-17,  2.39786224e-15,
                3.37267620e-15,  1.54723064e-15,  1.15960008e-15,  8.28011843e-16,
                2.64460596e-16,  1.00000000e+00,  2.41421356e+00,  2.41421356e+00,
                -4.99147826e-17, -8.77847311e-16, -9.34519928e-16, -1.67207327e-15,
                 -3.62079970e-15, -6.11671034e-16],

                [ 3.79020200e-16,  4.20095960e-16,  1.15493727e-16, -2.12165149e-15,
                -1.71637648e-15, -4.44853537e-16, -2.01861418e-15, -1.00061053e-15,
                4.16531675e-17, -1.89653860e-15, -1.75988242e-15, -9.34519928e-16,
                1.00000000e+00,  2.41421356e+00,  2.41421356e+00,  3.76896039e-15,
                4.99825348e-15,  2.62812479e-15],

                [ 5.24739610e-16,  8.35992470e-16,  4.44815272e-16,  2.86875257e-16,
                3.90290874e-16,  1.63389072e-16,  2.86133848e-15,  2.71550337e-15,
                9.30125439e-16, -1.94845282e-16, -2.66926209e-17, -6.11671034e-16,
                1.26263706e-15,  3.39445859e-15,  2.62812479e-15,  1.00000000e+00,
                2.41421356e+00,  2.41421356e+00]
            ])
            
            v_lqr = K_lqr@error
            self.state.v = v_lqr
            #Y configuration alpha_dot 논리, alpha 명령 줌 alpha_dot 아님
            failnum = self.fault_detection.failnum
            if 0 < failnum:
                i=failnum
                self.state.alpha = np.array([
                    [0,0,0,0]
                ]).T
                self.state.alpha[i]= -np.pi/6
                self.state.alpha[i-2]= np.pi/6
                
            #CM 위치 계산
            p2x = np.array([
                [np.cos(np.pi/4), -np.sin(np.pi/4),0],
                [np.sin(np.pi/4), np.cos(np.pi/4),0],
                [0,0,1]
                ])
            
            acml = self.drone_model.acml
            m_a = self.drone_model.m_a
            cmArm1 = [acml*np.cos(alpha[0])+bl, acml*np.sin(alpha[0]), 0].T
            cmArm2 = [-acml*np.sin(alpha[1]), acml*np.cos(alpha[1])+bl,  0].T
            cmArm3 = [-acml*np.cos(alpha[2])-bl, -acml*np.sin(alpha[2]), 0].T
            cmArm4 = [acml*np.sin(alpha[3]),-acml*np.cos(alpha[3])-bl, 0].T
            cmTot =  p2x *((m_a * cmArm1 + m_a * cmArm2 + m_a * cmArm3 + m_a * cmArm4)/(m_t))
            
            #I_d 값 계산(Numerical method)
            I_inv_cur = np.linalg.inv(I_cur)
            I_inv_prev = np.linalg.inv(I_prev)
            I_d = (I_inv_cur-I_inv_prev)/dt
            
            #F_a_b 값 계산
            F_a_b = np.array([
                [0, kf*np.sin(beta[0]), -kf*np.cos(beta[0])],
                [-kf*np.cos(alpha[1])*np.sin(beta[1]), -kf*np.sin(alpha[1])*np.sin(beta[1]), -kf*np.cos(beta[1])],
                [0, -kf*np.sin(beta[2]), -kf*np.cos(beta[2])],
                [kf*np.cos(alpha[3])*np.sin(beta[3]), kf*np.sin(alpha[3])*np.sin(beta[3]), -kf*np.cos(beta[3])]
            ]).T
            
            #Tau_a_b 값 계산 #bl에 1/sqrt(2) 안곱하는 이유 궁금
            lcm = cmTot[0]
            
            Tau_a_b = p2x @ np.array([
                            [0,                                                                                                        kf*(bl+al-lcm),                                                                                 -km],
                            [-kf*(bl+al*np.cos(alpha[1]))*np.cos(beta[1]) + km*np.cos(alpha[1])*np.sin(beta[1]),     -kf*(al*np.sin(alpha[1])+lcm)*np.cos(beta[1])+km*np.sin(alpha[1])*np.sin(beta[1]),            kf*(al*np.sin(alpha[1])+lcm)*np.sin(alpha[1])*np.sin(beta[1])+kf*(bl+al*np.cos(alpha[1]))*np.cos(alpha[1])*np.sin(beta[1])+ km*np.cos(beta[1])],
                            [                                       0,                                                          -kf*(bl+al+lcm)*np.cos(beta[2]) - km*np.sin(beta[2]),                                       kf*(bl+al+lcm)*np.sin(beta[2]) - km*np.cos(beta[2])],
                            [kf*(bl+al*np.cos(alpha[3]))*np.cos(beta[3])-km*np.cos(alpha[3])*np.sin(beta[3]),      kf*(al*np.sin(alpha[3])-lcm)*np.cos(beta[3])-km*np.sin(alpha[3])*np.sin(beta[3]),            kf*(al*np.sin(alpha[3])-lcm)*np.sin(alpha[3])*np.sin(beta[3])+kf*(bl+al*np.cos(alpha[3]))*np.cos(alpha[3])*np.sin(beta[3])+km*np.cos(beta[3])]
                        ])
            
            #JR matrix 6*6
            R = rpy2rot(phi,theta,psi)         # 3×3 회전 행렬
            R_T = R.T                      # RPY2Rot(obj.euler)'에 해당

            top_left = (1 / m_t) * R_T
            top_right = np.zeros((3, 3))
            bottom_left = np.zeros((3, 3))
            bottom_right = np.linalg.inv(I_cur)
            
            JR = np.block([
                [top_left,     top_right],
                [bottom_left,  bottom_right]
            ])
            
            #JR_dot matrix 
            R = RPY2Rot_derivative(phi,theta,psi,phi_dot,theta_dot,psi_dot)
            R_T = R.T
            top_left = (1/m_t)*R_T
            top_right = np.zeros((3,3))
            bottom_left = np.zeros((3,3))
            bottom_right = np.zeros((3,3))
            
            
            JRdot = np.block([
                [top_left, top_right],
                [bottom_left,bottom_right]
            ])
            
            #J_beta, J_betadot 계산후 B matrix 
            J_beta = np.vstack((F_a_b,Tau_a_b)) #6*4
            
            top = np.array([
                [0,  -kf*np.cos(alpha[1])*np.cos(beta[1])*w_m[1], 0, kf*np.cos(alpha[3])*np.cos(beta[3])*w_m[3]],
                [kf*np.cos(beta[0])*w_m[0],  -kf*np.sin(alpha[1])*np.cos(beta[1])*w_m[1], -kf*np.cos(beta[2])*w_m[2], kf*np.sin(alpha[3])*np.cos(beta[3])*w_m[3]],
                [kf*np.sin(beta[0])*w_m[0],kf*np.sin(beta[1])*w_m[1], kf*np.sin(beta[2])*w_m[2],kf*np.sin(beta[3])*w_m[3]]
            ])
            bottom = np.array([
                [0,kf*(bl+al*np.cos(alpha[1]))*np.sin(beta[1])*w_m[1] + km*np.cos(alpha[1])*np.cos(beta[1])*w_m[1],0,-kf*(bl+al*np.cos(alpha[3]))*np.sin(beta[3])*w_m[3] - km*np.cos(alpha[3])*np.cos(beta[3])*w_m[3]],
                [(-kf*(bl+al)*np.sin(beta[0]) + km*np.cos(beta[0]))*w_m[0], (kf*al*np.sin(alpha[1])*np.sin(beta[1])*w_m[1] + km*np.sin(alpha[1])*np.cos(beta[1]))*w_m[1],(kf*(bl+al)*np.sin(beta[2]) - km*np.cos(beta[2]))*w_m[2], -(kf*al*np.sin(alpha[3])*np.sin(beta[3]) + km*np.sin(alpha[3])*np.cos(beta[3]))*w_m[3]],
                [(kf*(bl+al)*np.cos(beta[0]) + km*np.sin(beta[0]))*w_m[0], (kf*al*(np.sin(alpha[1]))**2*np.cos(beta[1]) + kf*(bl+al*np.cos(alpha[1]))*np.cos(alpha[1])*np.cos(beta[1]) - km*np.sin(beta[1]))*w_m[1], (kf*(bl+al)*np.cos(beta[2]) + km*np.sin(beta[2]))*w_m[2], (kf*al*(np.sin(alpha[3]))**2*np.cos(beta[3]) + kf*(bl+al*np.cos(alpha[3]))*np.cos(alpha[3])*np.cos(beta[3]) - km*np.sin(beta[3]))*w_m[3]]
            ])
            J_betadot = np.vstack((top,bottom)) #6*4
            
            left = JR@J_beta
            right = JR@J_betadot
            
            B = np.hstack((left,right)) #6*8
            
            #fail 난 부분 삭제
            B = np.delete(B,[i-1,i+3],axis =1)
            
            #드디어 Control input 계산 명령을 주는건데 state class의 변수를 변화시키는 것이 맞나?
            B_inv = np.linalg.inv(B)
            lam = 1
            B_T = B.T
            J_beta = np.delete(J_beta,i-1,axis = 1)
            w_m = np.delete(w_m,i-1,axis = 0)
            inner = v_lqr - JRdot @ J_beta @ w_m
            regularized = B_T@B+lam*np.eye(B.shape[1])
            u_control = np.linalg.solve(regularized,B_T@inner)
            
            ##Control input 만듬 state class에 있는 것이 맞을지 검토해봐야 할 듯
            w_m = np.insert(w_m,i-1,np.zeros((0,1)),axis=0)
            self.state.w_d = w_m + np.insert(u_control[0:3,:],i-1,np.zeros((0,1)),axis=0)*dt
            self.state.beta_dot = np.insert(u_control[3:6,:],i-1,np.zeros((0,1)),axis=0)
            
            #F_ab Tau_ab 업데이트
            self.drone_model.F_ab = F_a_b
            self.drone_model.Tau_ab = Tau_a_b
            
        else:  # 'H'
            self.state.alpha = 0.0
            self.state.beta = 0.0
            self.state.w_d = 0.0