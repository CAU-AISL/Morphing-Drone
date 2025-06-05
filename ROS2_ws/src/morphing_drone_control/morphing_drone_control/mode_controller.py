import numpy as np
from morphing_drone_control.rpy2rot import rpy2rot
from morphing_drone_control.rpy2rot_derivative import RPY2Rot_derivative

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
        # alpha[[0,2]]=-1*alpha[[2,0]]
        # alpha[[1,3]]=-1*alpha[[3,1]]
        beta = self.state.beta
        # beta[[0,2]]=beta[[2,0]]
        # beta[[1,3]]=beta[[3,1]]
        
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
        # w_m[[0,2]] = w_m[[2,0]]
        # w_m[[1,3]] = w_m[[3,1]]
        # Guidance 관련 (update 되면 해야 함)
        x_d = self.guidance.x_d 
        y_d = self.guidance.y_d
        z_d = self.guidance.z_d
        phi_d = self.guidance.phi_d
        theta_d = self.guidance.theta_d
        psi_d = self.guidance.psi_d
        
        if self.state.mode == "X":
            
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
                [ 0.3162,  0.9457,  1.3826, -0.0000,  0.0000,  0.0000,  0.0000,  0.0000, -0.0000,  0.0000,  0.0000,  0.0000, -0.0000, -0.0000, -0.0000, -0.0000, -0.0000, -0.0000],
                [-0.0000, -0.0000, -0.0000,  0.3162,  0.9457,  1.3826,  0.0000,  0.0000, -0.0000, -0.0000, -0.0000, -0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000],
                [ 0.0000, -0.0000, -0.0000, -0.0000, -0.0000, -0.0000,  0.3162,  0.9457,  1.3826,  0.0000, -0.0000, -0.0000, -0.0000, -0.0000, -0.0000,  0.0000,  0.0000,  0.0000],
                [-0.0000, -0.0000, -0.0000,  0.0000,  0.0000, -0.0000,  0.0000,  0.0000,  0.0000,  0.1000,  0.4343,  0.9331,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000],
                [-0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000, -0.0000, -0.0000, -0.0000,  0.1000,  0.4343,  0.9331, -0.0000, -0.0000, -0.0000],
                [ 0.0000,  0.0000,  0.0000, -0.0000, -0.0000, -0.0000,  0.0000,  0.0000,  0.0000, -0.0000, -0.0000, -0.0000, -0.0000, -0.0000, -0.0000,  0.1000,  0.4343,  0.9331]
            ])
            # K_lqr = np.array([      
            
            v_lqr = K_lqr@error
            self.state.v = v_lqr
            
            #I_d 값 계산(Numerical method)
            I_inv_cur = np.linalg.inv(I_cur)
            I_inv_prev = np.linalg.inv(I_prev)
            I_d = (I_inv_cur-I_inv_prev)/dt
            
            #F_a_b 값 계산
            p2x = np.array([
                [np.cos(-np.pi/4), -np.sin(-np.pi/4),0],
                [np.sin(-np.pi/4), np.cos(-np.pi/4),0],
                [0,0,1]
            ])
            F_a_b = np.array([
                [0, kf*np.sin(beta[0][0]), -kf*np.cos(beta[0][0])],
                [-kf*np.cos(alpha[1][0])*np.sin(beta[1][0]), -kf*np.sin(alpha[1][0])*np.sin(beta[1][0]), -kf*np.cos(beta[1][0])],
                [0, -kf*np.sin(beta[2][0]), -kf*np.cos(beta[2][0])],
                [kf*np.cos(alpha[3][0])*np.sin(beta[3][0]), kf*np.sin(alpha[3][0])*np.sin(beta[3][0]), -kf*np.cos(beta[3][0])]
            ]).T
            F_a_b = p2x@F_a_b
            
            #Tau_a_b 값 계산 #bl에 1/sqrt(2) 안곱하는 이유 궁금
            Tau_a_b = np.array([
                [0, kf * (bl+al) * np.cos(beta[0][0]) + km*np.sin(beta[0][0]), kf * (bl+al) * np.sin(beta[0][0]) - km * np.cos(beta[0][0])],
                [-kf* (bl+al*np.cos(alpha[1][0]))*np.cos(beta[1][0]) + km*np.cos(alpha[1][0])*np.sin(beta[1][0]), -kf*al*np.sin(alpha[1][0])*np.cos(beta[1][0])+km*np.sin(alpha[1][0])*np.sin(beta[1][0]),kf*al*(np.sin(alpha[1][0]))**2*np.sin(beta[1][0])+kf*(bl+al*np.cos(alpha[1][0]))*np.cos(alpha[1][0])*np.sin(beta[1][0])+ km*np.cos(beta[1][0])],
                [0,  -kf*(bl+al)*np.cos(beta[2][0]) - km*np.sin(beta[2][0]),kf*(bl+al)*np.sin(beta[2][0])-km*np.cos(beta[2][0])],
                [kf*(bl+al*np.cos(alpha[3][0]))*np.cos(beta[3][0])-km*np.cos(alpha[3][0])*np.sin(beta[3][0]), kf*al*np.sin(alpha[3][0])*np.cos(beta[3][0])-km*np.sin(alpha[3][0])*np.sin(beta[3][0]),kf*al*(np.sin(alpha[3][0]))**2*np.sin(beta[3][0])+kf*(bl+al*np.cos(alpha[3][0]))*np.cos(alpha[3][0])*np.sin(beta[3][0])+km*np.cos(beta[3][0])]
            ]).T
            Tau_a_b = p2x @ Tau_a_b
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
            dR = RPY2Rot_derivative(phi,theta,psi,phi_dot,theta_dot,psi_dot)
            top_left = (1/m_t)*dR
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
            print(v_lqr)
            u_control = B_pinv@inner
            
            
            #Control input 만듬 state class에 있는 것이 맞을지 검토해봐야 할 듯, dt가 처음엔 0일텐데 흠 -> 프로펠러 안돌텐데 그 후엔 dt바뀌니까 되겠네
            self.state.w_d = w_m + u_control[0:4]*dt
            self.state.w_d = np.sqrt(np.maximum(self.state.w_d, 0.0))
            self.state.w_d[1][0] *= -1
            self.state.w_d[3][0] *= -1
            # self.state.w_d[[0,2]] = self.state.w_d[[2,0]]
            # self.state.w_d[[1,3]] = self.state.w_d[[3,1]]
            
            self.state.alpha = np.array([
                [0,0,0,0]
            ]).T # 팔 각도 고정
            self.state.beta_dot = u_control[4:]
            # self.state.beta_dot[[0,2]] = self.state.beta_dot[[2,0]]
            # self.state.beta_dot[[1,3]] = self.state.beta_dot[[3,1]]
            
            #F_ab,Tau_ab 업데이트
            self.drone_model.F_ab = F_a_b
            # self.drone_model.F_ab[:,[0,2]]=self.drone_model.F_ab[:,[2,0]]
            # self.drone_model.F_ab[:,[1,3]]=self.drone_model.F_ab[:,[3,1]]
            self.drone_model.Tau_ab = Tau_a_b
            
            
            # self.drone_model.Tau_ab[:,[0,2]]=self.drone_model.Tau_ab[:,[2,0]]
            # self.drone_model.Tau_ab[:,[1,3]]=self.drone_model.Tau_ab[:,[3,1]]
        else:  # ‘H’
            b1 = beta[0][0]
            b2 = beta[1][0]
            b3 = beta[2][0]
            b4 = beta[3][0]
            a1 = alpha[0][0]
            a2 = alpha[1][0]
            a3 = alpha[2][0]
            a4 = alpha[3][0]
            L = al
            k_f = kf
            k_m=km
            F = np.array([
                [kf*np.sin(b1)*np.sin(np.pi/4-a1), -kf*np.sin(b2)*np.cos(np.pi/4-a2), -k_f*np.sin(b3)*np.sin(np.pi/4 - a3), k_f*np.sin(b4)*np.cos(np.pi/4-a4)],
                [k_f*np.sin(b1)*np.cos(np.pi/4-a1), k_f*np.sin(b2)*np.sin(np.pi/4-a2), -k_f*np.sin(b3)*np.cos(np.pi/4-a3), -k_f*np.sin(b4)*np.sin(np.pi/4-a4)],
                [-k_f*np.cos(b1), -k_f*np.cos(b2), -k_f*np.cos(b3), -k_f*np.cos(b4)]
            ])
            tau = np.array([
                [L*k_f*np.cos(b1)*np.sin(np.pi/4-a1)+k_m*np.sin(b1)*np.sin(np.pi/4-a1), -L*k_f*np.cos(b2)*np.cos(np.pi/4-a2)+k_m*np.sin(b2)*np.cos(np.pi/4-a2), -L*k_f*np.cos(b3)*np.sin(np.pi/4-a3)-k_m*np.sin(b3)*np.sin(np.pi/4-a3), L*k_f*np.cos(b4)*np.cos(np.pi/4-a4)-k_m*np.sin(b4)*np.cos(np.pi/4-a4)],
                [L*k_f*np.cos(b1)*np.cos(np.pi/4-a1)+k_m*np.sin(b1)*np.os(np.pi/4-a1), L*k_f*np.cos(b2)*np.sin(np.pi/4-a2)-k_m*np.sin(b2)*np.sin(np.pi/4-a2), -L*k_f*np.cos(b3)*np.cos(np.pi/4-a3)-k_m*np.sin(b3)*np.cos(np.pi/4-a3), -L*k_f*np.cos(b4)*np.sin(np.pi/4-a4)+k_m*np.sin(b4)*np.sin(np.pi/4-a4)],
                [L*k_f*np.sin(b1)-k_m*np.cos(b1), L*k_f*np.sin(b2)+k_m*np.cos(b2), L*k_f*np.sin(b3)-k_m*np.cos(b3), L*k_f*np.sin(b4)+k_m*np.cos(b4)]
            ])
            bRi = rpy2rot(phi,theta,psi)
            R = bRi.T
            B = np.linalg.inv(I_cur)@tau
            C=(1/m_t) * (R@F)
            A = np.vstack([C[2],B])
            z_des = np.array([
                [z_d,0,phi_d,0,theta_d,0,psi_d,0]
            ]).T
            z = np.array([
                [z,z_dot,phi,phi_dot,theta,theta_dot,psi,psi_dot]
            ]).T
            e = z_des-z
            K_H = np.array([
                [22.3606797749979, 6.76175713479831, 3.17187005782990e-14, 1.67998582725492e-15, -5.10422023351287e-14, -1.95010219470621e-15, -1.69751621740729e-14, 1.60838999422581e-15],
                [1.13347232424008e-14, 4.37545952307856e-15, 31.6227766016837, 8.01533238258824, -3.27493164759318e-13, -3.66746303487621e-14, -1.16359055183032e-14, -4.58641139186953e-15],
                [4.59979363370932e-15, 6.00800720787557e-16, -5.37130300656578e-13, -4.62414253308719e-14, 100.000000000000, 14.1774468787578, 2.82287631764139e-13, 8.02392579044950e-14],
                [2.90555177486225e-14, 6.88242373873310e-15, 5.11473850929024e-14, 3.01098195798760e-15, -6.40810119780477e-13, -8.92760557648796e-15, 31.6227766016838, 8.01533238258823]
                ])
            v=K_H@e
            u = np.linalg.inv(A)@(np.array([
                [-9.81],
                [0],
                [0],
                [0]
            ])+v)
            if beta[0][0] < (np.pi/180)*30:
                self.state.beta_dot = np.array([
                    [(np.pi/180)*0.003],
                    [-(np.pi/180)*0.003],
                    [(np.pi/180)*0.003],
                    [-(np.pi/180)*0.003],
                ])
            else:
                self.state.beta_dot = np.array([
                    [0],
                    [0],
                    [0],
                    [0]
                ])
            self.state.alpha = np.array([
                [(np.pi/180)*30],
                [-(np.pi/180)*30],
                [(np.pi/180)*30],
                [-(np.pi/180)*30],
            ])
            self.state.w_d = u    
            
        
        