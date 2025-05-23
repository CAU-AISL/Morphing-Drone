import numpy as np
from .rpy2rot import rpy2rot
from .rpy2rot_derivative import dRpy2rot_dphi, dRpy2rot_dtheta, dRpy2rot_dpsi
from .kalman_filter import skew
import time

class DroneModel:
    def __init__(self, params,state): #state 추가함
        
        # 저장된 파라미터
        self.g = 9.81
        self.m_b = params['bodyMass']
        self.m_a = params['armMass']
        self.m_t = self.m_b + 4*self.m_a
        self.acml = params['armcmLength']
        self.al = params['armLength']
        self.bl = params['bodyLength']
        self.kf = params['ThrustCoeff']
        self.km = params['DragCoeff']
        
        # 관성모멘트 베이스
        Ixxa, Iyya, Izza, Ixza = params['Ixxa'], params['Iyya'], params['Izza'], params['Ixza']
        self.I_arm1 = np.array([[Ixxa,0,-Ixza],[0,Iyya,0],[-Ixza,0,Izza]])
        self.I_arm2 = np.array([[Iyya,0,0],[0,Ixxa,-Ixza],[0,-Ixza,Izza]])
        self.I_arm3 = np.array([[Ixxa,0,Ixza],[0,Iyya,0],[Ixza,0,Izza]])
        self.I_arm4 = np.array([[Iyya,0,0],[0,Ixxa,Ixza],[0,Ixza,Izza]])
        self.I_body = np.diag([params['Ixxb'], params['Iyyb'], params['Izzb']])
        
        self.prev_I_total = None
        self.cur_I_total = None
        
        self.current_time = None
        self.prev_time = None
        
        self.state = state #따라서 여기도 state 추가
        self.F_ab = np.zeros((3,4))
        self.Tau_ab = np.zeros((3,4))
        self.I_total = None
        
    def update(self, state, w_m, beta, alpha):
        
        #time 계산
        now = time.time()
        if self.current_time is None:
            # 첫 호출인 경우 초기화
            self.current_time =now
            self.prev_time = now
        
        else:
            self.prev_time = self.current_time
            self.current_time = now
            
        #Imatrix 계산
        #CM 위치 계산
            p2x = np.array([
                [np.cos(np.pi/4), -np.sin(np.pi/4),0],
                [np.sin(np.pi/4), np.cos(np.pi/4),0],
                [0,0,1]
                ])
            
            acml = self.acml
            m_a = self.m_a
            m_t = self.m_t
            bl = self.bl
            al = self.al
            
        cmArm1 = np.array([[acml*np.cos(alpha[0][0])+bl],
                           [acml*np.sin(alpha[0][0])],
                           [0.0]])
        cmArm2 = np.array([[-acml*np.sin(alpha[1][0])],
                           [acml*np.cos(alpha[1][0])+bl],
                           [0.0]])
        cmArm3 = np.array([[-acml*np.cos(alpha[2][0])-bl],
                           [-acml*np.sin(alpha[2][0])],
                           [0.0]])
        cmArm4 = np.array([[acml*np.sin(alpha[3][0])],
                           [-acml*np.cos(alpha[3][0])-bl],
                           [0.0]])
        cmTot = self.p2x @ ((m_a * cmArm1 + m_a * cmArm2 + m_a * cmArm3 + m_a * cmArm4) / m_t)
        alpha = self.state.alpha #alpha값 갖고옴
        
        if self.cur_I_total is None:
            self.cur_I_total = p2x@(rpy2rot(np.array([[0,0,alpha[0]]]))@self.I_arm1@rpy2rot(np.array([[0,0,alpha[0]]])).T
            
            +m_a*np.array([
                [(cmArm1[1]-cmTot[1])**2+(cmArm1[2]-cmTot[2])**2,   -(cmArm1[0]-cmTot[0])*(cmArm1[1]-cmTot[1]), -(cmArm1[0]-cmTot[0])*(cmArm1[2]-cmTot[2])],
                [-(cmArm1[0]-cmTot[0])*(cmArm1[1]-cmTot[1]),  (cmArm1[0]-cmTot[0])**2+(cmArm1[2]-cmTot[2])**2, -(cmArm1[1]-cmTot[1])*(cmArm1[2]-cmTot[2])],
                [-(cmArm1[0]-cmTot[0])*(cmArm1[2]-cmTot[2]),  -(cmArm1[1]-cmTot[1])*(cmArm1[2]-cmTot[2])   , (cmArm1[0]-cmTot[0])**2+(cmArm1[1]-cmTot[1])**2]
            ])
            
            + rpy2rot(np.array([[0,0,alpha[1]]]))@self.I_arm2@rpy2rot(np.array([[0,0,alpha[1]]])).T
            
            +m_a*np.array([
                [(cmArm2[1]-cmTot[1])**2+(cmArm2[2]-cmTot[2])**2,   -(cmArm2[0]-cmTot[0])*(cmArm2[1]-cmTot[1]), -(cmArm2[0]-cmTot[0])*(cmArm2[2]-cmTot[2])],
                [-(cmArm2[0]-cmTot[0])*(cmArm2[1]-cmTot[1]),  (cmArm2[0]-cmTot[0])**2+(cmArm2[2]-cmTot[2])**2, -(cmArm2[1]-cmTot[1])*(cmArm2[2]-cmTot[2])],
                [-(cmArm2[0]-cmTot[0])*(cmArm2[2]-cmTot[2]),  -(cmArm2[1]-cmTot[1])*(cmArm2[2]-cmTot[2])   , (cmArm2[0]-cmTot[0])**2+(cmArm2[1]-cmTot[1])**2]
            ])
            
            + rpy2rot(np.array([[0,0,alpha[2]]]))@self.I_arm3@rpy2rot(np.array([[0,0,alpha[2]]])).T
            
            + m_a*np.array([
                [(cmArm3[1]-cmTot[1])**2+(cmArm3[2]-cmTot[2])**2,   -(cmArm3[0]-cmTot[0])*(cmArm3[1]-cmTot[1]), -(cmArm3[0]-cmTot[0])*(cmArm3[2]-cmTot[2])],
                [-(cmArm3[0]-cmTot[0])*(cmArm3[1]-cmTot[1]),  (cmArm3[0]-cmTot[0])**2+(cmArm3[2]-cmTot[2])**2, -(cmArm3[1]-cmTot[1])*(cmArm3[2]-cmTot[2])],
                [-(cmArm3[0]-cmTot[0])*(cmArm3[2]-cmTot[2]),  -(cmArm3[1]-cmTot[2])*(cmArm3[2]-cmTot[2])   , (cmArm3[0]-cmTot[0])**2+(cmArm3[1]-cmTot[1])**2]
            ])
            
            +rpy2rot(np.array([[0,0,alpha[3]]]))@self.I_arm3@rpy2rot(np.array([[0,0,alpha[3]]])).T
            
            + m_a*np.array([
                [(cmArm4[1]-cmTot[1])**2+(cmArm4[2]-cmTot[2])**2,   -(cmArm4[0]-cmTot[0])*(cmArm4[1]-cmTot[1]), -(cmArm4[0]-cmTot[0])*(cmArm4[2]-cmTot[2])],
                [-(cmArm4[0]-cmTot[0])*(cmArm4[1]-cmTot[1]),  (cmArm4[0]-cmTot[0])**2+(cmArm4[2]-cmTot[2])**2, -(cmArm4[1]-cmTot[1])*(cmArm4[2]-cmTot[2])],
                [-(cmArm4[0]-cmTot[0])*(cmArm4[2]-cmTot[2]),  -(cmArm4[1]-cmTot[1])*(cmArm4[2]-cmTot[2])   , (cmArm4[0]-cmTot[0])**2+(cmArm4[1]-cmTot[1])**2]
            ])
            
            +self.I_body)@p2x
            self.prev_I_total=self.cur_I_total
            self.I_total = self.cur_I_total
            
        else:
            self.prev_I_total = self.cur_I_total
            self.cur_Itotal = p2x@(rpy2rot(np.array([[0,0,alpha[0]]]))@self.I_arm1@rpy2rot(np.array([[0,0,alpha[0]]])).T
            
            +m_a*np.array([
                [(cmArm1[1]-cmTot[1])**2+(cmArm1[2]-cmTot[2])**2,   -(cmArm1[0]-cmTot[0])*(cmArm1[1]-cmTot[1]), -(cmArm1[0]-cmTot[0])*(cmArm1[2]-cmTot[2])],
                [-(cmArm1[0]-cmTot[0])*(cmArm1[1]-cmTot[1]),  (cmArm1[0]-cmTot[0])**2+(cmArm1[2]-cmTot[2])**2, -(cmArm1[1]-cmTot[1])*(cmArm1[2]-cmTot[2])],
                [-(cmArm1[0]-cmTot[0])*(cmArm1[2]-cmTot[2]),  -(cmArm1[1]-cmTot[1])*(cmArm1[2]-cmTot[2])   , (cmArm1[0]-cmTot[0])**2+(cmArm1[1]-cmTot[1])**2]
            ])
            
            + rpy2rot(np.array([[0,0,alpha[1]]]))@self.I_arm2@rpy2rot(np.array([[0,0,alpha[1]]])).T
            
            +m_a*np.array([
                [(cmArm2[1]-cmTot[1])**2+(cmArm2[2]-cmTot[2])**2,   -(cmArm2[0]-cmTot[0])*(cmArm2[1]-cmTot[1]), -(cmArm2[0]-cmTot[0])*(cmArm2[2]-cmTot[2])],
                [-(cmArm2[0]-cmTot[0])*(cmArm2[1]-cmTot[1]),  (cmArm2[0]-cmTot[0])**2+(cmArm2[2]-cmTot[2])**2, -(cmArm2[1]-cmTot[1])*(cmArm2[2]-cmTot[2])],
                [-(cmArm2[0]-cmTot[0])*(cmArm2[2]-cmTot[2]),  -(cmArm2[1]-cmTot[1])*(cmArm2[2]-cmTot[2])   , (cmArm2[0]-cmTot[0])**2+(cmArm2[1]-cmTot[1])**2]
            ])
            
            + rpy2rot(np.array([[0,0,alpha[2]]]))@self.I_arm3@rpy2rot(np.array([[0,0,alpha[2]]])).T
            
            + m_a*np.array([
                [(cmArm3[1]-cmTot[1])**2+(cmArm3[2]-cmTot[2])**2,   -(cmArm3[0]-cmTot[0])*(cmArm3[1]-cmTot[1]), -(cmArm3[0]-cmTot[0])*(cmArm3[2]-cmTot[2])],
                [-(cmArm3[0]-cmTot[0])*(cmArm3[1]-cmTot[1]),  (cmArm3[0]-cmTot[0])**2+(cmArm3[2]-cmTot[2])**2, -(cmArm3[1]-cmTot[1])*(cmArm3[2]-cmTot[2])],
                [-(cmArm3[0]-cmTot[0])*(cmArm3[2]-cmTot[2]),  -(cmArm3[1]-cmTot[2])*(cmArm3[2]-cmTot[2])   , (cmArm3[0]-cmTot[0])**2+(cmArm3[1]-cmTot[1])**2]
            ])
            
            +rpy2rot(np.array([[0,0,alpha[3]]]))@self.I_arm3@rpy2rot(np.array([[0,0,alpha[3]]])).T
            
            + m_a*np.array([
                [(cmArm4[1]-cmTot[1])**2+(cmArm4[2]-cmTot[2])**2,   -(cmArm4[0]-cmTot[0])*(cmArm4[1]-cmTot[1]), -(cmArm4[0]-cmTot[0])*(cmArm4[2]-cmTot[2])],
                [-(cmArm4[0]-cmTot[0])*(cmArm4[1]-cmTot[1]),  (cmArm4[0]-cmTot[0])**2+(cmArm4[2]-cmTot[2])**2, -(cmArm4[1]-cmTot[1])*(cmArm4[2]-cmTot[2])],
                [-(cmArm4[0]-cmTot[0])*(cmArm4[2]-cmTot[2]),  -(cmArm4[1]-cmTot[1])*(cmArm4[2]-cmTot[2])   , (cmArm4[0]-cmTot[0])**2+(cmArm4[1]-cmTot[1])**2]
            ])
            
            +self.I_body)@p2x.T
            
            self.I_total = self.cur_I_total