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
        Ixxa, Ixya,Ixza,Iyya,Iyza, Izza, Ixza = params['Ixxa'],params['Ixya'],params['Ixza'], params['Iyya'],params['Iyza'], params['Izza'], params['Ixza']
        self.I_arm2 = self.I_arm3 = self.I_arm4 = self.I_arm1 = np.array([[Ixxa,Ixya,Ixza],[Ixya,Iyya,Iyza],[Ixza,Iyza,Izza]])
        self.I_body = np.array([
            [params['Ixxb'], 0.0, params['Ixzb']],
            [0.0, params['Iyyb'], 0.0],
            [params['Ixzb'], 0.0, params['Izzb']]
        ])
        
        self.prev_I_total = None
        self.cur_I_total = None
        
        self.current_time = None
        self.prev_time = None
        
        self.state = state #따라서 여기도 state 추가
        self.F_ab = np.zeros((3,4))
        self.Tau_ab = np.zeros((3,4))
        self.I_total = None
        
        self.p2x = np.array([
            [np.cos(np.pi/4), -np.sin(np.pi/4), 0],
            [np.sin(np.pi/4), np.cos(np.pi/4), 0],
            [0, 0, 1]
        ])
        
        
        
    def update(self, alpha):
        
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
            self.cur_I_total = self.p2x@(
            rpy2rot(0,0,alpha[0][0])@self.I_arm1@(rpy2rot(0,0,alpha[0][0]).T)
            
            +m_a*np.array([
                [(cmArm1[1][0]-cmTot[1][0])**2+(cmArm1[2][0]-cmTot[2][0])**2,   -(cmArm1[0][0]-cmTot[0][0])*(cmArm1[1][0]-cmTot[1][0]), -(cmArm1[0][0]-cmTot[0][0])*(cmArm1[2][0]-cmTot[2][0])],
                [-(cmArm1[0][0]-cmTot[0][0])*(cmArm1[1][0]-cmTot[1][0]),  (cmArm1[0][0]-cmTot[0][0])**2+(cmArm1[2][0]-cmTot[2][0])**2, -(cmArm1[1][0]-cmTot[1][0])*(cmArm1[2][0]-cmTot[2][0])],
                [-(cmArm1[0][0]-cmTot[0][0])*(cmArm1[2][0]-cmTot[2][0]),  -(cmArm1[1][0]-cmTot[1][0])*(cmArm1[2][0]-cmTot[2][0])   , (cmArm1[0][0]-cmTot[0][0])**2+(cmArm1[1][0]-cmTot[1][0])**2]
            ])
            
            + rpy2rot(0,0,alpha[1][0])@self.I_arm2@(rpy2rot(0,0,alpha[1][0]).T)
            
            +m_a*np.array([
                [(cmArm2[1][0]-cmTot[1][0])**2+(cmArm2[2][0]-cmTot[2][0])**2,   -(cmArm2[0][0]-cmTot[0][0])*(cmArm2[1][0]-cmTot[1][0]), -(cmArm2[0][0]-cmTot[0][0])*(cmArm2[2][0]-cmTot[2][0])],
                [-(cmArm2[0][0]-cmTot[0][0])*(cmArm2[1][0]-cmTot[1][0]),  (cmArm2[0][0]-cmTot[0][0])**2+(cmArm2[2][0]-cmTot[2][0])**2, -(cmArm2[1][0]-cmTot[1][0])*(cmArm2[2][0]-cmTot[2][0])],
                [-(cmArm2[0][0]-cmTot[0][0])*(cmArm2[2][0]-cmTot[2][0]),  -(cmArm2[1][0]-cmTot[1][0])*(cmArm2[2][0]-cmTot[2][0])   , (cmArm2[0][0]-cmTot[0][0])**2+(cmArm2[1][0]-cmTot[1][0])**2]
            ])
            
            + rpy2rot(0,0,alpha[2][0])@self.I_arm3@(rpy2rot(0,0,alpha[2][0]).T)
            
            + m_a*np.array([
                [(cmArm3[1][0]-cmTot[1][0])**2+(cmArm3[2][0]-cmTot[2][0])**2,   -(cmArm3[0][0]-cmTot[0][0])*(cmArm3[1][0]-cmTot[1][0]), -(cmArm3[0][0]-cmTot[0][0])*(cmArm3[2][0]-cmTot[2][0])],
                [-(cmArm3[0][0]-cmTot[0][0])*(cmArm3[1][0]-cmTot[1][0]),  (cmArm3[0][0]-cmTot[0][0])**2+(cmArm3[2][0]-cmTot[2][0])**2, -(cmArm3[1][0]-cmTot[1][0])*(cmArm3[2][0]-cmTot[2][0])],
                [-(cmArm3[0][0]-cmTot[0][0])*(cmArm3[2][0]-cmTot[2][0]),  -(cmArm3[1][0]-cmTot[2][0])*(cmArm3[2][0]-cmTot[2][0])   , (cmArm3[0][0]-cmTot[0][0])**2+(cmArm3[1][0]-cmTot[1][0])**2]
            ])
            
            +rpy2rot(0,0,alpha[3][0])@self.I_arm3@(rpy2rot(0,0,alpha[3][0]).T)
            
            + m_a*np.array([
                [(cmArm4[1][0]-cmTot[1][0])**2+(cmArm4[2][0]-cmTot[2][0])**2,   -(cmArm4[0][0]-cmTot[0][0])*(cmArm4[1][0]-cmTot[1][0]), -(cmArm4[0][0]-cmTot[0][0])*(cmArm4[2][0]-cmTot[2][0])],
                [-(cmArm4[0][0]-cmTot[0][0])*(cmArm4[1][0]-cmTot[1][0]),  (cmArm4[0][0]-cmTot[0][0])**2+(cmArm4[2][0]-cmTot[2][0])**2, -(cmArm4[1][0]-cmTot[1][0])*(cmArm4[2][0]-cmTot[2][0])],
                [-(cmArm4[0][0]-cmTot[0][0])*(cmArm4[2][0]-cmTot[2][0]),  -(cmArm4[1][0]-cmTot[1][0])*(cmArm4[2][0]-cmTot[2][0])   , (cmArm4[0][0]-cmTot[0][0])**2+(cmArm4[1][0]-cmTot[1][0])**2]
            ])
            
            +self.I_body)@(self.p2x.T)
            self.prev_I_total=self.cur_I_total
            self.I_total = self.cur_I_total
            self.cur_I_total = self.cur_I_total
            
        else:
            self.prev_I_total = self.cur_I_total
            self.cur_I_total = self.p2x@(rpy2rot(0,0,alpha[0][0])@self.I_arm1@rpy2rot(0,0,alpha[0][0]).T
            
            +m_a*np.array([
                [(cmArm1[1][0]-cmTot[1][0])**2+(cmArm1[2][0]-cmTot[2][0])**2,   -(cmArm1[0][0]-cmTot[0][0])*(cmArm1[1][0]-cmTot[1][0]), -(cmArm1[0][0]-cmTot[0][0])*(cmArm1[2][0]-cmTot[2][0])],
                [-(cmArm1[0][0]-cmTot[0][0])*(cmArm1[1][0]-cmTot[1][0]),  (cmArm1[0][0]-cmTot[0][0])**2+(cmArm1[2][0]-cmTot[2][0])**2, -(cmArm1[1][0]-cmTot[1][0])*(cmArm1[2][0]-cmTot[2][0])],
                [-(cmArm1[0][0]-cmTot[0][0])*(cmArm1[2][0]-cmTot[2][0]),  -(cmArm1[1][0]-cmTot[1][0])*(cmArm1[2][0]-cmTot[2][0])   , (cmArm1[0][0]-cmTot[0][0])**2+(cmArm1[1][0]-cmTot[1][0])**2]
            ])
            
            + rpy2rot(0,0,alpha[1][0])@self.I_arm2@(rpy2rot(0,0,alpha[1][0]).T)
            
            +m_a*np.array([
                [(cmArm2[1][0]-cmTot[1][0])**2+(cmArm2[2][0]-cmTot[2][0])**2,   -(cmArm2[0][0]-cmTot[0][0])*(cmArm2[1][0]-cmTot[1][0]), -(cmArm2[0][0]-cmTot[0][0])*(cmArm2[2][0]-cmTot[2][0])],
                [-(cmArm2[0][0]-cmTot[0][0])*(cmArm2[1][0]-cmTot[1][0]),  (cmArm2[0][0]-cmTot[0][0])**2+(cmArm2[2][0]-cmTot[2][0])**2, -(cmArm2[1][0]-cmTot[1][0])*(cmArm2[2][0]-cmTot[2][0])],
                [-(cmArm2[0][0]-cmTot[0][0])*(cmArm2[2][0]-cmTot[2][0]),  -(cmArm2[1][0]-cmTot[1][0])*(cmArm2[2][0]-cmTot[2][0])   , (cmArm2[0][0]-cmTot[0][0])**2+(cmArm2[1][0]-cmTot[1][0])**2]
            ])
            
            + rpy2rot(0,0,alpha[2][0])@self.I_arm3@(rpy2rot(0,0,alpha[2][0]).T)
            
            + m_a*np.array([
                [(cmArm3[1][0]-cmTot[1][0])**2+(cmArm3[2][0]-cmTot[2][0])**2,   -(cmArm3[0][0]-cmTot[0][0])*(cmArm3[1][0]-cmTot[1][0]), -(cmArm3[0][0]-cmTot[0][0])*(cmArm3[2][0]-cmTot[2][0])],
                [-(cmArm3[0][0]-cmTot[0][0])*(cmArm3[1][0]-cmTot[1][0]),  (cmArm3[0][0]-cmTot[0][0])**2+(cmArm3[2][0]-cmTot[2][0])**2, -(cmArm3[1][0]-cmTot[1][0])*(cmArm3[2][0]-cmTot[2][0])],
                [-(cmArm3[0][0]-cmTot[0][0])*(cmArm3[2][0]-cmTot[2][0]),  -(cmArm3[1][0]-cmTot[2][0])*(cmArm3[2][0]-cmTot[2][0])   , (cmArm3[0][0]-cmTot[0][0])**2+(cmArm3[1][0]-cmTot[1][0])**2]
            ])
            
            +rpy2rot(0,0,alpha[3][0])@self.I_arm3@(rpy2rot(0,0,alpha[3][0]).T)
            
            + m_a*np.array([
                [(cmArm4[1][0]-cmTot[1][0])**2+(cmArm4[2][0]-cmTot[2][0])**2,   -(cmArm4[0][0]-cmTot[0][0])*(cmArm4[1][0]-cmTot[1][0]), -(cmArm4[0][0]-cmTot[0][0])*(cmArm4[2][0]-cmTot[2][0])],
                [-(cmArm4[0][0]-cmTot[0][0])*(cmArm4[1][0]-cmTot[1][0]),  (cmArm4[0][0]-cmTot[0][0])**2+(cmArm4[2][0]-cmTot[2][0])**2, -(cmArm4[1][0]-cmTot[1][0])*(cmArm4[2][0]-cmTot[2][0])],
                [-(cmArm4[0][0]-cmTot[0][0])*(cmArm4[2][0]-cmTot[2][0]),  -(cmArm4[1][0]-cmTot[1][0])*(cmArm4[2][0]-cmTot[2][0])   , (cmArm4[0][0]-cmTot[0][0])**2+(cmArm4[1][0]-cmTot[1][0])**2]
            ])
            
            +self.I_body)@(self.p2x.T)
            
            self.I_total = self.cur_I_total
            
