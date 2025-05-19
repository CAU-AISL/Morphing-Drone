import numpy as np
from .rpy2rot import rpy2rot
from .rpy2rot_derivative import dRpy2rot_dphi, dRpy2rot_dtheta, dRpy2rot_dpsi
from .kalman_filter import skew

class DroneModel:
    def __init__(self, params):
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

    def update(self, state, w_m, beta, alpha):
        phi, theta, psi = state.euler

        self.F_ab = np.zeros((3,4))
        for i in range(4):

            self.F_ab[:,i] = [0, 0, -self.kf]

        self.Tau_ab = np.zeros((3,4))
        for i in range(4):
            self.Tau_ab[:,i] = [0, 0, self.km]

        self.I_tot = self.I_body.copy()

        self.w_m = w_m.reshape((4,1))

        return self
