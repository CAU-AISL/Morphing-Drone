import numpy as np
class GuidanceManager:
    def __init__(self, state, model): # controller dlfeks djqtdoshgdma
        self.state = state
        self.model = model
        # self.controller = controller
        self.x_d = 5
        self.y_d = -5
        self.z_d = -5
        self.phi_d = 0
        self.theta_d = 0
        self.psi_d = 0
        
    def Hcommand(self):
        k_f = self.model.k_f
        b1 = self.state.beta[0][0]
        b2 = self.state.beta[1][0]
        b3 = self.state.beta[2][0]
        b4 = self.state.beta[3][0]
        a1 = self.state.alpha[0][0]
        a2 = self.state.alpha[1][0]
        a3 = self.state.alpha[2][0]
        a4 = self.state.alpha[3][0]
        u = self.state.w_d
        T_x = np.array([
            [k_f*np.sin(b1)*np.sin(np.pi/4-a1)],
            [-k_f*np.sin(b2)*np.cos(np.pi/4-a2)],
            [-k_f*np.sin(b3)*np.sin(np.pi/4 - a3)],
            [k_f*np.sin(b4)*np.cos(np.pi/4-a4)]
        ]).T@np.array([
            [u[0][0]],
            [u[1][0]],
            [u[2][0]],
            [u[3][0]]
        ])
        T_y = np.array([
            [k_f*np.sin(b1)*np.cos(np.pi/4-a1)],
            [k_f*np.sin(b2)*np.sin(np.pi/4-a2)],
            [-k_f*np.sin(b3)*np.cos(np.pi/4-a3)],
            [-k_f*np.sin(b4)*np.sin(np.pi/4-a4)]
        ]).T@np.array([
            [u[0][0]],
            [u[1][0]],
            [u[2][0]],
            [u[3][0]]
        ])
        T_z = np.array([
            [-k_f*np.cos(b1)],
            [-k_f*np.cos(b2)],
            [-k_f*np.cos(b3)],
            [-k_f*np.cos(b4)]
        ]).T@np.array([
            [u[0][0]],
            [u[1][0]],
            [u[2][0]],
            [u[3][0]]
        ])
        K_px = 3
        K_dx = 3
        K_py = 3
        K_dy = 3
        a_x = K_px*(self.x_d-self.state.x_hat)+K_dx*(0-self.state.x_dot_hat)
        a_y = K_py*(self.y_d-self.state.y_hat)+K_dy*(0-self.state.y_dot_hat)
        R1 = np.array([
            [np.cos(self.state.psi_hat), np.sin(self.state.psi_hat)],
            [-np.sin(self.state.psi_hat), np.cos(self.state.psi_hat)]
        ])
        m=self.model.m_t
        g=self.model.g
        a_body = np.array([
            [a_x - (1/m)*(np.cos(self.state.psi_hat)*T_x - np.sin(self.state.psi_hat)*T_y)],
            [a_y - (1/m)*(np.sin(self.state.psi_hat)*T_x + np.cos(self.state.psi_hat)*T_y)]
        ])
        angles_dot = 1/g * R1@a_body
        self.theta_d = angles_dot[0][0]
        self.phi_d = angles_dot[1][0]
        self.theta_d = - self.theta_d
        limit = np.pi / 4  # MATLAB 코드와 동일한 π/4
        self.theta_d = np.clip(self.theta_d, -limit, limit)
        self.phi_d   = np.clip(self.phi_d, -limit, limit)
