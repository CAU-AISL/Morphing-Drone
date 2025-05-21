import numpy as np

class GuidanceManager:
    def __init__(self, state):
        self._state = state
        self.x_d = 2
        self.y_d = 2
        self.z_d = -2
        self.phi_d = 0
        self.theta_d = 0
        self.psi_d = 0