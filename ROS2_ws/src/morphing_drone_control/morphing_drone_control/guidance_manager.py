import numpy as np

class GuidanceManager:
    def __init__(self, state):
        self._state = state
        self.x_d = 0
        self.y_d = 0
        self.z_d = -1
        self.phi_d = 0
        self.theta_d = 0
        self.psi_d = 0
         