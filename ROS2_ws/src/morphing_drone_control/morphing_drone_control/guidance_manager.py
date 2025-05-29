import numpy as np

class GuidanceManager:
    def __init__(self, state):
        self._state = state
<<<<<<< Updated upstream
        self.x_d = 1
        self.y_d = 1
        self.z_d = -5
=======
        self.x_d = 0
        self.y_d = 0
        self.z_d = -1
>>>>>>> Stashed changes
        self.phi_d = 0
        self.theta_d = 0
        self.psi_d = 0
         