import numpy as np

def rpy2rot(phi,theta,psi):
    phi = float(phi)
    theta = float(theta)
    psi = float(psi)
    R3 = np.array([[ np.cos(psi),  np.sin(psi), 0],
                   [-np.sin(psi),  np.cos(psi), 0],
                   [           0,             0, 1]], dtype=float)
    R2 = np.array([[ np.cos(theta), 0, -np.sin(theta)],
                   [             0, 1,              0],
                   [ np.sin(theta), 0,  np.cos(theta)]], dtype=float)
    R1 = np.array([[1,            0,           0],
                   [0,  np.cos(phi), np.sin(phi)],
                   [0, -np.sin(phi), np.cos(phi)]], dtype=float)
    return R1.dot(R2).dot(R3)