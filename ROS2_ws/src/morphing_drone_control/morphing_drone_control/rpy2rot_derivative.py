import numpy as np

def dRpy2rot_dphi(phi, theta, psi):
    R2 = np.array([[ np.cos(theta), 0, -np.sin(theta)],
                   [             0, 1,              0],
                   [ np.sin(theta), 0,  np.cos(theta)]]).T
    R3 = np.array([[ np.cos(psi), np.sin(psi),0],
                   [-np.sin(psi), np.cos(psi),0],
                   [0,0,1]]).T
    dR1= np.array([[ 0,           0,          0],
                   [ 0, -np.sin(phi),   np.cos(phi)],
                   [ 0, -np.cos(phi),  -np.sin(phi)]]).T
    return R3.dot(R2).dot(dR1)

def dRpy2rot_dtheta(phi, theta, psi):
    R1 = np.array([[1,0,0],
                   [0,np.cos(phi),np.sin(phi)],
                   [0,-np.sin(phi),np.cos(phi)]]).T
    R3 = np.array([[ np.cos(psi), np.sin(psi),0],
                   [-np.sin(psi), np.cos(psi),0],
                   [0,0,1]]).T
    dR2= np.array([[-np.sin(theta),0,-np.cos(theta)],
                   [0,0,0],
                   [ np.cos(theta),0,-np.sin(theta)]]).T
    return R3.dot(dR2).dot(R1)

def dRpy2rot_dpsi(phi, theta, psi):
    R1 = np.array([[1,0,0],
                   [0,np.cos(phi),np.sin(phi)],
                   [0,-np.sin(phi),np.cos(phi)]]).T
    R2 = np.array([[ np.cos(theta),0,-np.sin(theta)],
                   [0,1,0],
                   [np.sin(theta),0,np.cos(theta)]]).T
    dR3= np.array([[-np.sin(psi), np.cos(psi),0],
                   [-np.cos(psi),-np.sin(psi),0],
                   [0,0,0]]).T
    return dR3.dot(R2).dot(R1)
