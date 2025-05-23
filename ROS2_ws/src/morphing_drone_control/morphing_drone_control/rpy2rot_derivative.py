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

def RPY2Rot_derivative(phi,theta,psi,dphi,dtheta,dpsi):
    phi = float(phi)
    theta = float(theta)
    psi=float(psi)
    dphi=float(dphi)
    dtheta=float(dtheta)
    dpsi=float(dpsi)
    R_3 = np.array([
        [np.cos(float(psi)), np.sin(float(psi)),0],
        [-np.sin(float(psi)), np.cos(float(psi)),0],
        [0,0,1]
    ])
    R_2 = np.array([
        [np.cos(float(theta)),0, -np.sin(float(theta))],
        [0,1,0],
        [np.sin(float(theta)), 0, np.cos(float(theta))]
    ])
    R_1 =np.array([
        [1,0,0],
        [0, np.cos(float(phi)), np.sin(float(phi))],
        [0,-np.sin(float(phi)),np.cos(float(phi))]
    ])
    
    bRi =R_1@R_2@R_3
    
    Omega = np.array([
        [0,-float(dpsi)*np.cos(float(theta))+float(dtheta)*np.sin(float(psi))*np.sin(float(theta)),float(dtheta)*np.cos(float(psi))+ float(dphi)*np.sin(float(theta))],
        [float(dpsi)*np.cos(float(theta)), 0, -float(dphi)*np.cos(float(theta)) + float(dpsi)*np.sin(float(theta))*np.sin(float(phi))],
        [-float(dtheta)*np.cos(psi) - float(dphi)*np.sin(theta), float(dphi)*np.cos(theta) - float(dpsi)*np.sin(theta)*np.sin(phi), 0]
    ])
    
    dR = bRi@Omega
    return dR