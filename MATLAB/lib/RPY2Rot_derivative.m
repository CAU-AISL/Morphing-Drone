function dR = RPY2Rot_derivative(angles, ang_vel)
    phi = angles(1);
    theta = angles(2);
    psi = angles(3);

    dphi = ang_vel(1);
    dtheta = ang_vel(2);
    dpsi = ang_vel(3);

    % 회전 행렬
    R_3 = [cos(psi), sin(psi), 0;
          -sin(psi), cos(psi), 0; 
                  0,        0, 1];
    R_2 = [cos(theta), 0, -sin(theta);
                    0, 1,           0;
           sin(theta), 0, cos(theta)];
    R_1 = [1,        0,        0;
           0, cos(phi), sin(phi);
           0,-sin(phi), cos(phi)];

    % 전체 회전 행렬
    bRi = R_1 * R_2 * R_3;

    % 각속도 행렬 Omega
    Omega = [ 0, -dpsi*cos(theta) + dtheta*sin(psi)*sin(theta), dtheta*cos(psi) + dphi*sin(theta);
             dpsi*cos(theta), 0, -dphi*cos(theta) + dpsi*sin(theta)*sin(phi);
            -dtheta*cos(psi) - dphi*sin(theta), dphi*cos(theta) - dpsi*sin(theta)*sin(phi), 0];

    % 회전 행렬의 미분
    dR = bRi * Omega;
end
