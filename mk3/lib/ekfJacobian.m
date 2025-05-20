function [A, B, H] = ekfJacobian(obj)
% ekfJacobian Compute discrete-time A, B for 12-state EKF
%   [A, B] = ekfJacobian(obj) returns the 12×12 state transition Jacobian A
%   and the 12×4 input Jacobian B based on the current state in obj and
%   sampling time obj.dt.

% unpack
phi   = obj.euler(1);
theta = obj.euler(2);
psi   = obj.euler(3);
omega = obj.w;
wm    = obj.w_m;
dt    = obj.dt;
m_t   = obj.m_t;
F_ab  = obj.F_a_b;
Tau_ab= obj.Tau_a_b;
I_tot = obj.I_tot;

% rotation matrix and transform
R = RPY2Rot([phi;theta;psi])';
T = [ 1,             sin(phi)*tan(theta),  cos(phi)*tan(theta);
      0,             cos(phi),             -sin(phi);
      0,             sin(phi)/cos(theta),  cos(phi)/cos(theta) ];

% initialize
A = eye(12);
B = zeros(12,4);

% 1) r–v block
A(1:3,4:6) = dt * eye(3);

% 2) v–e block
% compute ∂R/∂phi, ∂R/∂theta, ∂R/∂psi
dR_dphi   = dRpy2Rot_dphi(  phi,theta,psi);
dR_dtheta = dRpy2Rot_dtheta(phi,theta,psi);
dR_dpsi   = dRpy2Rot_dpsi(  phi,theta,psi);
J_R = (1/m_t)*[ dR_dphi*F_ab*wm, dR_dtheta*F_ab*wm, dR_dpsi*F_ab*wm ];
A(4:6,7:9) = dt * J_R;

% 3) e–e, e–ω blocks
% ∂(Tω)/∂e
dT_dphi = [ 0, cos(phi)*tan(theta), -sin(phi)*tan(theta);
            0, -sin(phi),           -cos(phi);
            0, cos(phi)/cos(theta), -sin(phi)/cos(theta) ];
dT_dtheta = [ 0, sin(phi)*sec(theta)^2, cos(phi)*sec(theta)^2;
              0, 0,                     0;
              0, sin(phi)*sec(theta)*tan(theta), cos(phi)*sec(theta)*tan(theta) ];
J_T = [ dT_dphi*omega, dT_dtheta*omega, zeros(3,1) ];
A(7:9,7:9)   = eye(3) + dt * J_T;
A(7:9,10:12) = dt * T;

% 4) ω–ω block
Jg = - I_tot \ ( skew(omega)*I_tot + skew(I_tot*omega) );
A(10:12,10:12) = eye(3) + dt * Jg;

% B blocks
B(4:6,:)   = dt * (1/m_t) * (R * F_ab);
B(10:12,:) = dt * (I_tot \ Tau_ab);

% H blocks

% Jacobian block
[]

end

%% helper functions
function R = RPY2Rot(a)
    phi   = a(1); theta = a(2); psi = a(3);
    R3 = [ cos(psi),  sin(psi), 0;
          -sin(psi),  cos(psi), 0;
             0,          0,     1];
    R2 = [ cos(theta), 0, -sin(theta);
               0,      1,      0;
           sin(theta), 0,  cos(theta)];
    R1 = [ 1,      0,       0;
           0, cos(phi), sin(phi);
           0,-sin(phi), cos(phi)];
    R = R1 * R2 * R3;
end

function dR = dRpy2Rot_dphi(phi,theta,psi)
    R2 = [ cos(theta),0,-sin(theta); 0,1,0; sin(theta),0,cos(theta)]';
    R3 = [ cos(psi), sin(psi),0; -sin(psi), cos(psi),0; 0,0,1]';
    dR1= [ 0,0,0; 0,-sin(phi), cos(phi); 0,-cos(phi),-sin(phi) ]';
    dR = R3 * R2 * dR1;
end

function dR = dRpy2Rot_dtheta(phi,theta,psi)
    R1 = [1,0,0; 0,cos(phi),sin(phi); 0,-sin(phi),cos(phi)]';
    R3 = [ cos(psi), sin(psi),0; -sin(psi), cos(psi),0; 0,0,1]';
    dR2= [-sin(theta),0,-cos(theta); 0,0,0; cos(theta),0,-sin(theta)]';
    dR = R3 * dR2 * R1;
end

function dR = dRpy2Rot_dpsi(phi,theta,psi)
    R1 = [1,0,0; 0,cos(phi),sin(phi); 0,-sin(phi),cos(phi)]';
    R2 = [ cos(theta),0,-sin(theta); 0,1,0; sin(theta),0,cos(theta)]';
    dR3= [-sin(psi), cos(psi),0; -cos(psi),-sin(psi),0; 0,0,0]';
    dR = dR3 * R2 * R1;
end

function S = skew(v)
    S = [  0,   -v(3),  v(2);
         v(3),   0,   -v(1);
        -v(2), v(1),    0 ];
end
