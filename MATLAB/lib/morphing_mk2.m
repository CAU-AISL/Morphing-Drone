classdef morphing_mk2 < handle
    %% MEMBERS
    properties
        g
        t
        dt
        tf
        kf
        km

        m_a
        m_b
        m_t        
        al   % acual arm length
        acml % arm cm length
        bl   % body length
        I_arm1
        I_arm2
        I_arm3
        I_arm4
        I_body
        I_tot
        I_tot_prev
        dI_totinv

        F_a_b
        Tau_a_b
        JR
        JRdot   % feedback linearization mapping
        J_beta
        J_betadot

        x           % [X, Y, Z, dX, dY, dZ, phi, theta,psi, p, q, r]'
        dx_prev
        r           % [X, Y, Z]'
        dr          % [dX, dY, dZ]'
        ddr         % [ddX, ddY, ddZ]'
        euler       % [phi, theta, psi]'
        w           % [p, q, r]'
        dw          % [dp, dq, dr]'
        alpha       % [alpha1, alpha2, alpha3, alpha4]' FLARE ANGLE
        dalpha      % angular rate of flare angle
        targetalpha % target flare angle
        beta        % [beta1, beta2, beta3, beta4]' TILT ANGLE
        dbeta       % angular rate of tilt angle 

        cmArm1
        cmArm2
        cmArm3
        cmArm4
        cmTot
        drone1_body

        dx

        u           % [w1, w2, w3, w4, wb3]'
        w_m
        w_b
        u_control
    end

    properties
        x_des  %  x 참조
        y_des
        z_des
        phi_des
        theta_des
        psi_des
        dx_des
        dy_des
        dz_des
        dphi_des
        dtheta_des
        dpsi_des
        ddx_des
        ddy_des
        ddz_des
        ddphi_des
        ddtheta_des
        ddpsi_des
        
        % for PID
        % phi_des
        % phi_err
        % phi_err_prev
        % phi_err_sum
        % 
        % theta_des
        % theta_err
        % theta_err_prev
        % theta_err_sum
        % 
        % psi_des
        % psi_err
        % psi_err_prev
        % psi_err_sum
        % 
        % zdot_des
        % zdot_err
        % zdot_err_prev
        % zdot_err_sum

        K_lqr   % LQR gain
        v_lqr   % LQR input
        %% gain for PID
        % KP_phi
        % KI_phi
        % KD_phi
        % 
        % KP_theta
        % KI_theta
        % KD_theta
        % 
        % KP_psi
        % KI_psi
        % KD_psi
        % 
        % KP_zdot
        % KI_zdot
        % KD_zdot
    end
    %% METHODS
    methods
        %% CONSTRUCTOR
        function obj = morphing_mk2(params, initStates, initInputs, initFlare, targetFlare, initTilt, simTime)
            obj.g = 9.81;
            obj.t = 0.0;
            obj.dt = 0.01;
            obj.tf = simTime;
            obj.alpha = initFlare;
            obj.targetalpha = targetFlare;
            obj.beta = initTilt;

            

            obj.m_b = params('bodyMass');
            obj.m_a = params('armMass');
            obj.m_t = obj.m_b + 4 * obj.m_a;
            obj.acml = params('armcmLength');
            obj.al = params('armLength');
            obj.bl = params('bodyLength');

            

            
            obj.I_arm1 = [params('Ixxa'),    0,               -params('Ixza');  ...
                         0,                 params('Iyya'),   0; ...
                        -params('Ixza'),    0,                params('Izza')];

            obj.I_arm2 = [params('Iyya'),    0,               0;  ...
                         0,                 params('Ixxa'),  -params('Ixza'); ...
                         0,                -params('Ixza'),   params('Izza')];

            obj.I_arm3 = [params('Ixxa'),    0,               params('Ixza');  ...
                         0,                 params('Iyya'),   0; ...
                         params('Ixza'),    0,                params('Izza')];

            obj.I_arm4 = [params('Iyya'),    0,               0;  ...
                         0,                 params('Ixxa'),   params('Ixza'); ...
                         0,                 params('Ixza'),   params('Izza')];

            obj.I_body = [params('Ixxb'), 0,              0;  ...
                          0,              params('Iyyb'), 0; ...
                          0,              0,              params('Izzb')];

           % aerodynamic coefficients
            obj.kf = params('ThrustCoeff');  
            obj.km = params('DragCoeff');
                          

            % state vectors
            obj.x = initStates;
            obj.r = obj.x(1:3); 
            obj.dr = obj.x(4:6);
            obj.ddr = obj.x(7:9);
            obj.euler = obj.x(10:12);
            obj.w = obj.x(13:15);
            obj.dw = obj.x(16:18);
            
            
            obj.dx = zeros(18,1);

            obj.dalpha = zeros(4,1);

            obj.I_tot_prev = [0.0582,   0,             0
                         0,        0.0582,        0
                         0,        0,             0.0928];
            obj.I_tot = [0.0582,   0,             0
                         0,        0.0582,        0
                         0,        0,             0.0928];
            

            obj.u = initInputs;
            obj.w_m = obj.u(1:4);  % w1,w2,w3,w4 (main motor omega)
            obj.w_b = obj.u(5);    % wb3 (tilt motor omega)
            
            %%Linear model

            matA = [zeros(6,6), eye(6), zeros(6,6);...
            zeros(6,6), zeros(6,6), eye(6);...
            zeros(6,6), zeros(6,6), zeros(6,6)];

            matB = [zeros(12,6);...
            eye(6)];

            matC = eye(18);

            matD = zeros(6,6);
            
            

            n = size(matA,1);
            m = size(matB,2);
            Q = 1*eye(n);
            R = 1*eye(m);

            [K, S, e] = lqr(matA, matB, Q, R);
            obj.K_lqr = K;  % 객체 속성에 저장

            disp('LQR Gain K =');
            disp(K);


                    
            % obj.phi_des = 0.0;
            % obj.phi_err = 0.0;
            % obj.phi_err_prev = 0.0;
            % obj.phi_err_sum = 0.0;
            % 
            % obj.theta_des = 0.0;
            % obj.theta_err = 0.0;
            % obj.theta_err_prev = 0.0;
            % obj.theta_err_sum = 0.0;
            % 
            % obj.psi_des = 0.0;
            % obj.psi_err = 0.0;
            % obj.psi_err_prev = 0.0;
            % obj.psi_err_sum = 0.0;
            % 
            % obj.zdot_des = 0.0;
            % obj.zdot_err = 0.0;
            % obj.zdot_err_prev = 0.0;
            % obj.zdot_err_sum = 0.0;

           
        end
        
%% 
        
        function obj = EvalF_a_b(obj) % Translation motion mixing matrix
            obj.F_a_b = [ 0,                        0,                   -obj.kf; ...
                          0,                        0,                   -obj.kf; ...
                          0, -obj.kf*sin(obj.beta(3)),  -obj.kf*cos(obj.beta(3)); ...
                          0,                        0,                   -obj.kf]'; 
            % disp('F function');
            % disp(obj.F_a_b);
        end

        function obj = EvalTau_a_b(obj) % Rotation motion mixing matrix
            obj.Tau_a_b = [                                        0,                                              obj.kf*(obj.bl+obj.al),                                                           -obj.km; ...
                           -obj.kf*(obj.bl+obj.al*cos(obj.alpha(2))),                                    -obj.kf*obj.al*sin(obj.alpha(2)),                                                            obj.km; ...
                                                                   0,  -obj.kf*(obj.bl+obj.al)*cos(obj.beta(3)) - obj.km*sin(obj.beta(3)),  obj.kf*(obj.bl+obj.al)*sin(obj.beta(3)) - obj.km*cos(obj.beta(3)); ...
                            obj.kf*(obj.bl+obj.al*cos(obj.alpha(4))),                                     obj.kf*obj.al*sin(obj.alpha(4)),                                                            obj.km]';
            % disp('Tau function');
            % disp(obj.Tau_a_b);
        end

        function obj = EvaldI(obj)
            
            
            a = inv(obj.I_tot);
            
            b = inv(obj.I_tot_prev);
            
              obj.dI_totinv = (a-b)/obj.dt;
              disp(obj.dI_totinv);


        end



        function state = GetState(obj) % main function에 상태 반환
            state.x = obj.x;
            state.flare = obj.alpha;
            state.body = obj.drone1_body;
        end

        function obj = EvalCM(obj) % 무게중심 계산

            obj.cmArm1 = [obj.acml*cos(obj.alpha(1))+obj.bl, obj.acml*sin(obj.alpha(1)), 0]';
            obj.cmArm2 = [-obj.acml*sin(obj.alpha(2)), obj.acml*cos(obj.alpha(2))+obj.bl,  0]';
            obj.cmArm3 = [-obj.acml*cos(obj.alpha(3))-obj.bl, -obj.acml*sin(obj.alpha(3)), 0]';
            obj.cmArm4 = [obj.acml*sin(obj.alpha(4)),-obj.acml*cos(obj.alpha(4))-obj.bl, 0]';
            obj.cmTot = (obj.m_a * obj.cmArm1 + obj.m_a * obj.cmArm2 + obj.m_a * obj.cmArm3 + obj.m_a * obj.cmArm4)/(obj.m_t);            
        end

        function obj = EvalItot(obj) % 관성모멘트 계산

            obj.I_tot_prev = obj.I_tot;
            obj.EvalCM();
            
            obj.I_tot = RPY2Rot([0, 0 ,obj.alpha(1)]) * obj.I_arm1 * RPY2Rot([0, 0 ,obj.alpha(1)])' + ...
            ...
                obj.m_a*[(obj.cmArm1(2)-obj.cmTot(2))^2+(obj.cmArm1(3)-obj.cmTot(3))^2,   -(obj.cmArm1(1)-obj.cmTot(1))*(obj.cmArm1(2)-obj.cmTot(2)), -(obj.cmArm1(1)-obj.cmTot(1))*(obj.cmArm1(3)-obj.cmTot(3)); ...
                          -(obj.cmArm1(1)-obj.cmTot(1))*(obj.cmArm1(2)-obj.cmTot(2)),  (obj.cmArm1(1)-obj.cmTot(1))^2+(obj.cmArm1(3)-obj.cmTot(3))^2, -(obj.cmArm1(2)-obj.cmTot(2))*(obj.cmArm1(3)-obj.cmTot(3)); ...
                          -(obj.cmArm1(1)-obj.cmTot(1))*(obj.cmArm1(3)-obj.cmTot(3)),  -(obj.cmArm1(2)-obj.cmTot(2))*(obj.cmArm1(3)-obj.cmTot(3))   , (obj.cmArm1(1)-obj.cmTot(1))^2+(obj.cmArm1(2)-obj.cmTot(2))^2] + ...
            ...
                        RPY2Rot([0, 0 ,obj.alpha(2)]) * obj.I_arm2 * RPY2Rot([0, 0 ,obj.alpha(2)])' + ...
            ...                
                obj.m_a*[(obj.cmArm2(2)-obj.cmTot(2))^2+(obj.cmArm2(3)-obj.cmTot(3))^2,   -(obj.cmArm2(1)-obj.cmTot(1))*(obj.cmArm2(2)-obj.cmTot(2)), -(obj.cmArm2(1)-obj.cmTot(1))*(obj.cmArm2(3)-obj.cmTot(3)); ...
                          -(obj.cmArm2(1)-obj.cmTot(1))*(obj.cmArm2(2)-obj.cmTot(2)),  (obj.cmArm2(1)-obj.cmTot(1))^2+(obj.cmArm2(3)-obj.cmTot(3))^2, -(obj.cmArm2(2)-obj.cmTot(2))*(obj.cmArm2(3)-obj.cmTot(3)); ...
                          -(obj.cmArm2(1)-obj.cmTot(1))*(obj.cmArm2(3)-obj.cmTot(3)),  -(obj.cmArm2(2)-obj.cmTot(2))*(obj.cmArm2(3)-obj.cmTot(3))   , (obj.cmArm2(1)-obj.cmTot(1))^2+(obj.cmArm2(2)-obj.cmTot(2))^2] + ...
            ...
                        RPY2Rot([0, 0 , obj.alpha(3)]) * obj.I_arm3 * RPY2Rot([0, 0 , obj.alpha(3)])' + ...
            ...
                obj.m_a*[(obj.cmArm3(2)-obj.cmTot(2))^2+(obj.cmArm3(3)-obj.cmTot(3))^2,   -(obj.cmArm3(1)-obj.cmTot(1))*(obj.cmArm3(2)-obj.cmTot(2)), -(obj.cmArm3(1)-obj.cmTot(1))*(obj.cmArm3(3)-obj.cmTot(3)); ...
                          -(obj.cmArm3(1)-obj.cmTot(1))*(obj.cmArm3(2)-obj.cmTot(2)),  (obj.cmArm3(1)-obj.cmTot(1))^2+(obj.cmArm3(3)-obj.cmTot(3))^2, -(obj.cmArm3(2)-obj.cmTot(2))*(obj.cmArm3(3)-obj.cmTot(3)); ...
                          -(obj.cmArm3(1)-obj.cmTot(1))*(obj.cmArm3(3)-obj.cmTot(3)),  -(obj.cmArm3(2)-obj.cmTot(2))*(obj.cmArm3(3)-obj.cmTot(3))   , (obj.cmArm3(1)-obj.cmTot(1))^2+(obj.cmArm3(2)-obj.cmTot(2))^2] + ...
            ...
                        RPY2Rot([0, 0 , obj.alpha(4)]) * obj.I_arm4 * RPY2Rot([0, 0 , obj.alpha(4)])' + ...
            ...
                obj.m_a*[(obj.cmArm4(2)-obj.cmTot(2))^2+(obj.cmArm4(3)-obj.cmTot(3))^2,   -(obj.cmArm4(1)-obj.cmTot(1))*(obj.cmArm4(2)-obj.cmTot(2)), -(obj.cmArm4(1)-obj.cmTot(1))*(obj.cmArm4(3)-obj.cmTot(3)); ...
                          -(obj.cmArm4(1)-obj.cmTot(1))*(obj.cmArm4(2)-obj.cmTot(2)),  (obj.cmArm4(1)-obj.cmTot(1))^2+(obj.cmArm4(3)-obj.cmTot(3))^2, -(obj.cmArm4(2)-obj.cmTot(2))*(obj.cmArm4(3)-obj.cmTot(3)); ...
                          -(obj.cmArm4(1)-obj.cmTot(1))*(obj.cmArm4(3)-obj.cmTot(3)),  -(obj.cmArm4(2)-obj.cmTot(2))*(obj.cmArm4(3)-obj.cmTot(3))   , (obj.cmArm4(1)-obj.cmTot(1))^2+(obj.cmArm4(2)-obj.cmTot(2))^2] + ...
                          obj.I_body;
              % disp(obj.I_tot);
        end

     


        function obj = EvalEOM(obj) % 운동방정식 계산
            %update x dot with EO
            obj.EvalItot(); % calculate I
            obj.EvalF_a_b(); % calculate F function
            obj.EvalTau_a_b(); % calculate Tau function
            obj.dx_prev = obj.dx;
            
            
            bRi = RPY2Rot(obj.euler);
            R = bRi';
            obj.dx(1:3) = obj.dr;
            obj.dx(4:6) = 1 / obj.m_t * ([0; 0; obj.m_t*obj.g] + R * obj.F_a_b * obj.w_m);  % no wb3 input

            phi = obj.euler(1); theta = obj.euler(2);
       
            obj.dx(10:12) = [ 1  sin(phi)*tan(theta)  cos(phi)*tan(theta); 
                            0  cos(phi)             -sin(phi);
                            0  sin(phi)*sec(theta)  cos(phi)*sec(theta)] * obj.w;
            obj.dx(13:15) = (obj.I_tot) \ (obj.Tau_a_b * obj.w_m - cross(obj.w, obj.I_tot * obj.w));  % no wb3 input 
            
        end
        %update state base on EOM
        function obj = UpdateState(obj)
            obj.t = obj.t + obj.dt;
            
            obj.alpha = obj.alpha + obj.dalpha .* obj.dt;
            obj.beta(3) = obj.beta(3) + obj.w_b .* obj.dt;
            % disp(obj.alpha)
            obj.EvalEOM();
            
            obj.x(1:6) = obj.x(1:6) + obj.dx(1:6) .* obj.dt;
            obj.x(10:15) = obj.x(10:15) + obj.dx(10:15) .* obj.dt;
            obj.dx(7:9) = (obj.dx(4:6) - obj.dx_prev(4:6))./obj.dt;
            obj.dx(16:18) = (obj.dx(13:15) - obj.dx_prev(13:15))./obj.dt;
            % disp(obj.dx)
            % can use runge-kutta methods for x update(better accuracy)
            obj.r = obj.x(1:3);
            obj.dr = obj.x(4:6);
            obj.euler = obj.x(10:12);
            obj.w = obj.x(13:15);
                       
            obj.drone1_body = [0.15*cos(obj.alpha(1))+0.05,        0.15*sin(obj.alpha(1)),      0,     1; ...
                                   -0.15*sin(obj.alpha(2)),   0.15*cos(obj.alpha(2))+0.05,      0,     1; ...
                              -0.15*cos(obj.alpha(3))-0.05,       -0.15*sin(obj.alpha(3)),      0,     1; ...
                                    0.15*sin(obj.alpha(4)),  -0.15*cos(obj.alpha(4))-0.05,      0,     1;...
                               0,                                  0,                           0,     1;...%payload at 0,0,0
                               0,                                  0,                       -0.15,     1]';

            
    
            if obj.t > 0.2
                obj.StartMorph();
            end
                
            
        end

        function obj = StartMorph(obj)
    % 일정한 각속도를 적용할 값 (deg/s -> rad/s)
    flareangleRate = (pi/180) * 60; 

    % 부호가 바뀐 적이 있는지 추적하는 배열 (4x1)
    % MATLAB 함수 내부에서 변수를 기억하기 위해 'persistent'를 사용
    persistent signSwitched
    if isempty(signSwitched)
        signSwitched = false(4,1); 
    end

    for i = 1:4
        alphaDiff = obj.targetalpha(i) - obj.alpha(i);
        newSign   = sign(alphaDiff);     % 현재 에러의 부호
        oldSign   = sign(obj.dalpha(i)); % 이전 프레임의 dalpha 부호

        if signSwitched(i)
            % 이미 부호가 한 번이라도 바뀌었다면 계속 0으로 유지
            obj.dalpha(i) = 0;
            continue
        end
        
        if (oldSign ~= 0) && (newSign ~= 0) && (oldSign ~= newSign)
            % 부호가 바뀌었으면 이후 계속 0
            obj.dalpha(i)      = 0;
            signSwitched(i)    = true;
        else
            % 부호가 안 바뀌었다면 일정 속도로 계속 진행
            obj.dalpha(i) = flareangleRate * newSign;
        end
    end
            % disp('current alpha');
            % disp(obj.alpha);
            % disp('target alpha');
            % disp(obj.targetalpha);
end

        
        %CONTROLLER
        function obj = AttitudeCtrl(obj, refSig)

            obj.x_des = refSig('x_des');
            obj.y_des = refSig('y_des');
            obj.z_des = refSig('z_des');

            obj.phi_des = refSig('phi_des');
            obj.theta_des = refSig('theta_des');
            obj.psi_des = refSig('psi_des');

            obj.dx_des = refSig('xdot_des');
            obj.dy_des = refSig('ydot_des');
            obj.dz_des = refSig('zdot_des');

            obj.dphi_des = refSig('phidot_des');
            obj.dtheta_des = refSig('thetadot_des');
            obj.dpsi_des = refSig('psidot_des');

            obj.ddx_des = refSig('xddot_des');
            obj.ddy_des = refSig('yddot_des');
            obj.ddz_des = refSig('zddot_des');

            obj.ddphi_des = refSig('phiddot_des');
            obj.ddtheta_des = refSig('thetaddot_des');
            obj.ddpsi_des = refSig('psiddot_des');

            x_current = obj.x;

            x_ref = zeros(18,1);
            x_ref(1) = obj.x_des; 
            x_ref(2) = obj.y_des;
            x_ref(3) = obj.z_des;

            x_ref(7) = obj.dx_des;
            x_ref(8) = obj.dy_des;
            x_ref(9) = obj.dz_des;

            x_ref(13) = obj.ddx_des;
            x_ref(14) = obj.ddy_des;
            x_ref(15) = obj.ddz_des;

            x_ref(4) = obj.phi_des;
            x_ref(5) = obj.theta_des;
            x_ref(6) = obj.psi_des;

            x_ref(10) = obj.dphi_des;
            x_ref(11) = obj.dtheta_des;
            x_ref(12) = obj.dpsi_des;

            x_ref(16) = obj.ddphi_des;
            x_ref(17) = obj.ddtheta_des;
            x_ref(18) = obj.ddpsi_des;
            
            x_error = x_current - x_ref;    % 18x1
            obj.v_lqr   = - obj.K_lqr * x_error;
            disp(obj.v_lqr);

            obj.EvaldI();
            obj.EvalF_a_b();
            obj.EvalTau_a_b();

            obj.JR = [RPY2Rot(obj.euler), zeros(3,3);...
                      zeros(3,3), inv(obj.I_tot)];
            obj.JRdot = [RPY2Rot_derivative(obj.euler,obj.w), zeros(3,3);...
                         zeros(3,3),         obj.dI_totinv];

            obj.J_beta = [obj.F_a_b;...
                          obj.Tau_a_b];
            obj.J_betadot = [0,  0,  0,  0;...
                           0,  0,  -obj.kf*cos(obj.beta(3)), 0;...
                           0,  0,   obj.kf*sin(obj.beta(3)), 0;...
                           0,  0,  0,  0;...
                           0,  0,  obj.kf*(obj.bl+obj.al)*sin(obj.beta(3)) - obj.km*cos(obj.beta(3)), 0;...
                           0,  0,  obj.kf*(obj.bl+obj.al)*cos(obj.beta(3)) + obj.km*sin(obj.beta(3)), 0];
            % disp('JR');
            % disp(size(obj.JR));
            % disp('JRdot');
            % disp(size(obj.JRdot));
            % disp('J_beta');
            % disp(size(obj.J_beta));
            % disp('J_betadot');
            % disp(size(obj.J_betadot));

           B = [obj.JR * obj.J_beta , obj.JR * obj.J_betadot * (obj.w_m)];
           obj.u_control = pinv(B)*(obj.v_lqr - obj.JRdot*obj.J_beta * obj.w_m);
           disp(obj.u_control);

           obj.w_m = obj.w_m + obj.u_control(1:4).*obj.dt;
           obj.w_b = obj.u_control(5);




          





            % obj.phi_des = refSig(1);
            % obj.theta_des = refSig(2);
            % obj.psi_des = refSig(3);
            % obj.zdot_des = refSig(4);
            % 
            % 
            % obj.phi_err = obj.phi_des - obj.euler(1);
            % obj.theta_err = obj.theta_des - obj.euler(2);
            % obj.psi_err = obj.psi_des - obj.euler(3);
            % obj.zdot_err = obj.zdot_des - obj.dr(3);
            

            




            % 시뮬레이션 환경 검증용 PID control
            % obj.u(2) = (obj.KP_phi * obj.phi_err+ ...
            %             obj.KI_phi * obj.phi_err_sum+ ...
            %             obj.KD_phi * (obj.phi_err - obj.phi_err_prev)/obj.dt);
            % 
            % obj.phi_err_sum = obj.phi_err_sum + obj.phi_err;
            % obj.phi_err_prev = obj.phi_err;
            % 
            % obj.u(3) = (obj.KP_theta * obj.theta_err+ ...
            %             obj.KI_theta * obj.theta_err_sum+ ...
            %             obj.KD_theta * (obj.theta_err - obj.theta_err_prev)/obj.dt);
            % 
            % obj.theta_err_sum = obj.theta_err_sum + obj.theta_err;
            % obj.theta_err_prev = obj.theta_err;
            % 
            % obj.u(4) = (obj.KP_psi * obj.psi_err+ ...
            %             obj.KI_psi * obj.psi_err_sum+ ...
            %             obj.KD_psi * (obj.psi_err - obj.psi_err_prev)/obj.dt);
            % 
            % obj.psi_err_sum = obj.psi_err_sum + obj.psi_err;
            % obj.psi_err_prev = obj.psi_err;
            % 
            % obj.u(1) = obj.m_t*obj.g - (obj.KP_zdot * obj.zdot_err+ ...
            %             obj.KI_zdot * obj.zdot_err_sum+ ...
            %             obj.KD_zdot * (obj.zdot_err - obj.zdot_err_prev)/obj.dt);
            % 
            % obj.zdot_err_sum = obj.zdot_err_sum + obj.zdot_err;
            % obj.zdot_err_prev = obj.zdot_err;
            % 
            % 
            % 
            % obj.T = obj.u(1);
            % obj.M = obj.u(2:4);
            
        end


        

    end


  
end

