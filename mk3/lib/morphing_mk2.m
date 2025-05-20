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
        p2x

        F_a_b
        Tau_a_b
        JR
        JRdot   % feedback linearization mapping
        J_beta
        J_betadot
        Trq

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
        myPath
        pathIndex

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
        matA
        matB
        
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

        z_error
        z
        dz
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

        % Kalman Filter properties
        zx_est
        P
        Q
        R_imu
        x_pred       % 칼만 filter 사전 예측 상태 (12×1)
        lastImu1     % 마지막으로 사용된 IMU1 측정값
        lastImu2     % 마지막으로 사용된 IMU2 측정값
        % --- GPS 관련 추가 ---
        lastGPS     % 마지막 GPS 위치 측정값 [x;y;z]
        z_k
        accelormeter

    end
    %% METHODS
    methods
        %% CONSTRUCTOR
        function obj = morphing_mk2(params, initStates, initInputs, initFlare, targetFlare, initTilt, path, simTime)
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

            % path
            obj.myPath = path;
            obj.pathIndex = 1;

                          

            % state vectors
            obj.x = initStates;
            obj.r = obj.x(1:3); 
            obj.dr = obj.x(4:6);
            obj.ddr = obj.x(7:9);
            obj.euler = obj.x(10:12);
            obj.w = obj.x(13:15);
            obj.dw = obj.x(16:18);
            obj.z = initStates;
            
            
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
            obj.w_b = obj.u(5:8);    % wb3 (tilt motor omega)
            
            


            obj.z_error = zeros(18,1);     
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

            % rotation matrix for + to x
            obj.p2x = [cosd(-45), -sind(-45), 0;
                        sind(-45),  cosd(-45), 0;
                        0      ,      0      , 1];
            % for kalman filter
            obj.zx_est = obj.x(1:18);
            obj.P = eye(18) * 1e-4;
            
            % 프로세스 노이즈
            obj.Q = eye(18)* 1e-8;
            % 두 imu 합쳐서 12x12 측정 노이즈 공분산
            sigma_acc = 0.1;
            sigma_gyro = 0.001;
            sigma_gps = 0.003;  % 예: 0.3 m 표준편차
            sigma_mag = 0.003;
            R_gps  = sigma_gps^2 * eye(3);
            R_acc = sigma_acc^2 * eye(3);
            R_gyro = sigma_gyro^2 * eye(3);
            R_euler_acc = 0.0003^2 *eye(2);
            R_mag = sigma_mag^2; 
            obj.R_imu = blkdiag(R_gps,R_acc,R_euler_acc,R_mag, R_gyro);
            obj.lastGPS   = obj.x(1:3);  % 초기 위치
            obj.z_k = [0,0,0,0,0,0,0,0,0,0,0,0]';
           
        end
        

        %%
    function obj = EKFPredict(obj)
        A = [eye(6), obj.dt*eye(6), (1/2)*obj.dt^2*eye(6);
             zeros(6,6), eye(6),     obj.dt*eye(6);
             zeros(6,6), zeros(6,6), eye(6)];
        
        B = [(1/6)*obj.dt^3*eye(6);
            (1/2)*obj.dt^2*eye(6);
            obj.dt*eye(6)];

        % feedback linearization domain 에서 진행
        v      = obj.v_lqr;               % 4×1 모터 속도 입력
        obj.zx_est = A*obj.zx_est + B*v; % 상태 예측
        obj.P     = A*obj.P*A.' + obj.Q;% 공분산 예측
        disp('Kalman A')
        disp(A)
    end

    %% EKF 측정갱신 (두 IMU)
    function obj = EKFUpdateIMU(obj, imu1, imu2, gps, mag, euler_acc)
        % 두 IMU를 상보필터로 융합, 실제 센서 사용시 가속도 센서와 자이로 도 융합 필요 (roll,pitch)
        % 센서 데이터 생성시 true 값을 이용, 실제 센서 이용시
        % 가속도값은 body frame 각속도는 omega로 나타날거임 
        % 여기서 측정값과 상태의 비선형성이 생김 야코비안 필요
        % 현재는 우선 true값으로 잡고 각속도 항만 야코비안 도입
        alph = 0.98;
        % 가속도 저주파 합: acc_f = α·acc1 + (1−α)·acc2
        acc_f  = alph * imu1.acc  + (1-alph) * imu2.acc;
        % 자이로 고주파 합: gyro_f= α·gyro1+ (1−α)·gyro2
        gyro_f = alph * imu1.gyro + (1-alph) * imu2.gyro;

        % phi_acc   = atan2( acc_f(2),                 acc_f(3) );
        % theta_acc = atan2(-acc_f(1), sqrt(acc_f(2)^2 + acc_f(3)^2));


        obj.z_k = [gps', euler_acc', mag, gyro_f',acc_f' ]';
        H = [eye(3),    zeros(3,15);
             zeros(3,3), eye(3), zeros(3,12);
             zeros(3,9), eye(3), zeros(3,6);
             zeros(3,12),  eye(3), zeros(3,3)];

        


    
    
        % --- 칼만 이득 계산 & 업데이트 ---
        S = H*obj.P*H.' + obj.R_imu;
        K = obj.P * H.' / S;
        obj.zx_est = obj.zx_est + K*(obj.z_k - H*obj.zx_est);
        obj.P     = (eye(18)-K*H)*obj.P;
    end

        function obj = SetLQR_Gain(obj)
             %%Linear model

            matA = [zeros(6,6), eye(6), zeros(6,6);...
            zeros(6,6), zeros(6,6), eye(6);...
            zeros(6,6), zeros(6,6), zeros(6,6)];
            obj.matA = matA;

            matB = [zeros(12,6);...
            eye(6)];
            obj.matB = matB;
            matC = eye(18);

            matD = zeros(6,6);
            
            

            n = size(matA,1);
            m = size(matB,2);
            if obj.t>100
            Q = [eye(3), zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,3);...
                 zeros(3,3), 20*eye(3), zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,3);...
                 zeros(3,3), zeros(3,3), eye(3), zeros(3,3), zeros(3,3), zeros(3,3);...
                 zeros(3,3), zeros(3,3), zeros(3,3), 20*eye(3), zeros(3,3), zeros(3,3);...
                 zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,3), eye(3), zeros(3,3);...
                 zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,3), 20*eye(3)];
            else
            Q = [20*eye(3), zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,3);...
                 zeros(3,3), eye(3), zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,3);...
                 zeros(3,3), zeros(3,3), eye(3), zeros(3,3), zeros(3,3), zeros(3,3);...
                 zeros(3,3), zeros(3,3), zeros(3,3), eye(3), zeros(3,3), zeros(3,3);...
                 zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,3), eye(3), zeros(3,3);...
                 zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,3), zeros(3,3), eye(3)];
            end
            R = eye(m);

            [K, S, e] = lqr(matA, matB, Q, R);
            obj.K_lqr = K;  % 객체 속성에 저장

            disp('LQR Gain K =');
            disp(K);
        end
%% 
        
        function obj = EvalF_a_b(obj) % Translation motion mixing matrix
            obj.F_a_b = obj.p2x*( [ 0,                                                            obj.kf*sin(obj.beta(1)),                   -obj.kf*cos(obj.beta(1)); ...
                         -obj.kf*cos(obj.alpha(2))*sin(obj.beta(2)), -obj.kf*sin(obj.alpha(2))*sin(obj.beta(2)),                   -obj.kf*cos(obj.beta(2)); ...
                          0,                                         -obj.kf*sin(obj.beta(3)),                                     -obj.kf*cos(obj.beta(3)); ...
                          obj.kf*cos(obj.alpha(4))*sin(obj.beta(4)),  obj.kf*sin(obj.alpha(4))*sin(obj.beta(4)),                   -obj.kf*cos(obj.beta(4))]'); 
            
            % F = obj.F_a_b * obj.w_m;
            % disp('F');
            % disp(F);
            
        end

        function obj = EvalTau_a_b(obj) % Rotation motion mixing matrix
            obj.Tau_a_b = obj.p2x*([                                        0,                                                                      obj.kf*(obj.bl+obj.al)*cos(obj.beta(1)) + obj.km*sin(obj.beta(1)),                                                                                      obj.kf*(obj.bl+obj.al)*sin(obj.beta(1)) - obj.km*cos(obj.beta(1)); ...
                           -obj.kf*(obj.bl+obj.al*cos(obj.alpha(2)))*cos(obj.beta(2))+obj.km*cos(obj.alpha(2))*sin(obj.beta(2)),          -obj.kf*obj.al*sin(obj.alpha(2))*cos(obj.beta(2))+obj.km*sin(obj.alpha(2))*sin(obj.beta(2)),                    obj.kf*obj.al*(sin(obj.alpha(2)))^2*sin(obj.beta(2))+obj.kf*(obj.bl+obj.al*cos(obj.alpha(2)))*cos(obj.alpha(2))*sin(obj.beta(2))+ obj.km*cos(obj.beta(2)); ...
                                                                   0,  -obj.kf*(obj.bl+obj.al)*cos(obj.beta(3)) - obj.km*sin(obj.beta(3)),  obj.kf*(obj.bl+obj.al)*sin(obj.beta(3)) - obj.km*cos(obj.beta(3)); ...
                            obj.kf*(obj.bl+obj.al*cos(obj.alpha(4)))*cos(obj.beta(4))-obj.km*cos(obj.alpha(4))*sin(obj.beta(4)),             obj.kf*obj.al*sin(obj.alpha(4))*cos(obj.beta(4))-obj.km*sin(obj.alpha(4))*sin(obj.beta(4)),                                                            obj.kf*obj.al*(sin(obj.alpha(4)))^2*sin(obj.beta(4))+obj.kf*(obj.bl+obj.al*cos(obj.alpha(4)))*cos(obj.alpha(4))*sin(obj.beta(4))+obj.km*cos(obj.beta(4))]');
            
            Trq = obj.Tau_a_b * obj.w_m;
            disp('Torque');
            disp(Trq);
            
            % disp('Tau function');
            % disp(obj.Tau_a_b);
        end

        function obj = EvalfailsafeTau(obj)
            obj.EvalCM();
            lcm = obj.cmTot(1);
            % disp(lcm);
            obj.Tau_a_b = [0,                                                                                                        obj.kf*(obj.bl+obj.al-lcm),                                                                                 -obj.km; ...
                     -obj.kf*(obj.bl+obj.al*cos(obj.alpha(2)))*cos(obj.beta(2)) + obj.km*cos(obj.alpha(2))*sin(obj.beta(2)),     -obj.kf*(obj.al*sin(obj.alpha(2))+lcm)*cos(obj.beta(2))+obj.km*sin(obj.alpha(2))*sin(obj.beta(2)),            obj.kf*(obj.al*sin(obj.alpha(2))+lcm)*sin(obj.alpha(2))*sin(obj.beta(2))+obj.kf*(obj.bl+obj.al*cos(obj.alpha(2)))*cos(obj.alpha(2))*sin(obj.beta(2))+ obj.km*cos(obj.beta(2)); ...
                                                                   0,                                                          -obj.kf*(obj.bl+obj.al+lcm)*cos(obj.beta(3)) - obj.km*sin(obj.beta(3)),                                       obj.kf*(obj.bl+obj.al+lcm)*sin(obj.beta(3)) - obj.km*cos(obj.beta(3)); ...
                      obj.kf*(obj.bl+obj.al*cos(obj.alpha(4)))*cos(obj.beta(4))-obj.km*cos(obj.alpha(4))*sin(obj.beta(4)),      obj.kf*(obj.al*sin(obj.alpha(4))-lcm)*cos(obj.beta(4))-obj.km*sin(obj.alpha(4))*sin(obj.beta(4)),            obj.kf*(obj.al*sin(obj.alpha(4))-lcm)*sin(obj.alpha(4))*sin(obj.beta(4))+obj.kf*(obj.bl+obj.al*cos(obj.alpha(4)))*cos(obj.alpha(4))*sin(obj.beta(4))+obj.km*cos(obj.beta(4))]';        
           
        end

        function obj = EvaldI(obj)
            
            
            a = inv(obj.I_tot);
            
            b = inv(obj.I_tot_prev);
            
              obj.dI_totinv = (a-b)/obj.dt;
              % disp(obj.dI_totinv);


        end



        function state = GetState(obj) % main function에 상태 반환
            state.x = obj.x;
            state.flare = obj.alpha;
            state.body = obj.drone1_body;
            state.tilt = obj.beta;
            state.w_m = obj.w_m;
            state.Trq = obj.Trq;
            % EKF 추정치
            state.x_pred   = obj.x_pred;     % 칼만필터 사전 예측
            state.z_k      = obj.z_k;    % 센서 측정치, gps, gyro
            state.zx_est   = obj.zx_est;   % 12×1 추정 상태
            state.P       = obj.P;       % 12×12 공분산 (필요 시)
            state.gps     = obj.lastGPS;

        end

        function obj = EvalCM(obj) % 무게중심 계산

            obj.cmArm1 = [obj.acml*cos(obj.alpha(1))+obj.bl, obj.acml*sin(obj.alpha(1)), 0]';
            obj.cmArm2 = [-obj.acml*sin(obj.alpha(2)), obj.acml*cos(obj.alpha(2))+obj.bl,  0]';
            obj.cmArm3 = [-obj.acml*cos(obj.alpha(3))-obj.bl, -obj.acml*sin(obj.alpha(3)), 0]';
            obj.cmArm4 = [obj.acml*sin(obj.alpha(4)),-obj.acml*cos(obj.alpha(4))-obj.bl, 0]';
            obj.cmTot = obj.p2x*((obj.m_a * obj.cmArm1 + obj.m_a * obj.cmArm2 + obj.m_a * obj.cmArm3 + obj.m_a * obj.cmArm4)/(obj.m_t));            
        end

        function obj = EvalItot(obj) % 관성모멘트 계산

            obj.I_tot_prev = obj.I_tot;
            obj.EvalCM();
            
            obj.I_tot = obj.p2x * (RPY2Rot([0, 0 ,obj.alpha(1)]) * obj.I_arm1 * RPY2Rot([0, 0 ,obj.alpha(1)])' + ...
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
                          obj.I_body) * obj.p2x';
               
        end

     


        function obj = EvalEOM(obj) % 운동방정식 계산
            %update x dot with EO
            obj.EvalItot(); % calculate I
            obj.EvalF_a_b(); % calculate F function
            obj.EvalTau_a_b(); % calculate Tau function
            obj.dx_prev = obj.dx;

            disp('w_m');
            disp(obj.w_m);
            
           
            bRi = RPY2Rot(obj.euler);
            R = bRi';
            obj.dx(1:3) = obj.dr;
            obj.dx(4:6) = 1 / obj.m_t * ([0; 0; obj.m_t*obj.g] + R * obj.F_a_b * obj.w_m);  % no wb3 input
            obj.accelormeter = R'*obj.dx(4:6)- (1 / obj.m_t)* obj.F_a_b * obj.w_m;
            disp('F');
            disp([0; 0; obj.m_t*obj.g] + R * obj.F_a_b * obj.w_m);
            
            phi = obj.euler(1); theta = obj.euler(2);
            
            obj.dx(10:12) = [ 1  sin(phi)*tan(theta)  cos(phi)*tan(theta); 
                            0  cos(phi)             -sin(phi);
                            0  sin(phi)*sec(theta)  cos(phi)*sec(theta)] * obj.w;
            obj.dx(13:15) = (obj.I_tot) \ (obj.Tau_a_b * obj.w_m - cross(obj.w, obj.I_tot * obj.w));  % no wb3 input 

            obj.Trq = obj.Tau_a_b * obj.w_m;
            
        end

        function obj = EvalfailsafeEOM(obj)
            obj.EvalItot(); % calculate I
            obj.EvalF_a_b(); % calculate F function
            obj.EvalfailsafeTau(); % calculate Tau function
            obj.dx_prev = obj.dx;

            disp('w_m');
            disp(obj.w_m);
            
           
            bRi = RPY2Rot(obj.euler);
            R = bRi';
            obj.dx(1:3) = obj.dr;
            obj.dx(4:6) = 1 / obj.m_t * ([0; 0; obj.m_t*obj.g] + R * obj.F_a_b * obj.w_m);  % no wb3 input
            obj.accelormeter = R'*obj.dx(4:6)-(1 / obj.m_t)* obj.F_a_b * obj.w_m;
            disp('F');
            disp([0; 0; obj.m_t*obj.g] + R * obj.F_a_b * obj.w_m);
            
            phi = obj.euler(1); theta = obj.euler(2);
            
            obj.dx(10:12) = [ 1  sin(phi)*tan(theta)  cos(phi)*tan(theta); 
                            0  cos(phi)             -sin(phi);
                            0  sin(phi)*sec(theta)  cos(phi)*sec(theta)] * obj.w;
            obj.dx(13:15) = (obj.I_tot) \ (obj.Tau_a_b * obj.w_m - cross(obj.w, obj.I_tot * obj.w));  % no wb3 input 

            obj.Trq = obj.Tau_a_b * obj.w_m;
            
            
        end
        %update state base on EOM
        function obj = UpdateState(obj)
            obj.t = obj.t + obj.dt;
            
            obj.alpha = obj.alpha + obj.dalpha .* obj.dt;
            obj.beta(1:4) = obj.beta(1:4) + obj.w_b .* obj.dt;
            
            % disp(obj.alpha)

            
            if obj.t>100   % for failsafe controller
                obj.EvalfailsafeEOM();
            else           % for attitude controller 
                obj.EvalEOM();
            end
             
            
            

            
            obj.x = obj.x + obj.dx .* obj.dt;
            obj.x(7:9) = obj.dx(4:6);
            obj.x(16:18) = obj.dx(13:15);
           
            % disp(obj.dx)
            % can use runge-kutta methods for x update(better accuracy)
            obj.r = obj.x(1:3);
            obj.dr = obj.x(4:6);
            obj.ddr = obj.x(7:9);
            obj.euler = obj.x(10:12);
            obj.w = obj.x(13:15);
            obj.dw = obj.x(16:18);
                       
            obj.drone1_body = [obj.p2x, [0,0,0]' ; 0, 0, 0, 1]*[0.15*cos(obj.alpha(1))+0.05,        0.15*sin(obj.alpha(1)),      0,     1; ...
                                   -0.15*sin(obj.alpha(2)),   0.15*cos(obj.alpha(2))+0.05,      0,     1; ...
                              -0.15*cos(obj.alpha(3))-0.05,       -0.15*sin(obj.alpha(3)),      0,     1; ...
                                    0.15*sin(obj.alpha(4)),  -0.15*cos(obj.alpha(4))-0.05,      0,     1;...
                               0,                                  0,                           0,     1;...%payload at 0,0,0
                               0,                                  0,                       -0.15,     1]';

            
    
            if obj.t > 100
                obj.StartMorph();
            end

            % 1) 시뮬레이션 IMU 데이터 생성
            [imu1, imu2] = obj.SimulateIMU();
            gps = obj.SimulateGPS();
            mag = obj.SimulateMag();
            euler_acc = obj.SimulateAcc_euler();
            % → 측정값 저장
            obj.lastImu1 = imu1;
            obj.lastImu2 = imu2;
            % … (상태 x 업데이트 끝난 후) …
            obj = obj.EKFPredict();
            obj.x_pred = obj.zx_est;
            obj = obj.EKFUpdateIMU( imu1, imu2, gps, mag, euler_acc);    
            
        end

        function obj = StartMorph(obj)
    % 일정한 각속도를 적용할 값 (deg/s -> rad/s)
    flareangleRate = (pi/180) * 30; 

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

        function obj = StopMotor(obj)
            persistent startTime w_m1_init  % 초기 값들 저장
        if isempty(startTime)
            % 첫 호출 시점에만 기록
            startTime  = obj.t;
            w_m1_init  = obj.w_m(1);  % 모터1 초기 회전수 저장
        end

        rampTime = 1;  % 5초 동안 선형 감속

        elapsed = obj.t - startTime;  % 감속 시작 후 경과 시간
        if elapsed < rampTime
            ratio = elapsed / rampTime;
            % 서서히 줄이기: 시작값에서 linear하게 0으로
            obj.w_m(1) = (1 - ratio) * w_m1_init;
        else
            % rampTime이 지난 뒤에는 완전히 0
            obj.w_m(1) = 0;
        end
        end

        
        %CONTROLLER 
        function obj = AttitudeCtrl(obj)

                     
            

            obj.x_des = 2;
            obj.y_des = 2;
            obj.z_des = -2;
            obj.phi_des   = 0*pi/180;
            obj.theta_des = 0*pi/180;
            obj.psi_des   = 0*pi/180;
                        
            
            %% 실제 코드

            % 매핑 인덱스 정의 (obj.x의 인덱스를 z_current의 인덱스에 대응)
            mapping = [1 2 3 10 11 12 4 5 6 13 14 15 7 8 9 16 17 18];

            %%
            %새로운 z_current 생성
            
            z_current = obj.x(mapping);
            z_ref     = zeros(18,1);
            z_ref(1)  = obj.x_des;
            z_ref(2)  = obj.y_des;
            z_ref(3)  = obj.z_des;
            z_ref(4)  = obj.phi_des;
            z_ref(5)  = obj.theta_des;
            z_ref(6)  = obj.psi_des;

            disp(z_ref);
            
            

            obj.v_lqr   =  obj.K_lqr * (z_ref-z_current);
            
            disp('obj.v_lqr');
            disp(obj.v_lqr);

            obj.EvaldI();
            obj.EvalF_a_b();
            obj.EvalTau_a_b(); 

            % 회전 매트릭스 transpose 필요 JR dot 행렬의 I matrix 미분항 0으로 설정
            obj.JR = [(1/obj.m_t)*RPY2Rot(obj.euler)', zeros(3,3);...
                      zeros(3,3), inv(obj.I_tot)];
            obj.JRdot = [(1/obj.m_t)*RPY2Rot_derivative(obj.euler,obj.w), zeros(3,3);...
                         zeros(3,3),      zeros(3,3)    ];
            % disp(obj.dI_totinv);

            obj.J_beta = [obj.F_a_b;...
                          obj.Tau_a_b];
            obj.J_betadot  = [obj.p2x*[0,  -obj.kf*cos(obj.alpha(2))*cos(obj.beta(2))*obj.w_m(2), 0,  obj.kf*cos(obj.alpha(4))*cos(obj.beta(4))*obj.w_m(4); 
                              obj.kf*cos(obj.beta(1))*obj.w_m(1),  -obj.kf*sin(obj.alpha(2))*cos(obj.beta(2))*obj.w_m(2), -obj.kf*cos(obj.beta(3))*obj.w_m(3),  obj.kf*sin(obj.alpha(4))*cos(obj.beta(4))*obj.w_m(4);
                              obj.kf*sin(obj.beta(1))*obj.w_m(1),  obj.kf*sin(obj.beta(2))*obj.w_m(2),           obj.kf*sin(obj.beta(3))*obj.w_m(3),  obj.kf*sin(obj.beta(4))*obj.w_m(4)];

                                obj.p2x*[0, ...
                                obj.kf*(obj.bl+obj.al*cos(obj.alpha(2)))*sin(obj.beta(2))*obj.w_m(2) + obj.km*cos(obj.alpha(2))*cos(obj.beta(2))*obj.w_m(2), ...     % dTau function 1st row
                                0, ...
                                -obj.kf*(obj.bl+obj.al*cos(obj.alpha(4)))*sin(obj.beta(4))*obj.w_m(4) - obj.km*cos(obj.alpha(4))*cos(obj.beta(4))*obj.w_m(4); ...
    
                                (-obj.kf*(obj.bl+obj.al)*sin(obj.beta(1)) + obj.km*cos(obj.beta(1)))*obj.w_m(1), ...
                                (obj.kf*obj.al*sin(obj.alpha(2))*sin(obj.beta(2))*obj.w_m(2) + obj.km*sin(obj.alpha(2))*cos(obj.beta(2)))*obj.w_m(2), ...            % dTau function 2nd row
                                (obj.kf*(obj.bl+obj.al)*sin(obj.beta(3)) - obj.km*cos(obj.beta(3)))*obj.w_m(3), ...
                                -(obj.kf*obj.al*sin(obj.alpha(4))*sin(obj.beta(4)) + obj.km*sin(obj.alpha(4))*cos(obj.beta(4)))*obj.w_m(4); ...
    
                                (obj.kf*(obj.bl+obj.al)*cos(obj.beta(1)) + obj.km*sin(obj.beta(1)))*obj.w_m(1), ...
                                (obj.kf*obj.al*(sin(obj.alpha(2)))^2*cos(obj.beta(2)) + obj.kf*(obj.bl+obj.al*cos(obj.alpha(2)))*cos(obj.alpha(2))*cos(obj.beta(2)) - obj.km*sin(obj.beta(2)))*obj.w_m(2), ...      % dTau function 3rd row
                                (obj.kf*(obj.bl+obj.al)*cos(obj.beta(3)) + obj.km*sin(obj.beta(3)))*obj.w_m(3), ...
                                (obj.kf*obj.al*(sin(obj.alpha(4)))^2*cos(obj.beta(4)) + obj.kf*(obj.bl+obj.al*cos(obj.alpha(4)))*cos(obj.alpha(4))*cos(obj.beta(4)) - obj.km*sin(obj.beta(4)))*obj.w_m(4)] ];
    
           

           B = [obj.JR * obj.J_beta , obj.JR * obj.J_betadot];
           

           obj.u_control = pinv(B)*(obj.v_lqr - obj.JRdot*obj.J_beta * obj.w_m);
           v = B*obj.u_control + obj.JRdot*obj.J_beta * obj.w_m;
           disp('u_control')
           disp(obj.u_control);

           
           disp('JR');
           disp(obj.JR);
           disp('JR_dot');
           disp(obj.JRdot);
           disp('J_beta');
           disp(obj.J_beta);
           disp('J_betadot');
           disp(obj.J_betadot);
           disp('B');
           disp(B);
           disp('A');
           disp(obj.JRdot*obj.J_beta * obj.w_m);
           

           % disp(obj.u_control);
           obj.w_m = obj.w_m + obj.u_control(1:4).*obj.dt;
           % disp(obj.w_m);
           obj.w_b = obj.u_control(5:8);                  

            
            
        end
        
        %%failsafe control
        function obj = FailsafeCtrl(obj, refSig)
            
            obj.x_des = refSig('x_des');
            obj.y_des = refSig('y_des');
            obj.z_des = refSig('z_des');

            mapping = [1 2 3 10 11 12 4 5 6 13 14 15 7 8 9 16 17 18];

            z_current = obj.x(mapping); % x를 z state로 변환

            z_ref = zeros(18,1);  % z ref 초기화

            z_ref(1) = obj.x_des; 
            z_ref(2) = obj.y_des;
            z_ref(3) = obj.z_des;

            obj.v_lqr   =  obj.K_lqr * (z_ref-z_current);
            
            
            obj.EvalCM();
            obj.EvalF_a_b();
            obj.EvalfailsafeTau();

            lcm = obj.cmTot(1);
             
            
           

            % 회전 매트릭스 transpose 필요 JR dot 행렬의 I matrix 미분항 0으로 설정
            obj.JR = [(1/obj.m_t)*RPY2Rot(obj.euler)', zeros(3,3);...
                      zeros(3,3), inv(obj.I_tot)];
            obj.JRdot = [(1/obj.m_t)*RPY2Rot_derivative(obj.euler,obj.w), zeros(3,3);...
                         zeros(3,3),      zeros(3,3)    ];
            % disp(obj.dI_totinv);


          
            obj.J_beta = [obj.F_a_b;...
                          obj.Tau_a_b];
            
            
            obj.J_betadot  = [  -obj.kf*cos(obj.alpha(2))*cos(obj.beta(2))*obj.w_m(2), 0,  obj.kf*cos(obj.alpha(4))*cos(obj.beta(4))*obj.w_m(4); 
                                -obj.kf*sin(obj.alpha(2))*cos(obj.beta(2))*obj.w_m(2), -obj.kf*cos(obj.beta(3))*obj.w_m(3),  obj.kf*sin(obj.alpha(4))*cos(obj.beta(4))*obj.w_m(4);
                                 obj.kf*sin(obj.beta(2))*obj.w_m(2),           obj.kf*sin(obj.beta(3))*obj.w_m(3),  obj.kf*sin(obj.beta(4))*obj.w_m(4);

                                ...
                                obj.kf*(obj.bl+obj.al*cos(obj.alpha(2)))*sin(obj.beta(2))*obj.w_m(2) + obj.km*cos(obj.alpha(2))*cos(obj.beta(2))*obj.w_m(2), ...     % dTau function 1st row
                                0, ...
                                -obj.kf*(obj.bl+obj.al*cos(obj.alpha(4)))*sin(obj.beta(4))*obj.w_m(4) - obj.km*cos(obj.alpha(4))*cos(obj.beta(4))*obj.w_m(4); ...
    
                                ...
                                (obj.kf*(obj.al*sin(obj.alpha(2))+lcm)*sin(obj.beta(2)) + obj.km*sin(obj.alpha(2))*cos(obj.beta(2)))*obj.w_m(2), ...            % dTau function 2nd row
                                (obj.kf*(obj.bl+obj.al+lcm)*sin(obj.beta(3)) - obj.km*cos(obj.beta(3)))*obj.w_m(3), ...
                                -(obj.kf*(obj.al*sin(obj.alpha(4))-lcm)*sin(obj.beta(4)) + obj.km*sin(obj.alpha(4))*cos(obj.beta(4)))*obj.w_m(4); ...
    
                                ...
                                (obj.kf*(obj.al*sin(obj.alpha(2))+lcm)*sin(obj.alpha(2))*cos(obj.beta(2)) + obj.kf*(obj.bl+obj.al*cos(obj.alpha(2)))*cos(obj.alpha(2))*cos(obj.beta(2)) - obj.km*sin(obj.beta(2)))*obj.w_m(2), ...      % dTau function 3rd row
                                (obj.kf*(obj.bl+obj.al+lcm)*cos(obj.beta(3)) + obj.km*sin(obj.beta(3)))*obj.w_m(3), ...
                                (obj.kf*(obj.al*sin(obj.alpha(4))-lcm)*sin(obj.alpha(4))*cos(obj.beta(4)) + obj.kf*(obj.bl+obj.al*cos(obj.alpha(4)))*cos(obj.alpha(4))*cos(obj.beta(4)) - obj.km*sin(obj.beta(4)))*obj.w_m(4) ];
    
           B = [obj.JR * obj.J_beta(:,2:4) , obj.JR * obj.J_betadot];
           
           %tikhonov's regulation term added
           disp('B');
           disp(B);
           lambda = 1;

           obj.u_control = (B'*B+lambda*eye(size(B,2)))\B'*(obj.v_lqr - obj.JRdot*obj.J_beta(:,2:4) * obj.w_m(2:4));
           
           obj.w_m = obj.w_m + [0,obj.u_control(1:3)']'.*obj.dt;
           
           obj.w_b(2:4) = obj.u_control(4:6);

        end

        function [imu1, imu2] = SimulateIMU(obj)
        % 1) 진짜 선형 가속도 (world-frame에서 계산된 ddX,ddY,ddZ)
        a_true = obj.x(7:9);           % 또는 obj.dx(7:9) 와 동일
        
        % 2) 진짜 각속도
        omega_true = obj.w;         % [p;q;r]
    
        % 3) 노이즈 스펙
        sigma_acc  = 0.05;   % m/s^2
        sigma_gyro = 0.0005; % rad/s
    
        % 4) 측정값 생성: 진짜 값 + 노이즈
        imu1.acc  = a_true      + sigma_acc  * randn(3,1);
        imu1.gyro = omega_true  + sigma_gyro * randn(3,1);
        imu2.acc  = a_true      + sigma_acc  * randn(3,1);
        imu2.gyro = omega_true  + sigma_gyro * randn(3,1);
        
        end
        
        function euler_acc = SimulateAcc_euler(obj)
        euler_a = obj.accelormeter;
        sigma_acc = 0.0003;
        phi_acc = atan2(euler_a(2),euler_a(3)) + sigma_acc * randn(1,1);
        theta_acc = atan2(-euler_a(1),sqrt(euler_a(2)^2+euler_a(3)^2))+ sigma_acc * randn(1,1);
        euler_acc = [phi_acc, theta_acc]';
        end

        function gps = SimulateGPS(obj)
            sigma_gps = 0.3;
            % 실제 위치에 가우시안 노이즈만 더한 GPS 시뮬레이션
            truePos = obj.x(1:3);
            gps = truePos + sigma_gps * randn(3,1);
        end

        function mag = SimulateMag(obj)
            sigma_mag = 0.003;
            trueYaw = obj.euler(3);
            mag = trueYaw + sigma_mag * randn(1,1);
        end


        

    end

    methods (Static, Access = private)
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
    end

    


  
end

