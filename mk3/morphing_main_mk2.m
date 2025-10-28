close all; clear; clc;
addpath('./lib');

%% 시뮬레이션 설정
R2D = 180/pi; D2R = pi/180;
dt  = 0.01;                    % 타임스텝
T   = 10;                      % 총 시뮬레이션 시간 [s]
N   = T/dt;                    % 스텝 수
time = (0:dt:T-dt)';           % 시간 벡터

%% 드론 객체 생성 (생략된 파라미터 설정 부분은 동일)
drone1_Params = containers.Map({'bodyMass','armMass','armcmLength','armLength','bodyLength','Ixxb','Iyyb','Izzb','Ixxa','Iyya','Izza','Ixza','ThrustCoeff','DragCoeff'}, ...
    {1.00, 0.2, 0.1, 0.2, 0.05, 0.0132, 0.0132, 0.0268, 0.006, 0.012, 0.012, 0.01, 4, 0.4 });


drone1_initStates = [0, 0, 0, ...      % X,Y,Z
    0, 0, 0, ...                        % dX, dY, dZ
    0, 0, 0, ...                        % ddX, ddY, ddZ
    0, 0, 0, ...                        % phi, theta, psi
    0, 0, 0, ...                        % p, q, r
    0, 0, 0]';                           % dp, dq, dr

                          
drone1_initInputs = [1.5, 1.5, 1.5, 1.5, 0, 0, 0, 0]'; % w1, w2, w3, w4, wb3


drone1_initFlare = D2R*[0, 0, 0, 0]';      % radian parameter
drone1_targetFlare = D2R*[0, -40, 0, 40]'; % radian parameter
drone1_initTilt = D2R*[0, 0, 0, 0]';       % radian parameter

drone1_body = [0.2,         0,         0,        1; ...
                 0,       0.2,         0,        1; ...
              -0.2,         0,         0,        1; ...
                 0,      -0.2,         0,        1;...
                 0,         0,         0,        1;...%payload at 0,0,0
                 0,         0,     -0.15,        1]';
drone1_frame = [0.05,    0,   0;...
                  0,  0.05,   0;...
               -0.05,    0,   0;...
                  0, -0.05,   0]';                



% Path generation
myPath = generatePath(N);


% body가 변화하는 경우 drone_body에 대한 parameter들을 함수에 변수로 추가해 줄 필요 있음
drone1 = morphing_mk2(drone1_Params, drone1_initStates, drone1_initInputs, drone1_initFlare, drone1_targetFlare, drone1_initTilt, myPath, T);

%% 데이터 저장용 배열 미리 할당
dataTrue   = zeros(N,18);   % [X Y Z, dX dY dZ, φ θ ψ, p q r]
dataPred   = zeros(N,18);   % 칼만 예측
dataEst    = zeros(N,18);   % 칼만 추정
dataAccMeas= zeros(N,3);    % IMU1 가속도 3축
dataGyroMeas= zeros(N,3);   % IMU1 자이로 3축

%% 시뮬레이션 루프
for k = 1:N
    drone1.SetLQR_Gain();
    drone1.AttitudeCtrl();
    drone1.UpdateState();

    % 1) 진짜 상태
    xt = drone1.GetState().x;
    dataTrue(k,:) = [ xt(1:3)', xt(4:6)', xt(7:9)', xt(10:12)', xt(13:15)', xt(16:18)' ];

    % 2) 칼만 사전 예측 (UpdateState 안에서 obj.x_pred에 저장됨)
    % xp = drone1.x_pred;
    % dataPred(k,:) = xp';

    % 3) 센서 측정값
    z_k = drone1.z_k;
    dataZkgps(k,:)  = z_k(1:3);
    dataZkgyro(k,:) = z_k(7:9);
    dataEuler(k,:) = z_k(4:6);

    % 4) 칼만 사후 추정
    xe = drone1.zx_est;
    dataEst(k,:) = xe';
end

%% 1) 위치 플롯 (X,Y,Z)
figure('Name','Position'); vars={'X','Y','Z'};
for i=1:3
    subplot(1,3,i); hold on; grid on;
    plot(time, dataTrue(:,i),   '.r', 'DisplayName','Truth(meas)');   % 대체 측정값
    % plot(time, dataPred(:,i),   '-k','DisplayName','Predict');
    plot(time, dataEst(:,i),    '-b', 'DisplayName','Estimate');
    plot(time, dataZkgps(:,i),  ':m', 'DisplayName','GPSdata');
    title(vars{i}); xlabel('Time [s]'); ylabel([vars{i} ' [m]']);
    if i==1, legend('Location','best'); end
end

%% 2) 속도 플롯 (dX,dY,dZ) 
figure('Name','Velocity'); vars={'dX','dY','dZ'};
for i=1:3
    subplot(1,3,i); hold on; grid on;
    plot(time, dataTrue(:,3+i),   '.r', 'DisplayName','Truth');
    % plot(time, dataPred(:,3+i),   '-k','DisplayName','Predict');
    plot(time, dataEst(:,6+i),    '-b', 'DisplayName','Estimate');
    title(vars{i}); xlabel('Time [s]'); ylabel([vars{i} ' [m/s]']);
    if i==1, legend('Location','best'); end
end

%% 3) 가속도 플롯 (ddX,ddY,ddZ) 
figure('Name','Acc'); vars={'ddX','ddY','ddZ'};
for i=1:3
    subplot(1,3,i); hold on; grid on;
    plot(time, dataTrue(:,6+i),   '.r', 'DisplayName','Truth');
    % plot(time, dataPred(:,3+i),   '-k','DisplayName','Predict');
    plot(time, dataEst(:,12+i),    '-b', 'DisplayName','Estimate');
    title(vars{i}); xlabel('Time [s]'); ylabel([vars{i} ' [m/s]']);
    if i==1, legend('Location','best'); end
end

%% 4) Euler 각 (φ,θ,ψ)
figure('Name','Euler Angles'); vars={'Phi','Theta','Psi'};
for i=1:3
    subplot(1,3,i); hold on; grid on;
    plot(time, R2D*dataTrue(:,9+i),  '.r', 'DisplayName','Truth');
    % plot(time, R2D*dataPred(:,6+i),  '-k','DisplayName','Predict');
    plot(time, R2D*dataEst(:,3+i),   '-b', 'DisplayName','Estimate');
    
    plot(time, R2D * dataEuler(:,i), ':m', 'DisplayName', 'MAGdata');
    
    title(vars{i}); xlabel('Time [s]'); ylabel([vars{i} ' [deg]']);
    if i==1, legend('Location','best'); end
end

%% 5) 각속도 (p,q,r) + 자이로
figure('Name','Angular Rates & GyroMeas'); vars={'p','q','r'};
for i=1:3
    subplot(1,3,i); hold on; grid on;
    plot(time, R2D*dataTrue(:,12+i),     '.r', 'DisplayName','Truth');
    % plot(time, R2D*dataPred(:,9+i),     '-k','DisplayName','Predict');
    plot(time, R2D*dataEst(:,9+i),      '-b', 'DisplayName','Estimate');
    plot(time, R2D*dataZkgyro(:,i),   ':m', 'DisplayName','GyroMeas');
    title(vars{i}); xlabel('Time [s]'); ylabel([vars{i} ' [deg/s]']);
    if i==1, legend('Location','best'); end
end

%% 6) 각가속도 (dp,dq,dr) + 자이로
figure('Name','Jerk Angular'); vars={'dp','dq','dr'};
for i=1:3
    subplot(1,3,i); hold on; grid on;
    plot(time, R2D*dataTrue(:,15+i),     '.r', 'DisplayName','Truth');
    % plot(time, R2D*dataPred(:,9+i),     '-k','DisplayName','Predict');
    plot(time, R2D*dataEst(:,15+i),      '-b', 'DisplayName','Estimate');
    title(vars{i}); xlabel('Time [s]'); ylabel([vars{i} ' [deg/s]']);
    if i==1, legend('Location','best'); end
end
