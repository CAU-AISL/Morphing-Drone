close all;
clear all;
clc; clear;

addpath('./lib');

%% DEFINE
R2D=180/pi;
D2R=pi/180;

%% INIT. PARAMS.
drone_state_data = [];
drone_flare_data = []; 
drone_angle_data = [];
drone_actuator_data = [];
drone_Trq_data = [];
time = [];

drone1_Params = containers.Map({'bodyMass','armMass','armcmLength','armLength','bodyLength','Ixxb','Iyyb','Izzb','Ixxa','Iyya','Izza','Ixza','ThrustCoeff','DragCoeff'}, ...
    {1.00, 0.2, 0.1, 0.2, 0.05, 0.0132, 0.0132, 0.0268, 0.006, 0.012, 0.012, 0.01, 4, 0.4 });

drone1_initStates = [0, 0, 0, ...      % X, Y, Z
                     0, 0, 0, ...      % dX, dY, dZ
                     0, 0, 0, ...      % ddX, ddY, ddZ
                     0, 0, 0, ...      % phi, theta, psi
                     0, 0, 0, ...      % p, q, r
                     0, 0, 0]';        % dp, dq, dr

drone1_initInputs = [1.5, 1.5, 1.5, 1.5, 0, 0, 0, 0]'; % w1, w2, w3, w4, wb3

drone1_initFlare      = D2R*[0, 0, 0, 0]';       % rad
drone1_targetFlare    = D2R*[0, 0, 0, 0]';     % rad
drone1_initTilt       = D2R*[0, 0, 0, 0]';       % rad

% 드론 몸체 좌표(4개 암 + 중앙 + payload)
drone1_body = [  0.2,    0,      0,      1; ...
                 0,      0.2,    0,      1; ...
                -0.2,    0,      0,      1; ...
                 0,     -0.2,    0,      1; ...
                 0,      0,      0,      1; ... % payload at (0,0,0)
                 0,      0,     -0.15,   1]';

% 위에서 바라본(2D) frame
drone1_frame = [0.05,   0,   0; ...
                0,   0.05,  0; ...
               -0.05,  0,   0; ...
                0,  -0.05,  0 ]';

simulationTime = 20;

drone1 = morphing_mk2(drone1_Params, ...
                      drone1_initStates, ...
                      drone1_initInputs, ...
                      drone1_initFlare, ...
                      drone1_targetFlare, ...
                      drone1_initTilt, ...
                      simulationTime);

%% 원하는 위치 명령
commandSig = containers.Map({'x_des','y_des','z_des','phi_des','theta_des','psi_des'}, {2, 2, -2, 0, 0, 0});

dt = 0.01;
numSteps = simulationTime/dt;

for i = 1:numSteps
    if i > 3000
        drone1.SetLQR_Gain();
        drone1.StopMotor();
        drone1.FailsafeCtrl(commandSig);
        %drone1.AttitudeCtrl(commandSig);
    else
        drone1.SetLQR_Gain();
        drone1.AttitudeCtrl(commandSig);
    end
    
    % 상태 업데이트
    drone1.UpdateState();
    drone1_state = drone1.GetState();
    
    % 시뮬레이션 데이터 저장
    drone_state_data = [drone_state_data, drone1_state.x];
    drone_angle_data = [drone_angle_data, drone1_state.tilt];
    drone_flare_data = [drone_flare_data, drone1_state.flare];
    drone_actuator_data = [drone_actuator_data, drone1_state.w_m];
    drone_Trq_data = [drone_Trq_data, drone1_state.Trq];
    phi_deg   = R2D * drone_state_data(10,:);              % roll (phi)
    theta_deg = R2D * drone_state_data(11,:);              % pitch (theta)
    psi_deg   = R2D * unwrap(drone_state_data(12,:));      % yaw (psi), unwrap in rad then convert
    time = [time, i*dt];
    
    % 만약 Z축이 0 이상으로 올라온다면 충돌 등으로 중단하려면 사용
     % if (drone1_state.x(3) >= 0)
     %     msgbox('Crashed!!','Error','error');
     %     break;
     % end
end

%% ----------------------------
%    여기까지가 "데이터만 수집"
%    이제부터 "모아서 한 번에 플롯"
%% ----------------------------

%% 시간 인덱스 계산
[~, idx_10s] = min(abs(time - 10));

%% 시간에 따른 X, Y, Z, 각도(phi, theta, psi)
figure('Position',[50 50 1600 300]);

% X
subplot(2,3,1)
plot(time, drone_state_data(1,:), 'b','LineWidth',1); hold on;
xline(10, 'k--', 'LineWidth', 1.2);
plot(time(idx_10s), drone_state_data(1,idx_10s), 'ro', 'MarkerFaceColor', 'r');
% x_des = 2에 빨간 점선
yline(commandSig('x_des'), 'r--', 'LineWidth', 1.2);
title('x [m]'); grid on;

% Y
subplot(2,3,2)
plot(time, drone_state_data(2,:), 'b','LineWidth',1); hold on;
xline(10, 'k--', 'LineWidth', 1.2);
plot(time(idx_10s), drone_state_data(2,idx_10s), 'ro', 'MarkerFaceColor', 'r');
% y_des = 2에 빨간 점선
yline(commandSig('y_des'), 'r--', 'LineWidth', 1.2);
title('y [m]'); grid on;

% Z
subplot(2,3,3)
plot(time, drone_state_data(3,:), 'b','LineWidth',1); hold on;
xline(10, 'k--', 'LineWidth', 1.2);
plot(time(idx_10s), drone_state_data(3,idx_10s), 'ro', 'MarkerFaceColor', 'r');
% z_des = -2에 빨간 점선
yline(commandSig('z_des'), 'r--', 'LineWidth', 1.2);
title('z [m]'); grid on;

% phi
subplot(2,3,4)
plot(time, R2D * drone_state_data(10,:), 'b','LineWidth',1); hold on;
xline(10, 'k--', 'LineWidth', 1.2);
plot(time(idx_10s), R2D * drone_state_data(10,idx_10s), 'ro', 'MarkerFaceColor', 'r');
title('phi [degree]'); grid on;
 
% theta
subplot(2,3,5)
plot(time, R2D * drone_state_data(11,:), 'b','LineWidth',1); hold on;
xline(10, 'k--', 'LineWidth', 1.2);
plot(time(idx_10s), R2D * drone_state_data(11,idx_10s), 'ro', 'MarkerFaceColor', 'r');
title('theta [degree]'); grid on;
 
% psi
subplot(2,3,6)
plot(time, R2D * drone_state_data(12,:), 'b','LineWidth',1); hold on;
xline(10, 'k--', 'LineWidth', 1.2);
plot(time(idx_10s), R2D * drone_state_data(12,idx_10s), 'ro', 'MarkerFaceColor', 'r');
title('psi [degree]'); grid on;

%% 메인 모터 출력
figure('Position',[100 50 200 300]);

subplot(4,1,1)
plot(time, drone_actuator_data(1,:), 'b','LineWidth',1); hold on;
xline(10, 'k--', 'LineWidth', 1.2);
plot(time(idx_10s), drone_actuator_data(1,idx_10s), 'ro', 'MarkerFaceColor', 'r');
title('main motor 1 ang/rate^2'); grid on;

subplot(4,1,2)
plot(time, drone_actuator_data(2,:), 'b','LineWidth',1); hold on;
xline(10, 'k--', 'LineWidth', 1.2);
plot(time(idx_10s), drone_actuator_data(2,idx_10s), 'ro', 'MarkerFaceColor', 'r');
title('main motor 2 ang/rate^2'); grid on;

subplot(4,1,3)
plot(time, drone_actuator_data(3,:), 'b','LineWidth',1); hold on;
xline(10, 'k--', 'LineWidth', 1.2);
plot(time(idx_10s), drone_actuator_data(3,idx_10s), 'ro', 'MarkerFaceColor', 'r');
title('main motor 3 ang/rate^2'); grid on;

subplot(4,1,4)
plot(time, drone_actuator_data(4,:), 'b','LineWidth',1); hold on;
xline(10, 'k--', 'LineWidth', 1.2);
plot(time(idx_10s), drone_actuator_data(4,idx_10s), 'ro', 'MarkerFaceColor', 'r');
title('main motor 4 ang/rate^2'); grid on;

%% 틸트 각도
figure('Position',[100 50 1100 500]);

% 주의: 아래 코드는 인덱싱/타이틀을 조금 수정하셔야
% 실제 tilt angle(1~4)에 대응됩니다. 
% 현재 예시로 그대로 둡니다.
subplot(2,2,1)
plot(time, R2D * drone_angle_data(1,:), 'b','LineWidth',2); hold on;
xline(10, 'k--', 'LineWidth', 1.2);
plot(time(idx_10s), R2D * drone_angle_data(2,idx_10s), 'ro', 'MarkerFaceColor', 'r');
title('tilt angle1 [degree]'); grid on;

subplot(2,2,2)
plot(time, R2D * drone_angle_data(2,:), 'b','LineWidth',2); hold on;
xline(10, 'k--', 'LineWidth', 1.2);
plot(time(idx_10s), R2D * drone_angle_data(3,idx_10s), 'ro', 'MarkerFaceColor', 'r');
title('tilt angle2 [degree]'); grid on;

subplot(2,2,3)
plot(time, R2D * drone_angle_data(3,:), 'b','LineWidth',2); hold on;
xline(10, 'k--', 'LineWidth', 1.2);
plot(time(idx_10s), R2D * drone_angle_data(4,idx_10s), 'ro', 'MarkerFaceColor', 'r');
title('tilt angle3 [degree]'); grid on;

subplot(2,2,4)
plot(time, R2D * drone_angle_data(4,:), 'b','LineWidth',2); hold on;
xline(10, 'k--', 'LineWidth', 1.2);
plot(time(idx_10s), R2D * drone_angle_data(4,idx_10s), 'ro', 'MarkerFaceColor', 'r');
title('tilt angle4 [degree]'); grid on;

figure('Position',[80 80 1100 300]);
tl = tiledlayout(1,4,'TileSpacing','compact','Padding','compact');
names = {'\alpha_1','\alpha_2','\alpha_3','\alpha_4'};
for k = 1:4
    nexttile;
    plot(time, R2D*drone_flare_data(k,:), 'LineWidth',1.4); hold on;
    xline(10,'k--','LineWidth',1.2);
    title([names{k} ' [deg]']); grid on;
end



%% ===== Save to MAT (time-aligned) =====
% 행 = 시간스텝, 열 = 변수
t     = time(:);                        % Nx1
X     = drone_state_data.';             % Nx18  (state.x 전체)
Alpha = drone_flare_data.';             % Nx4   (alpha 4개)
Beta = drone_angle_data.';
Wm    = drone_actuator_data(1:4,:).';   % Nx4   (main motor angular rate 4개)

% 안전 체크 (길이 일치)
assert( size(X,1)==numel(t) && size(Alpha,1)==numel(t) && size(Wm,1)==numel(t), ...
    'Length mismatch: time vs data matrices');

% 폴더 및 파일명
if ~exist('logs','dir'), mkdir('logs'); end
outFile = fullfile('logs', sprintf('morphing_mk2_log_%s.mat', datestr(now,'yyyymmdd_HHMMSS')));

% 저장 (필요하면 -v7.3 제거 가능)
save(outFile, 't','X','Alpha','Beta','Wm', '-v7.3');

fprintf('Saved MAT: %s\n', outFile);