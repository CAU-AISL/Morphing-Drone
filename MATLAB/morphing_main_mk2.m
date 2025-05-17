close all;
clear all;
clc; clear;

addpath('./lib');
 
% v = VideoWriter('simulation_video.mp4', 'MPEG-4'); % Create video writer object
% open(v); % Open the video writer

%% DEFINE
R2D=180/pi;
D2R=pi/180;

%% INIT. PARAMS. 
% body mass : 1kg
% drone arm mas : 0.2m
% arm cmlength : 0.15m
% arm length : 0.2m
% body length : 0.05m
% Ixxb : body x axis moment 
% Iyyb : body y axis moment 
% Izzb : body z axis moment 
% Ixxa : arm x axis moment 
% Iyya : arm y axis moment 
% Izza : arm z axis moment 
% Ixza : arm xa axis moment 
% kf : thrust coeff
% km : drag coeff(torque)

drone_state_data = [];
time = [];

drone1_Params = containers.Map({'bodyMass','armMass','armcmLength','armLength','bodyLength','Ixxb','Iyyb','Izzb','Ixxa','Iyya','Izza','Ixza','ThrustCoeff','DragCoeff'}, ...
    {1.00, 0.2, 0.1, 0.2, 0.05, 0.0132, 0.0132, 0.0268, 0.006, 0.012, 0.012, 0.01, 1, 1 });


drone1_initStates = [2, 0, -5, ...      % X,Y,Z
    0, 0, 0, ...                        % dX, dY, dZ
    0, 0, 0, ...                        % ddX, ddY, ddZ
    0, 0, 0, ...                        % phi, theta, psi
    0, 0, 0, ...                        % p, q, r
    0, 0, 0]';                           % dp, dq, dr

                          
drone1_initInputs = [0, 0, 0, 0, 0]'; % w1, w2, w3, w4, wb3


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


simulationTime = 2;
% body가 변화하는 경우 drone_body에 대한 parameter들을 함수에 변수로 추가해 줄 필요 있음
drone1 = morphing_mk2(drone1_Params, drone1_initStates, drone1_initInputs, drone1_initFlare, drone1_targetFlare, drone1_initTilt, simulationTime);


%% Init. 3D Fig.
fig1 = figure('pos', [0 60 800 700]);
h = gca;
view(3);
fig1.CurrentAxes.ZDir = 'Reverse';
fig1.CurrentAxes.YDir = 'Reverse';

axis equal;
grid on;

xlim([-5 5]); ylim([-5 5]); zlim([-8 0]);
xlabel('X[m]'); ylabel('Y[m]'); zlabel('Z[m]');

hold(gca, 'on');
drone1_state = drone1.GetState();

wHb = [RPY2Rot(drone1_state.x(10:12))', drone1_state.x(1:3);
       0, 0, 0, 1];
drone1_world = wHb * drone1_body;
drone1_2D = drone1_body(1:3, :);
drone1_atti = drone1_world(1:3, :);

fig1_ARM1 = plot3(gca, drone1_atti(1,[1 5]), drone1_atti(2,[1 5]), drone1_atti(3,[1 5]), '-ro', 'MarkerSize', 5);
fig1_ARM2 = plot3(gca, drone1_atti(1,[2 5]), drone1_atti(2,[2 5]), drone1_atti(3,[2 5]), '-bo', 'MarkerSize', 5);
fig1_ARM3 = plot3(gca, drone1_atti(1,[3 5]), drone1_atti(2,[3 5]), drone1_atti(3,[3 5]), '-ro', 'MarkerSize', 5);
fig1_ARM4 = plot3(gca, drone1_atti(1,[4 5]), drone1_atti(2,[4 5]), drone1_atti(3,[4 5]), '-bo', 'MarkerSize', 5);

fig1_payload = plot3(gca, drone1_atti(1,[5 6]), drone1_atti(2,[5 6]), drone1_atti(3,[5 6]), '-k', 'LineWidth', 3);
fig1_shadow = plot3(gca, 0 ,0, 0, 'xk','LineWidth',3);

hold(gca, 'off');

%% Init. 2D plot Fig.
fig1_2 = figure('pos', [820 80 200 200]);
hold(gca, 'on');
fig1_2_ARM1 = plot(gca, [drone1_2D(1,1), drone1_frame(1,1)], [drone1_2D(2,1), drone1_frame(2,1)], '-ro', 'MarkerSize', 5);
fig1_2_ARM2 = plot(gca, [drone1_2D(1,2), drone1_frame(1,2)], [drone1_2D(2,2), drone1_frame(2,2)], '-bo', 'MarkerSize', 5);
fig1_2_ARM3 = plot(gca, [drone1_2D(1,3), drone1_frame(1,3)], [drone1_2D(2,3), drone1_frame(2,3)], '-ro', 'MarkerSize', 5);
fig1_2_ARM4 = plot(gca, [drone1_2D(1,4), drone1_frame(1,4)], [drone1_2D(2,4), drone1_frame(2,4)], '-bo', 'MarkerSize', 5);
hold(gca, 'off');

%% Init. Data Fig.
fig2 = figure('pos',[820 300 700 450]);
subplot(2,3,1)
title('x[m]');
grid on;
hold on;
subplot(2,3,2)
title('y[m]');
grid on;
hold on;
subplot(2,3,3)
title('z[m]');
grid on;
hold on;
subplot(2,3,4)
title('xdot[m/s]');
grid on;
hold on;
subplot(2,3,5)
title('ydot[m/s]');
grid on;
hold on;
subplot(2,3,6)
title('zdot[m/s]');
grid on;
hold on;


%%
commandSig = containers.Map({'x_des','y_des','z_des',...
                         'phi_des', 'theta_des' 'psi_des'...
                         'xdot_des','ydot_des','zdot_des',...
                         'phidot_des', 'thetadot_des', 'psidot_des'...
                         'xddot_des', 'yddot_des', 'zddot_des'...
                         'phiddot_des', 'thetaddot_des', 'psiddot_des'}, ...
                        {0, 0, -5,...
                         0, 0, 0,...
                         0, 0, 0,...
                         0, 0, 0,...
                         0, 0, 0,...
                         0, 0, 0});

for i = 1:simulationTime/0.01
    drone1.AttitudeCtrl(commandSig);
    drone1.UpdateState();
    drone1_state = drone1.GetState();
    drone_state_data = [drone_state_data, drone1_state.x];
    time = [time, i/100];
    %% 3D plot 
    figure(1)
    wHb = [RPY2Rot(drone1_state.x(10:12))' drone1_state.x(1:3); 0 0 0 1];
    drone1_world = wHb * drone1_state.body;
    drone1_atti = drone1_world(1:3, :);
    drone1_2D = drone1_state.body(1:3, :);

    set(fig1_ARM1, ...
        'xData', drone1_atti(1,[1,5]), ...
        'yData', drone1_atti(2,[1,5]), ...
        'zData', drone1_atti(3,[1,5])); 
    set(fig1_ARM2, ...
        'xData', drone1_atti(1,[2,5]), ...
        'yData', drone1_atti(2,[2,5]), ...
        'zData', drone1_atti(3,[2,5])); 
    set(fig1_ARM3, ...
        'xData', drone1_atti(1,[3,5]), ...
        'yData', drone1_atti(2,[3,5]), ...
        'zData', drone1_atti(3,[3,5]));
    set(fig1_ARM4, ...
        'xData', drone1_atti(1,[4,5]), ...
        'yData', drone1_atti(2,[4,5]), ...
        'zData', drone1_atti(3,[4,5]));
    set(fig1_payload, ...
        'xData', drone1_atti(1,[5,6]), ...
        'yData', drone1_atti(2,[5,6]), ...
        'zData', drone1_atti(3,[5,6])); 
    set(fig1_shadow, ...
        'xData', drone1_state.x(1), ...
        'yData', drone1_state.x(2), ...
        'zData', 0); 

    figure(2)
    set(fig1_2_ARM1, ...
        'xData', [drone1_2D(1,1), drone1_frame(1,1)], ...
        'yData', [drone1_2D(2,1), drone1_frame(2,1)]);
      
    set(fig1_2_ARM2, ...
        'xData', [drone1_2D(1,2), drone1_frame(1,2)], ...
        'yData', [drone1_2D(2,2), drone1_frame(2,2)]);
        
    set(fig1_2_ARM3, ...
        'xData', [drone1_2D(1,3), drone1_frame(1,3)], ...
        'yData', [drone1_2D(2,3), drone1_frame(2,3)]);

    set(fig1_2_ARM4, ...
        'xData', [drone1_2D(1,4), drone1_frame(1,4)], ...
        'yData', [drone1_2D(2,4), drone1_frame(2,4)]);


    %     frame = getframe(gcf); % Capture the current figure as a frame
    % writeVideo(v, frame); % Write the frame to video
    

    line_width = 1;
    figure(3)
    subplot(2,3,1)
            plot(time, drone_state_data(1,:), 'Color', 'b', 'LineWidth', line_width);
    subplot(2,3,2)
            plot(time, drone_state_data(2,:), 'Color', 'b', 'LineWidth', line_width);
    subplot(2,3,3)
            plot(time, drone_state_data(3,:), 'Color', 'b', 'LineWidth', line_width);
    subplot(2,3,4)
            plot(time, drone_state_data(4,:), 'Color', 'b', 'LineWidth', line_width);
    subplot(2,3,5)
            plot(time, drone_state_data(5,:), 'Color', 'b', 'LineWidth', line_width);
    subplot(2,3,6)
            plot(time, drone_state_data(6,:), 'Color', 'b', 'LineWidth', line_width);

    


    drawnow;

    if (drone1_state.x(3) >=0)
    msgbox('Crashed!!','Error','error');
    break;
    end
end
% close(v);