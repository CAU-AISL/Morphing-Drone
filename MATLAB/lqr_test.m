clc;
clear;
close all;

% 시스템 모델 정의 (상태공간 표현)
A = [zeros(6,6), eye(6), zeros(6,6);...
            zeros(6,6), zeros(6,6), eye(6);...
            zeros(6,6), zeros(6,6), zeros(6,6)];  % 시스템 행렬
B = [zeros(12,6);...
            eye(6)];  % 입력 행렬
C = eye(18);  % 출력 행렬
D = zeros(18,6); % D 행렬 (출력 영향 없음)

% LQR 가중치 행렬 정의
Q = eye(size(A));  % 상태 가중 행렬
R = eye(size(B,2)); % 입력 가중 행렬

% LQR 게인 계산
K = lqr(A, B, Q, R);

% 참조 입력 추종을 위한 피드포워드 이득 계산
Nbar = pinv(C*pinv(A-B*K)*B); % 정적 피드포워드 이득

% 시뮬레이션 설정
dt = 0.01;  % 샘플링 타임
Tfinal = 10; % 시뮬레이션 시간
time = 0:dt:Tfinal;
x = zeros(size(A,1), length(time)); % 상태 초기화

% 18개의 상태에 대한 참조 입력 (예제: 일부는 1, 일부는 0.5, 일부는 0)
r = [5*ones(6, length(time));   % 첫 6개 상태의 참조 입력: 1
     0.5*ones(6, length(time)); % 중간 6개 상태의 참조 입력: 0.5
     zeros(6, length(time))]; % 마지막 6개 상태의 참조 입력: 0

u = zeros(size(B,2), length(time)-1); % 입력 초기화

% 상태 피드백 제어 루프
for k = 1:length(time)-1
    u(:,k) = -K*x(:,k) - Nbar*r(:,k);  % LQR 제어 입력 (피드포워드 포함)
    x(:,k+1) = x(:,k) + dt*(A*x(:,k) + B*u(:,k)); % 상태 업데이트
end

% 마지막 입력 보정하여 time과 길이 맞추기
u(:,end+1) = u(:,end);

% 출력 계산
y = C*x;

figure;
for i = 1:3
    subplot(3,1,i);
    plot(time, r(i,:), 'r--', 'LineWidth', 1.5); hold on;  % 참조 입력 (Reference)
    plot(time, y(i,:), 'b', 'LineWidth', 1.5);  % 시스템 응답 (Response)
    xlabel('Time (s)');
    ylabel(sprintf('State %d', i));
    legend('Reference', 'Response');
    title(sprintf('State %d: Reference vs Response', i));
end