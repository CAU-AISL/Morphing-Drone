function path = generatePath(N)
%GENERATEPATH  Generate trajectory with linear position and linear pitch
%   path = generatePath(N) returns an N×6 matrix of [x, y, z, roll, pitch, yaw]
%   where:
%     - Position moves from [0;0;0] to [2;0;-2] linearly
%     - Pitch moves from 0° to 90° linearly
%     - Roll and yaw remain at 0

  % Define endpoints internally
  P0     = [0; 0;  0];    % start position
  P1     = [2; 0; -2];    % end position
  theta0 =   0;           % start pitch [deg]
  theta1 =  90;           % end pitch [deg]

  % Parameter s ∈ [0,1]
  s = linspace(0,1,N)';

  % Linear position interpolation (N×3)
  % pos(k,:) = P0' + s(k)*(P1-P0)'
  pos = repmat(P0', N, 1) + s * (P1 - P0)';

  % Linear pitch interpolation (N×1)
  theta = theta0 + (theta1 - theta0) .* s;

  % Roll & yaw fixed at zero
  phi = zeros(N,1);
  psi = zeros(N,1);

  % Combine into full path: [x, y, z, roll, pitch, yaw]
  path = [pos, phi, theta, psi];
end