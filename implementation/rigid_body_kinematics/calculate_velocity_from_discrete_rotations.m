function rot_velocity = calculate_velocity_from_discrete_rotations(R,timestamps)
% In this code, rotational velocity is calculated based on discrete
% rotations using central differences.
%
% Input:
%   R: rotation matrix                      (3x3xN)
%   timestamps: time                        (Nx1)
% Output:
%   rot_velocity: rotational velocity       (Nx6)

%%
N = size(R,3);
rot_velocity = zeros(N,3);

%% First sample
% Rotation
DeltaR = R(:,:,2)*R(:,:,1)';
dt = timestamps(2)-timestamps(1);
dtwist = logm(DeltaR)/dt;
omega = [-dtwist(2,3) dtwist(1,3) -dtwist(1,2)];

% Pose twist
rot_velocity(1,:) = omega;

%% Middle samples (central difference)
for i=2:N-1
    % Rotation
    DeltaR = R(:,:,i+1)*R(:,:,i-1)';
    dt = timestamps(i+1)-timestamps(i-1);
    dtwist = logm(DeltaR)/(dt);
    omega = [-dtwist(2,3) dtwist(1,3) -dtwist(1,2)];

    % rot velocity
    rot_velocity(i,:) = omega;
end

%% Last sample
% Rotation
DeltaR = R(:,:,N)*R(:,:,N-1)';
dt = timestamps(N)-timestamps(N-1);
dtwist = logm(DeltaR)/dt;
omega = [-dtwist(2,3) dtwist(1,3) -dtwist(1,2)];

% rot velocity
rot_velocity(N,:) = omega;