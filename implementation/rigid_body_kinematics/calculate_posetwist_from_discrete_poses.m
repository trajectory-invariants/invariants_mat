function twist = calculate_posetwist_from_discrete_poses(R,p,dt)
% In this code, pose twists are calculated based on discrete poses using
% central differences.
%
% Input:
%   R: rotation matrix          (3x3xN)
%   p: position                 (3xN)
%   dt: sample time             (1x1)
% Output:
%   pose twist                 (Nx6)

N = size(R,3);
twist = zeros(N,6);

%% First sample
% Rotation
DeltaR = R(:,:,2)*R(:,:,1)';
dtwist = logm(DeltaR)/(dt);
omega = [-dtwist(2,3) dtwist(1,3) -dtwist(1,2)];

% Position
DeltaP = p(:,2)-p(:,1);
v = DeltaP/(dt);

% Pose twist
twist(1,:) = [omega v'];

%% Middle samples (central difference)
for i=2:N-1
    % Rotation
    DeltaR = R(:,:,i+1)*R(:,:,i-1)';
    dtwist = logm(DeltaR)/(2*dt);
    omega = [-dtwist(2,3) dtwist(1,3) -dtwist(1,2)];

    % Position
    DeltaP = p(:,i+1)-p(:,i-1);
    v = DeltaP/(2*dt);

    % Pose twist
    twist(i,:) = [omega v'];
end

%% Last sample
% Rotation
DeltaR = R(:,:,N)*R(:,:,N-1)';
dtwist = logm(DeltaR)/(dt);
omega = [-dtwist(2,3) dtwist(1,3) -dtwist(1,2)];

% Position
DeltaP = p(:,N)-p(:,N-1);
v = DeltaP/(dt);

% Pose twist
twist(N,:) = [omega v'];