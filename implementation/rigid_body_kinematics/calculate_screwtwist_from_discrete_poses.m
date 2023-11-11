function twist = calculate_screwtwist_from_discrete_poses(T,dt)
%%
% In this code, screw twists are calculated based on discrete poses using
% central differences.
%
% Input:
%   R: rotation matrix          (3x3xN)
%   p: position                 (3xN)
%   dt: sample time             (1x1)
% Output:
%   screw twist                 (Nx6)
%

%% Initialization
N = size(T,3);
twist = zeros(N,6);
R = T(1:3,1:3,:);
p = squeeze(T(1:3,4,:));

%% First sample
% Rotation
DeltaR = R(:,:,2)*R(:,:,1)';
dtwist = logm(DeltaR)/(dt);
omega = [-dtwist(2,3) dtwist(1,3) -dtwist(1,2)];

% Position
DeltaP = p(:,2)-p(:,1);
v = DeltaP/(dt);

% Change refpoint
v0 = v' - cross(omega,p(:,1));

% Screw twist
twist(1,:) = [omega v0];

%% Middle samples (central difference)
for i = 2 : N-1
    % Rotation
    DeltaR = R(:,:,i+1)*R(:,:,i-1)';
    dtwist = logm(DeltaR)/(2*dt);
    omega = [-dtwist(2,3) dtwist(1,3) -dtwist(1,2)];

    % Position
    DeltaP = p(:,i+1)-p(:,i-1);
    v = DeltaP/(2*dt);

    % Change ref. point
    v0 = v' - cross(omega,p(:,i));

    % Screw twist
    twist(i,:) = [omega v0];
end

%% Last sample
% Rotation
DeltaR = R(:,:,N)*R(:,:,N-1)';
dtwist = logm(DeltaR)/(dt);
omega = [-dtwist(2,3) dtwist(1,3) -dtwist(1,2)];

% Position
DeltaP = p(:,N)-p(:,N-1);
v = DeltaP/(dt);

% Change ref. point
v0 = v' - cross(omega,p(:,N));

% Screw twist
twist(N,:) = [omega v0];

