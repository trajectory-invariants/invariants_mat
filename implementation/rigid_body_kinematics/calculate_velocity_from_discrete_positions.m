function transl_velocity = calculate_velocity_from_discrete_positions(p,timestamps)
% In this code, translational velocity is calculated based on discrete
% positions using central differences.
%
% Input:
%   p: position vector                          (3x3xN)
%   timestamps: time                            (Nx1)
% Output:
%   transl_velocity: translational velocity     (Nx6)

%%
N = size(p,1);
transl_velocity = zeros(N,3);

%% First sample

% Position
DeltaP = p(2,:)-p(1,:);
dt = timestamps(2)-timestamps(1);
v = DeltaP/dt;

% Translational velocity
transl_velocity(1,:) = v';

%% Middle samples (central difference)
for i=2:N-1
    % Position
    DeltaP = p(i+1,:)-p(i-1,:);
    dt = timestamps(i+1)-timestamps(i-1);
    v = DeltaP/dt;

    % Translational velocity
    transl_velocity(i,:) = v';
end

%% Last sample
% Position
DeltaP = p(N,:)-p(N-1,:);
dt = timestamps(N)-timestamps(N-1);
v = DeltaP/dt;

% Translational velocity
transl_velocity(N,:) = v';