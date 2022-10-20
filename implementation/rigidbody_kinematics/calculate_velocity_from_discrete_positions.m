function transl_velocity = calculate_velocity_from_discrete_positions(p,timestamps)


N = size(p,1);
transl_velocity = zeros(N,3);

%% First sample

% Position
DeltaP = p(2,:)-p(1,:);
dt = timestamps(2)-timestamps(1);
v = DeltaP/dt;

% Pose twist
transl_velocity(1,:) = v';

%% Middle samples (central difference)
for i=2:N-1
    % Position
    DeltaP = p(i+1,:)-p(i-1,:);
    dt = timestamps(i+1)-timestamps(i-1);
    v = DeltaP/dt;

    % Pose twist
    transl_velocity(i,:) = v';
end

%% Last sample
% Position
DeltaP = p(N,:)-p(N-1,:);
dt = timestamps(N)-timestamps(N-1);
v = DeltaP/dt;

% Pose twist
transl_velocity(N,:) = v';