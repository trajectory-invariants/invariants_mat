function [invariants_output, ISA_pose_output] = initialize_invariants_screw(screw_trajectory,bool)
%%
% In this code, initial values for the screw-invariants and moving frame are calculated
% based on an Average Screw Axis (ASA) frame.
%
% Input:
%   screw_trajectory (Nx6)  : Columns 1-3 contain the directional part of the screw.
%                             Columns 4-6 contain the translational part of the screw.
%   bool                    : {0,1} boolean. If set to 1, it is ensured that the initial object invariants are always positive
% Output:
%   invariants       (Nx6)  : initial values for the screw invariants
%   ISA_pose         (4x4xN): initial values for the moving frame
%

T_ASA = calculate_ASA_pose(screw_trajectory);

% Select the signed directions of the axes of the initial moving frames
N = size(screw_trajectory,1);
e_x = T_ASA(1:3,1);
if norm(T_ASA(1:3,2)) < 0.5 % special case for reference values for torque
    vector = [0;1;0];
    vector = vector-(vector'*T_ASA(1:3,1))*T_ASA(1:3,1);
    vector = vector/norm(vector);
    e_y = vector;
else
    e_y = T_ASA(1:3,2);
end

% Select a segment of the trajectory containing sufficient trajectory information
% and where the mean is well-defined
%(four this contour, 3/4 of the trajectory is selected)
mean_vector = sum(screw_trajectory(1:end-round(N/4),1:3),1)*3/N;

if dot(e_x,mean_vector) < 0
    e_x = -e_x;
end
if dot(e_y,mean_vector) < 0
    e_y = -e_y;
end
e_z = cross(e_x,e_y);

% Update the orientation of the ASA with the signed directions
T_ASA(1:3,1:3) = [e_x,e_y,e_z];
inv_T_ASA = inverse_pose(T_ASA);

% Calculate initial values for the invariants
trajectory_in_ASA = zeros(N,6);
for k =1:N
    trajectory_in_ASA(k,:) = transpose(transform_screw(inv_T_ASA,screw_trajectory(k,:)'));
end

invariants_output = zeros(N,6);
ISA_pose_output = zeros(4,4,N);
if ~bool
    for k = 1:N
        invariants_output(k,:) = [trajectory_in_ASA(k,1),0.01,0.01,...
            trajectory_in_ASA(k,4),0.01,0.01];
        ISA_pose_output(:,:,k) = T_ASA;
    end
else % case object invariant positive
    for k = 1:N
        if trajectory_in_ASA(k,1) < 0
            invariants_output(k,:) = [-trajectory_in_ASA(k,1),0.01,0.01,...
                -trajectory_in_ASA(k,4),0.01,0.01];
            ISA_pose_output(:,:,k) = [-T_ASA(1:3,1),T_ASA(1:3,2),...
                cross(-T_ASA(1:3,1),T_ASA(1:3,2)),T_ASA(1:3,4);...
                0 0 0 1];
        else
            invariants_output(k,:) = [trajectory_in_ASA(k,1),0.01,0.01,...
                trajectory_in_ASA(k,4),0.01,0.01];
            ISA_pose_output(:,:,k) = T_ASA;
        end
    end
end
end