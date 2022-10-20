function [invariants_output, R_FS_output] = initialize_invariants_vector(vector_trajectory,bool)
%%
% In this code, initial values for the vector-invariants and moving frame are calculated
% based on an Average Screw Axis (ASA) frame.
%
% Input:
%   vector_trajectory (Nx3)
%   bool                    : {0,1} boolean. If set to 1, it is ensured that the initial object invariants are always positive
% Output:
%   invariants       (Nx3)  : initial values for the vector invariants
%   R_FS             (3x3xN): initial values for the moving frame
%

N = size(vector_trajectory,1);
T_ASA = calculate_ASA_pose([vector_trajectory,zeros(N,3)]);

% Select the signed directions of the axes of the initial moving frames
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
mean_vector = sum(vector_trajectory(1:end-round(N/4),:),1)*3/N;
if dot(e_x,mean_vector) < 0
    e_x = -e_x;
end
if dot(e_y,mean_vector) < 0
    e_y = -e_y;
end
e_z = cross(e_x,e_y);

% Update the orientation of the ASA with the signed directions
T_ASA(1:3,1:3) = [e_x,e_y,e_z];

trajectory_in_ASA = vector_trajectory*T_ASA(1:3,1:3);

invariants_output = zeros(N,3);
R_FS_output = zeros(3,3,N);
if ~bool
    for k = 1:N
        invariants_output(k,:) = [trajectory_in_ASA(k,1),0.01,0.01];
        R_FS_output(:,:,k) = T_ASA(1:3,1:3);
    end
else % case object invariant positive
    for k = 1:N
        if trajectory_in_ASA(k,1) < 0
            invariants_output(k,:) = [-trajectory_in_ASA(k,1),0.01,0.01];
            R_FS_output(:,:,k) = [-T_ASA(1:3,1),T_ASA(1:3,2),cross(-T_ASA(1:3,1),T_ASA(1:3,2))];
        else
            invariants_output(k,:) = [trajectory_in_ASA(k,1),0.01,0.01];
            R_FS_output(:,:,k) = T_ASA(1:3,1:3);
        end
    end
end

end