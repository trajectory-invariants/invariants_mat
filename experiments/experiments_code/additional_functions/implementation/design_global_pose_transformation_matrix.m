function T_transform = design_global_pose_transformation_matrix(measured_pose)
% This function builds a pose transformation matrix: T_transform which is intended to be used for a
% left-multiplication (i.e. T_new = T_transform*T_old).
% The matrix T_transform is build from smaller elementary operations (T1,
% T2, T3, T4)
    T1_inverse = inverse_pose([eye(3),measured_pose(1:3,4,50); 0 0 0 1]); % translate the measurements to the origin
    T2 = [rot_y(160)*rot_z(-20)*rot_y(80), [0;0;0];0 0 0 1]; % apply a small rotation
    T3 = [eye(3), [-0.2;0;-0.3];0 0 0 1]; % apply a small translation
    T1 = [eye(3),measured_pose(1:3,4,50); 0 0 0 1];
    T_transform = T1*(T3*T2)*T1_inverse; % remark that a kind of 'similarity transformation' was used here
end