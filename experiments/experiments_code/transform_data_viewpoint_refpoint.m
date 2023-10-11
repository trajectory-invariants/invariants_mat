function [pose, wrench] = transform_data_viewpoint_refpoint(T_reparam, wrench_reparam, p_reparam, settings_analysis, T_tcp_tr)

ref_point_motion = settings_analysis.ref_point_motion;
N = settings_analysis.N;
ref_frame_force = settings_analysis.ref_frame_force;

% Change reference point motion
if strcmp(ref_point_motion,'tracker')
    pose = zeros(4,4,N);
    for i=1:N
        pose(:,:,i) = T_reparam(:,:,i)*T_tcp_tr; % right-multiplication
    end
elseif strcmp(ref_point_motion,'tool_point')
    pose = T_reparam; % no change
end

% Change reference frame wrench
if strcmp(ref_frame_force,'tracker')
    wrench = zeros(N,6);
    for i=1:N
        wrench(i,:) = transform_screw(T_tcp_tr,wrench_reparam(i,:)');
    end
elseif strcmp(ref_frame_force,'tool_point')
    wrench = wrench_reparam; % no change
elseif strcmp(ref_frame_force,'under_contour')
    % first transform reference frame to world
    wrench_world = zeros(N,6);
    for i=1:N
        wrench_world(i,:) = transform_screw(inverse(T_reparam(:,:,i)),wrench_reparam(i,:)');
    end
    
    % construct transformation matrix 
    p_virfs_w = ((p_reparam(1,:)+p_reparam(end,:))/2)'; % position of the virtual fs in the middle of the contour
    R_virfs_w = eye(3);
    T_virfs_w = compose_pose_matrix(R_virfs_w,p_virfs_w'); % transformation matrix of the virtual fs in the middle of the contour
    
    wrench = zeros(N,6);
    for i=1:N
        wrench(i,:) = transform_screw(inverse(T_virfs_w(:,:,i)),wrench_world(i,:)');
    end
end
end