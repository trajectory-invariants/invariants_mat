function data_reference_preprocessed = preprocess_contour_reference_data(data_exp_trial,settings_analysis)

% Motion is tcp wrt w
% wrench is of tcp in tcp

N = settings_analysis.N;
ref_point_motion = settings_analysis.ref_point_motion;
ref_frame_force = settings_analysis.ref_frame_force;
progress_choice = settings_analysis.progress_choice;

%% Raw data

time_ = data_exp_trial.t; % timestamp
p_raw = data_exp_trial.pos; % position
R_raw = data_exp_trial.rotm; % rotation matrix
wrench_raw = [data_exp_trial.force data_exp_trial.torque]; % wrench in load cell

%% Reparameterization of data according to chosen progress

% Calculate progress function
if strcmp(progress_choice,'arc_length')
    [progress_wrt_time,progress_equidistant] = calculate_normalized_arclength(p_raw,N);
elseif strcmp(progress_choice,'arc_angle')
    [progress_wrt_time,progress_equidistant] = calculate_normalized_anglelength(R_raw,N);
elseif strcmp(progress_choice,'time')
    progress_wrt_time = time_;
    progress_equidistant = linspace(time_(1),time_(end),N);
end
    
% Interpolate data
p_reparam = interp1(progress_wrt_time,p_raw,progress_equidistant);
R_reparam = interp_rot(progress_wrt_time,R_raw,progress_equidistant);
T_reparam = compose_pose_matrix(R_reparam,p_reparam);
wrench_reparam = interp1(progress_wrt_time,wrench_raw,progress_equidistant);
velocity_profile = [time_,progress_wrt_time];

%% Transform data to the required viewpoint and reference point

T_tcp_tr = eye(4);

% Change reference point motion
if strcmp(ref_point_motion,'tracker')
    pose = T_reparam*T_tcp_tr;
elseif strcmp(ref_point_motion,'tool_point')
    pose = T_reparam; % no change
end

% Change reference frame wrench
if strcmp(ref_frame_force,'tracker')
    wrench = transform_screw(T_tcp_tr,wrench_reparam);
elseif strcmp(ref_frame_force,'tool_point')
    wrench = wrench_reparam; % no change
elseif strcmp(ref_frame_force,'under_contour')
    wrench_world = transform_screw(inverse(T_reparam),wrench_reparam);
    p_virfs_w = ((p_tcp_w_reparam(1,:)+p_tcp_w_reparam(end,:))/2)'; % position of the virtual fs in the middle of the contour
    R_virfs_w = eye(3);
    T_virfs_w = compose_pose_matrix(R_virfs_w,p_virfs_w'); % transformation matrix of the virtual fs in the middle of the contour
    wrench = transform_screw(inverse(T_virfs_w),wrench_world);
end

%% Store data in structure 

data_reference_preprocessed.progress_ref = progress_wrt_time;
data_reference_preprocessed.pose_ref = pose;
data_reference_preprocessed.position_ref = squeeze(pose(1:3,4,:))';
data_reference_preprocessed.rotation_ref = pose(1:3,1:3,:);
data_reference_preprocessed.wrench_ref = wrench;
data_reference_preprocessed.force_ref = wrench(:,1:3);
data_reference_preprocessed.moment_ref = wrench(:,4:6);
data_reference_preprocessed.velocity_profile_ref.velocity_profile = velocity_profile';

