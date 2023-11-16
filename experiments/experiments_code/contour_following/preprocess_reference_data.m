function data_reference_preprocessed = preprocess_reference_data(data_exp_trial,settings_analysis)

%% Raw data
time_ = data_exp_trial.t; % timestamp
p_raw = data_exp_trial.pos; % position
R_raw = data_exp_trial.rotm; % rotation matrix
wrench_raw = [data_exp_trial.force data_exp_trial.torque]; % wrench in load cell

%% Reparameterization of data according to chosen progress
[progress,p_reparam,T_reparam,wrench_reparam,vel_profile] = ... 
    reparameterize(p_raw, R_raw, time_, wrench_raw, settings_analysis);

%% Transform data to the required viewpoint and reference point
T_tcp_tr = eye(4);
[pose, wrench] = transform_data_viewpoint_refpoint(T_reparam, wrench_reparam, p_reparam, settings_analysis, T_tcp_tr);

%% Store data in structure 
data_reference_preprocessed.progress = progress;
data_reference_preprocessed.pose = pose;
data_reference_preprocessed.position = squeeze(pose(1:3,4,:))';
data_reference_preprocessed.rotation = pose(1:3,1:3,:);
data_reference_preprocessed.wrench = -wrench;
data_reference_preprocessed.force = -wrench(:,1:3);
data_reference_preprocessed.moment = -wrench(:,4:6);
data_reference_preprocessed.vel_prof = vel_profile';

