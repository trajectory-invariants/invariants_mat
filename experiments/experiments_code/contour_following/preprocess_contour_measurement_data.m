function data_preprocessed = preprocess_contour_measurement_data(data_exp,settings_analysis)

data_preprocessed = cell(12,1);
N = settings_analysis.N;
trial_0 = settings_analysis.trial_0;
trial_n = settings_analysis.trial_n;

for trial=trial_0:trial_n
    %% Load measurement data
    data_exp_trial = data_exp{trial};

    % Extract trajectory coordinates
    time_raw = data_exp_trial.t; % timestamp
    p_raw = data_exp_trial.pos; % position
    R_raw = data_exp_trial.rotm; % rotation matrix
    T_raw = compose_pose_matrix(R_raw,p_raw); % pose matrix
    wrench_raw = [data_exp_trial.force data_exp_trial.torque]; % wrench in load cell

    % Remove duplicate samples in all data
    [~,index_unq] = unique(p_raw,'rows','stable');
    time_unq = time_raw(index_unq) - time_raw(1);
    T_unq = T_raw(:,:,index_unq);
    wrench_unq = wrench_raw(index_unq,:);

    % Rotate the world frame by -90 degrees around the X-axis so that the Z-axis is pointing upwards in vertical direction
    deltaR = inv(rot_x(-90));
    deltaT = [deltaR,[0,0,0]';[0,0,0,1]];
    for j = 1 : size(T_unq,3)
        T_unq(:,:,j) = deltaT*T_unq(:,:,j);
    end

    %% (optional) Weight compensation
    % The effect of gravity on the tool is removed from the measured
    % contact force. 
    % This step is not required but allows us to study
    % the contact force without influence of the weight.
    % Weight compensation requires geometric properties and mass of the 
    % tool, which need to be obtained from CAD data or a calibration procedure.
    if strcmp(settings_analysis.application,'contour')
        [weight_calib,transform_tcp_tr,transform_tcp_lc] = configuration_properties_contour(trial); % properties of the tool are loaded based on the tool configuration of the trial 
    elseif strcmp(settings_analysis.application,'peg')
        [weight_calib,transform_tcp_tr,transform_tcp_lc] = configuration_properties_peg(); % properties of the tool are loaded based on the tool configuration of the trial
    end
    wrench_comp = weight_compensation(T_unq,wrench_unq,weight_calib);
    
    %% Transform data to TCP (needed for segmentation + reparameterization)
    % For the contour following case, we choose to define the task progress 
    % using the translation of the tool-center-point (TCP) as a reference 
    % point. To segment and reparameterize data based on the chosen progress,
    % the measurement data need to be transformed from the tracker
    % frame (tr) to the tool-center-point frame (tcp). This requires
    % knowing the relative transformations between tcp, tr and lc.
    %
    % This is a limitation of our approach. It could be avoided by 
    % defining a progress variable that is invariant of an a priori chosen
    % reference point.

    [T_tcp,wrench_tcp] = transform_data_to_tcp(T_unq,wrench_comp,transform_tcp_tr,transform_tcp_lc);
    p_tcp = squeeze(T_tcp(1:3,4,:))';
    R_tcp = T_tcp(1:3,1:3,:);
    
    %% Segmentation start/end data based on magnitude of the velocity
    
    % Determine segment start and end
    if strcmp(settings_analysis.application,'contour')
        % based on translational velocity of TCP
        v_tcp = calculate_velocity_from_discrete_positions(p_tcp,time_unq);
        [start_segment,end_segment] = detect_velocity_segments(v_tcp,settings_analysis.velocity_translation_threshold);
    elseif strcmp(settings_analysis.application,'peg')
        % based on rotational velocity
        omega_tcp = calculate_velocity_from_discrete_rotations(R_tcp,time_unq);
        [start_segment,end_segment] = detect_velocity_segments(omega_tcp,settings_analysis.velocity_rotation_threshold);
    end

    % Segment all data
    time_ = time_unq(start_segment:end_segment,1)-time_unq(start_segment,1);
    p_segm = p_tcp(start_segment:end_segment,:);
    R_segm = R_tcp(:,:,start_segment:end_segment);
    wrench_segm = wrench_tcp(start_segment:end_segment,:);

    %% (optional) Introduce artificial variations in pose data to test invariance
    if settings_analysis.artificial_variations
        [R_segm,p_segm,~] = ...
            make_artificial_variations(R_segm,p_segm,trial);
    end

    %% (optional) Replace real contact wrench with a synthetic one
    if settings_analysis.wrench_synthetic
        wrench_segm = zeros(size(wrench_segm))+[0 -25*cos(pi/4) -25*cos(pi/4) 0 0 0];
    end

    %% Reparameterization of data to become equidistant in time (timebased), equidistant in path length (geometric), or equidistant in path angle (geometric)
    [progress,p_reparam,T_reparam,wrench_reparam,vel_profile] = ...
        reparameterize(p_segm, R_segm, time_, wrench_segm, settings_analysis);

    %% Transform data to the required viewpoint and reference point
    [pose, wrench] = transform_data_viewpoint_refpoint(T_reparam, wrench_reparam, p_reparam, settings_analysis, transform_tcp_tr);

    %% Store data
    trial_nb = trial-trial_0+1;
    data_preprocessed{trial_nb}.progress = progress;
    data_preprocessed{trial_nb}.pose = pose;
    data_preprocessed{trial_nb}.rotation = pose(1:3,1:3,:);
    data_preprocessed{trial_nb}.position = squeeze(pose(1:3,4,:))';
    data_preprocessed{trial_nb}.wrench = -wrench;
    data_preprocessed{trial_nb}.force = -wrench(:,1:3);
    data_preprocessed{trial_nb}.moment = -wrench(:,4:6);
    data_preprocessed{trial_nb}.vel_prof = vel_profile';
    data_preprocessed{trial_nb}.pose_tcp = T_reparam;
end
