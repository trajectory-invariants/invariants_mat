function [progress_all,pose_all,position_all,rotation_all,wrench_all,force_all,moment_all,...
    progress_ref,pose_ref,position_ref,rotation_ref,wrench_ref,force_ref,moment_ref,...
    velocity_profile_all,velocity_profile_ref] = ...
    load_and_preprocess_contour_measurement_data(settings_analysis)


N = settings_analysis.N;
viewpoint = settings_analysis.viewpoint;
parameterization = settings_analysis.parameterization; 
referencepoint = settings_analysis.referencepoint;
trial_0 = settings_analysis.trial_0;
trial_n = settings_analysis.trial_n;
path_to_data_folder = settings_analysis.path_to_data_folder;

%% Parameters for loading real data

path_to_data = strcat([path_to_data_folder 'data_segmented_original/',application]);

% number identifying the location and orientation of the force sensor {lc} and motion tracker {tr} on the tool
sensor_id = configurations();

% initialization
progress_all = zeros(N,trial_n-trial_0+1);
pose_all = zeros(4,4,N,trial_n-trial_0+1);
rotation_all = zeros(3,3,N,trial_n-trial_0+1);
position_all = zeros(N,3,trial_n-trial_0+1);
wrench_all = zeros(N,6,trial_n-trial_0+1);
force_all = zeros(N,3,trial_n-trial_0+1);
moment_all = zeros(N,3,trial_n-trial_0+1);
velocity_profile_all = struct();


%% Calculation of trials

data_exp = load_all_trials_contour(path_to_data);
if isempty(data_exp)
    error('wrong path to data')
end


for trial=trial_0:trial_n
    %% Load raw measurement dataQ
    
    data_exp_trial = data_exp{trial};
    
    % Extract trajectories
    time_raw = data_exp_trial.t(:,1); % timestamp
    p_tr_w_raw = data_exp_trial.pos(:,:); % position
    R_tr_w_raw = data_exp_trial.rotm(:,:,:); % rotation matrix
    T_tr_w_raw = compose_pose_matrix(R_tr_w_raw,p_tr_w_raw); % pose matrix
    wrench_lc_lc_raw = [data_exp_trial.force(:,:) data_exp_trial.torque(:,:)]; % wrench in load cell
    
    % Make all data unique by removing duplicates
    [~,index_unq] = unique(p_tr_w_raw,'rows','stable');
    time_unq = time_raw(index_unq,1);
    time_unq = time_unq - time_unq(1);
    T_tr_w_unq = T_tr_w_raw(:,:,index_unq);
    wrench_lc_lc_unq = wrench_lc_lc_raw(index_unq,:);
    
    % Rotate the world frame by -90 degrees around the X-axis so that the Z-axis is pointing upwards in vertical direction
    R_w_wp = inv(rot_x(-90));
    T_w_wp = [R_w_wp,[0,0,0]';[0,0,0,1]];
    for j = 1 : size(T_tr_w_unq,3)
        T_tr_w_unq(:,:,j) = T_w_wp*T_tr_w_unq(:,:,j);
    end
    
    %% Weight compensation and transformation of data to TCP (needed for segmentation)
    
    if strcmp(application,'contour')
        [T_tcp_tr,T_tcp_lc,T_lc_tr,p_cog_lc] = configuration_properties_contour(sensor_id(trial)); % properties of the tool are loaded based on the tool configuration of the trial
        mass = 1.96138/9.806; % mass of tool in [kg] determined with a least-squares calibration procedure
    elseif strcmp(application,'peg')
        [T_tcp_tr,T_tcp_lc,T_lc_tr,p_cog_lc] = configuration_properties_peg(); % properties of the tool are loaded based on the tool configuration of the trial
        mass = 1.80234/9.81; % mass of tool in [kg] determined with a least-squares calibration procedure
    end
    [wrench_lc_lc_weight] = weight_compensation(T_tr_w_unq,wrench_lc_lc_unq,T_lc_tr,mass,p_cog_lc);
    [T_tcp_w,wrench_tcp_tcp_reparam] = transform_data_to_tcp(T_tr_w_unq,wrench_lc_lc_weight,T_tcp_tr,T_tcp_lc);
    if strcmp(wrenchtype,'synthetic') && strcmp(application,'contour')
        wrench_tcp_tcp_reparam = zeros(size(wrench_tcp_tcp_reparam))+[0 -25*cos(pi/4) -25*cos(pi/4) 0 0 0];
    end
    p_tcp_w = squeeze(T_tcp_w(1:3,4,:))';
    R_tcp_w = T_tcp_w(1:3,1:3,:);
    
    %% Segmentation start/end data based on magnitude of the translational velocity
    
    if strcmp(application,'contour')
        % Determine start and end of segment based on translational velocity
        v_tcp_w = calculate_velocity_from_discrete_positions(p_tcp_w,time_unq);
        [start_segment,end_segment] = detect_velocity_segments(v_tcp_w,velocity_translation_threshold);
    elseif strcmp(application,'peg')
        % Determine start and end of segment based on rotational velocity
        omega_tcp_w = calculate_velocity_from_discrete_rotations(R_tcp_w,time_unq);
        [start_segment,end_segment] = detect_velocity_segments(omega_tcp_w,velocity_rotation_threshold);
    end
    
    % Segment all data
    time_segm = time_unq(start_segment:end_segment,1)-time_unq(start_segment,1);
    p_tcp_w_segm = p_tcp_w(start_segment:end_segment,:);
    R_tcp_w_segm = R_tcp_w(:,:,start_segment:end_segment);
    wrench_tcp_tcp_seg = wrench_tcp_tcp_reparam(start_segment:end_segment,:);
    
    %% Introduce artificial variations in pose data to make the case more complex
    if strcmp(application,'contour')
        [R_tcp_w_segm,p_tcp_w_segm,~] = make_artificial_variations(R_tcp_w_segm,p_tcp_w_segm,trial);
    end
    
    %% Reparameterization of data to become equidistant in time (timebased) or equidistant in path length (geometric)
    if strcmp(parameterization,'dimless_arclength')
        if strcmp(application,'contour')
            % Determine arc length
            [arclength,arclength_equidistant] = calculate_normalized_arclength(p_tcp_w_segm,N);
            % Interpolate data based on arc length
            p_tcp_w_reparam = interp1(arclength,p_tcp_w_segm,arclength_equidistant);
            R_tcp_w_reparam = interp_rot(arclength,R_tcp_w_segm,arclength_equidistant);
            wrench_tcp_tcp_reparam = interp1(arclength,wrench_tcp_tcp_seg,arclength_equidistant);
            progress = arclength_equidistant;
            velocity_profile = [time_segm,arclength];
        elseif strcmp(application,'peg')
            % Determine arc length
            [arclength,arclength_equidistant] = calculate_normalized_anglelength(R_tcp_w_segm,N);
            % Interpolate data based on angle length
            p_tcp_w_reparam = interp1(arclength,p_tcp_w_segm,arclength_equidistant);
            R_tcp_w_reparam = interp_rot(arclength,R_tcp_w_segm,arclength_equidistant);
            wrench_tcp_tcp_reparam = interp1(arclength,wrench_tcp_tcp_seg,arclength_equidistant);
            progress = arclength_equidistant;
            velocity_profile = [time_segm,arclength];
        end
    elseif strcmp(parameterization,'time_based')
        % Make time vector equidistant
        time_equidistant = linspace(time_segm(1),time_segm(end),N);
        % Interpolate data based on equidistant time
        p_tcp_w_reparam = interp1(time_segm,p_tcp_w_segm,time_equidistant);
        R_tcp_w_reparam = interp_rot(time_segm,R_tcp_w_segm,time_equidistant);
        wrench_tcp_tcp_reparam = interp1(time_segm,wrench_tcp_tcp_seg,time_equidistant);
        progress = linspace(0,time_segm(end)-time_segm(1),N)';
        velocity_profile = [time_equidistant',time_equidistant'];
    end
    
    %% Transform data to the required viewpoint and reference point
    T_tcp_w_reparam = compose_pose_matrix(R_tcp_w_reparam,p_tcp_w_reparam);
    if strcmp(viewpoint,'world')
        if strcmp(referencepoint,'tool_point')
            T_refp_w = T_tcp_w_reparam;
            pose = T_refp_w;
            wrench_refp_refp_reparam = wrench_tcp_tcp_reparam;
            for j = 1 : N
                wrench(j,:) = rotate_screw(T_refp_w(:,:,j),wrench_refp_refp_reparam(j,:));
            end
        elseif strcmp(referencepoint,'tracker')
            [T_refp_w,wrench_refp_refp_reparam] = transform_data_tcp_to_tr(T_tcp_tr,[p_tcp_w_reparam,rotm2quat(R_tcp_w_reparam)],wrench_tcp_tcp_reparam);
            pose = T_refp_w;
            for j = 1 : N
                wrench(j,:) = rotate_screw(T_refp_w(:,:,j),wrench_refp_refp_reparam(j,:));
            end
        elseif strcmp(referencepoint,'force_sensor')
            if strcmp(application,'contour')
                p_virfs_w = ((p_tcp_w_reparam(1,:)+p_tcp_w_reparam(end,:))/2)'; % position of the virtual fs in the middle of the contour
                R_virfs_w = eye(3);
                T_virfs_w = compose_pose_matrix(R_virfs_w,p_virfs_w'); % transformation matrix of the virtual fs in the middle of the contour
            elseif strcmp(application,'peg')
                p_virfs_w = p_tcp_w_reparam(end,:)'; % position of the virtual fs in the middle of the contour
                R_virfs_w = eye(3);
                T_virfs_w = compose_pose_matrix(R_virfs_w,p_virfs_w'); % transformation matrix of the virtual fs in the middle of the contour
            end
            for j=1:N
                wrench_w_w_reparam(j,:) = transform_screw(T_tcp_w_reparam(:,:,j),wrench_tcp_tcp_reparam(j,:)')'; % wrench expressed in world frame
                wrench(j,:) = transform_screw(inverse_pose(T_virfs_w),wrench_w_w_reparam(j,:)')'; % wrench expressed in virtual fs frame
            end
            pose = zeros(4,4,N);
            for k = 1:N
                pose(:,:,k) = T_virfs_w;
            end
        end
    elseif strcmp(viewpoint,'body')
        if strcmp(referencepoint,'tool_point')
            N = size(T_tcp_w_reparam,3);
            pose = zeros(4,4,N);
            pose(:,:,1) = eye(4);
            for k = 1:N-1
                test = inverse_pose(T_tcp_w_reparam(:,:,k))*T_tcp_w_reparam(:,:,k+1);
                delta_position = test(1:3,4);
                pose(1:3,1:3,k+1) = pose(1:3,1:3,k)*test(1:3,1:3);
                pose(1:3,4,k+1) = pose(1:3,4,k)+delta_position;
            end
            wrench = wrench_tcp_tcp_reparam;
        elseif strcmp(referencepoint,'tracker')
            [T_refp_w,wrench_refp_refp_reparam] = transform_data_tcp_to_tr(T_tcp_tr,[p_tcp_w_reparam,rotm2quat(R_tcp_w_reparam)],wrench_tcp_tcp_reparam);
            N = size(T_refp_w,3);
            pose = zeros(4,4,N);
            pose(:,:,1) = eye(4);
            for k = 1:N-1
                test = inverse_pose(T_refp_w(:,:,k))*T_refp_w(:,:,k+1);
                delta_position = test(1:3,4);
                pose(1:3,1:3,k+1) = pose(1:3,1:3,k)*test(1:3,1:3);
                pose(1:3,4,k+1) = pose(1:3,4,k)+delta_position;
            end
            wrench = wrench_refp_refp_reparam;
        elseif strcmp(referencepoint,'middle_contour')
            N = size(T_tcp_w_reparam,3);
            pose = zeros(4,4,N);
            for k = 1:N
                pose(:,:,k) = inverse_pose(T_tcp_w_reparam(:,:,k))*T_tcp_w_reparam(:,:,round(N/2)); % middle of the contour
            end
            wrench = zeros(N,6); % this wrench case is not supported by toolbox
        end
        
    end
    
    position = squeeze(pose(1:3,4,:))';
    rotation = pose(1:3,1:3,:);
    force = wrench(:,1:3);
    moment = wrench(:,4:6);
    
    progress_all(:,trial-trial_0+1) = progress;
    pose_all(:,:,:,trial-trial_0+1) = pose;
    rotation_all(:,:,:,trial-trial_0+1) = rotation;
    position_all(:,:,trial-trial_0+1) = position;
    wrench_all(:,:,trial-trial_0+1) = wrench;
    force_all(:,:,trial-trial_0+1) = force;
    moment_all(:,:,trial-trial_0+1) = moment;
    velocity_profile_all(trial-trial_0+1).velocity_profile = velocity_profile;
end

%% Removing extra variables and keeping necessary data

clearvars -except N_des viewpoint parameterization referencepoint trial_0 trial_n ...
    progress_all pose_all rotation_all position_all wrench_all force_all moment_all ...
    velocity_profile_all application path_to_data_folder
% Invert sign of force when viewed in the world (reaction forces)
if strcmp(viewpoint,'world')
    wrench_ref = -wrench_ref; force_ref = -force_ref; moment_ref = -moment_ref;
    wrench = -wrench; force = -force; moment = -moment;
end

