function [progress_equidistant, p_reparam, T_reparam, wrench_reparam, velocity_profile] = reparameterize(p_raw, R_raw, time_, wrench_raw, settings_analysis)

progress_choice = settings_analysis.progress_choice;
N = settings_analysis.N;

% Calculate progress function
if strcmp(progress_choice,'arclength')
    [progress_wrt_time,progress_equidistant] = calculate_normalized_arclength(p_raw,N);
elseif strcmp(progress_choice,'arcangle')
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
