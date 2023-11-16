function [measured_trajectory,progress] = select_measurements(measurement_data,trajectory_type)

progress = measurement_data.progress;
switch trajectory_type
    case 'pose'
        measured_trajectory = measurement_data.pose;
    case 'rotation'
        measured_trajectory = measurement_data.rotation;
    case 'position'
        measured_trajectory = measurement_data.position;
    case 'wrench'
        measured_trajectory = measurement_data.wrench;
    case 'force'
        measured_trajectory = measurement_data.force;
    case 'moment'
        measured_trajectory = measurement_data.moment;
end
