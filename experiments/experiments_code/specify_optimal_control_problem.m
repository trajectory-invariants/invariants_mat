function OCP = specify_optimal_control_problem(parameters_OCP,trajectory_type)

switch trajectory_type
    case 'pose'
        OCP = OCP_calculate_screw_invariants_pose(parameters_OCP);
    case 'rotation'
        OCP = OCP_calculate_vector_invariants_rotation(parameters_OCP);
    case 'position'
        OCP = OCP_calculate_vector_invariants_position(parameters_OCP);
    case 'wrench'
        OCP = OCP_calculate_screw_invariants_wrench(parameters_OCP);
    case 'force'
        parameters_OCP.weights.rms_error_traj = parameters_OCP.weights.rms_error_force;
        OCP = OCP_calculate_vector_invariants_vector(parameters_OCP);
    case 'moment'
        parameters_OCP.weights.rms_error_traj = parameters_OCP.weights.rms_error_moment;
        OCP = OCP_calculate_vector_invariants_vector(parameters_OCP);
end

