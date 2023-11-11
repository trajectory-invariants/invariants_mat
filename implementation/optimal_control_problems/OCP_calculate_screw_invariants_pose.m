classdef OCP_calculate_screw_invariants_pose < handle
    %OCP_calculate_screw_invariants_pose Optimal control problem
    %formulation to calculate screw invariants from pose trajectories. 
    %   The reason for making this a class is to allow 
	%	the problem specification to be reused in an easier
	% 	way than with separate functions. The definition of all variables
	%	is encapsulated within this class, to that they not need
	%	to be copied each time to the function that calculates the invariants. 
	%	The specification can be reused for calculating different trials,
	% 	but also for moving-window optimization.

    properties
        opti; % symbolic specification of optimization problem
        X; % all symbolic states at each sample
        U; % all symbolic controls at each sample
        P; % all symbolic parameters at each sample
        O; % value for the objective
        param_positive_obj_invariant;
        param_positive_mov_invariant;
        window_length; % window size
        regul_origin_ASA; % values of parameters
     end

    methods
        function obj = OCP_calculate_screw_invariants_pose(parameters)
            %OCP_calculate_screw_invariants_pose Construct an instance of this class
            %   Specification of optimal control problem in a symbolic way

            load_casadi_library();
            import casadi.*
            opti = casadi.Opti(); % use OptiStack package from Casadi for easy bookkeeping of variables (no cumbersome indexing)

            %% Setting parameters of optimization problem
            max_iters = initialize_parameter(parameters,'max_iters',500); % maximum number of iterations
            window_length = parameters.window.window_length; % window size
            param_positive_obj_invariant = parameters.positive_obj_invariant;
            param_positive_mov_invariant = parameters.positive_mov_invariant;
            rms_error_orientation = parameters.weights.rms_error_orientation;
            rms_error_translation = parameters.weights.rms_error_translation;
            characteristic_length = parameters.weights.L; % scaling factor for rotations
 
            %% Create decision variables and parameters for the non-linear optimization problem (NLP)

            % Define system states X (unknown object pose + moving frame pose at every time step)
            T_isa = cell(1,window_length);% moving ISA frame
            T_obj = cell(1,window_length); % object frame
            X = cell(1,window_length);
            for k=1:window_length
                T_isa{k}  = opti.variable(3,4); % moving ISA frame
                T_obj{k}  = opti.variable(3,4); % object frame
                X{k} =  [vec(T_isa{k});vec(T_obj{k})];
            end

            % System controls U (unknown invariants at every time step)
            % order: [a omega_kappa omega_tau b v_kappa v_tau ]
            U = opti.variable(6,window_length-1);

            % System parameters P (known values that need to be set right before solving)
            T_obj_m = cell(1,window_length); % measured object frame
            h = opti.parameter(1,1);
            for k=1:window_length
                T_obj_m{k}  = opti.parameter(3,4); % measured object frame
            end

            %% Specifying the constraints

            % Constrain rotation matrices to be orthogonal (only needed for one timestep, property is propagated by geometric integrator)
            deltaR_isa = T_isa{1}(1:3,1:3)'*T_isa{1}(1:3,1:3) - eye(3);
            deltaR_obj = T_obj{1}(1:3,1:3)'*T_obj{1}(1:3,1:3) - eye(3);
            opti.subject_to([deltaR_isa(1,1:3) deltaR_isa(2,2:3) deltaR_isa(3,3)] == 0)
            opti.subject_to([deltaR_obj(1,1:3) deltaR_obj(2,2:3) deltaR_obj(3,3)] == 0)
			% opti.subject_to(T_isa{1}(1:3,1:3)'*T_isa{1}(1:3,1:3) - eye(3) == 0);
            % opti.subject_to(T_obj{1}(1:3,1:3)'*T_obj{1}(1:3,1:3) - eye(3) == 0);

            % Dynamic constraints using a multiple shooting approach
            geom_integr = cas_integrator_screw_invariants(); % Define a symbolic function necessary to integrate invariants in a correct geometric way
            for k=1:window_length-1
                % Integrate current state to obtain next state
                Xk_end = geom_integr(X{k},U(:,k),h); % geometric integrator

                % "Close the gap" constraint for multiple shooting
                opti.subject_to(Xk_end==X{k+1});
            end

            % Lower bounds on control
            if param_positive_obj_invariant
                opti.subject_to(U(1,:)>=0);
            end
            if param_positive_mov_invariant
                opti.subject_to(U(2,:)>=0); % lower bounds on control
            end
            opti.subject_to(U(2,end) == U(2,end-1)); % Last sample has no impact on RMS error

            % Trajectory error constraint
            traj_error_position = 0;
            traj_error_orientation = 0;
            for k=1:window_length
                T_obj_k = T_obj{k};
                T_obj_m_k = T_obj_m{k};
                e_position = T_obj_k(1:3,4) - T_obj_m_k(1:3,4); % position error
                e_rotation = T_obj_m_k(1:3,1:3)'*T_obj_k(1:3,1:3) - eye(3); % rotation error
                e_rotation = vec(triu(e_rotation));

                traj_error_position = traj_error_position + dot(e_position,e_position);
                traj_error_orientation = traj_error_orientation + dot(e_rotation,e_rotation);
            end
            opti.subject_to(traj_error_orientation/window_length/(rms_error_orientation^2) < 1); %
            opti.subject_to(traj_error_position/window_length/(rms_error_translation^2) < 1);

            %% Specifying the objective

            % Regularization constraints to deal with singularities and noise
            objective_reg = 0;
            for k=1:window_length-1
                e_regul_abs = U([2 3 5 6],k); % extract moving frame invariants
                objective_reg = objective_reg + e_regul_abs(1)^2 + e_regul_abs(2)^2 + ...
                    1/characteristic_length^2*(e_regul_abs(3)^2 + e_regul_abs(4)^2);
            end
            objective = objective_reg/(window_length-1);

            %% Define solver
            opti.minimize(objective);
            opti.solver('ipopt',struct('print_time',1,'expand',true),struct('max_iter',max_iters,'tol',10e-8,'print_level',5));

            %% Save window variables
            obj.opti = opti;
            obj.X.T_obj = T_obj;
            obj.X.T_isa = T_isa;
            obj.U = U;
            obj.P.T_obj_m = T_obj_m;
            obj.P.h = h;
            obj.O.objective = objective;
            obj.O.objective_reg = objective_reg;
            obj.param_positive_obj_invariant = param_positive_obj_invariant;
            obj.param_positive_mov_invariant = param_positive_mov_invariant;
            obj.window_length = window_length;
            obj.regul_origin_ASA = parameters.regul_origin_ASA;
        end

        function optim_result = calculate_invariants(obj,meas_poses,stepsize)
            
            N = obj.window_length;

            % Initial guess of invariants and moving frames 
            twist_init = calculate_screwtwist_from_discrete_poses(meas_poses,stepsize);
            [invariants_init, ISA_frame_init] = initialize_invariants_screw(twist_init,obj.param_positive_obj_invariant,obj.regul_origin_ASA);

            % Initialize states
            for k=1:N
                obj.opti.set_initial(obj.X.T_isa{k}, ISA_frame_init(1:3,:,k)); % initialized with initial guess
                obj.opti.set_initial(obj.X.T_obj{k}, meas_poses(1:3,:,k)); % initialized with measurements
            end

            % Initialize controls
            for k=1:N-1
                obj.opti.set_initial(obj.U(:,k), invariants_init(k,:)); % initialized with initial guess
            end

            % Set values parameters
            for k=1:N
                obj.opti.set_value(obj.P.T_obj_m{k}, meas_poses(1:3,:,k));  % values set using measurements
            end
            obj.opti.set_value(obj.P.h,stepsize); % size of one step in the window

            % Solve window
            solution = obj.opti.solve_limited();
            %disp(['solved in ' num2str(solution.stats.iter_count) ' iterations and '])% num2str(solution.stats.t_wall_total) ' seconds'])
            %sol.value(opti.g,opti.initial()) % for debugging purposes

            % Store results
            optim_result.objective = solution.value(obj.O.objective);
            optim_result.objective_reg = solution.value(obj.O.objective_reg);
            optim_result.moving_frames = reshape(solution.value([obj.X.T_isa{:}]),3,4,N);
            optim_result.reconstruction = reshape(solution.value([obj.X.T_obj{:}]),3,4,N);
            optim_result.invariants = solution.value(obj.U)';
            optim_result.invariants = [optim_result.invariants ; optim_result.invariants(end,:)];
            optim_result.solving_time = solution.stats.t_proc_total;
        end
    end
end

