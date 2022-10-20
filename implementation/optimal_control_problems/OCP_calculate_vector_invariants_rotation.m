classdef OCP_calculate_vector_invariants_rotation < handle
    %CLASS_FRENETSERRET_CALCULATION Calculation of Frenet-Serret frames, curvature and torsion using optimal control
    %   Detailed explanation goes here

    properties
        opti; % symbolic specification of optimization problem
        X; % all symbolic states at each sample
        U; % all symbolic controls at each sample
        P; % all symbolic parameters at each sample
        O; % evaluation of the objective
        window_length; % window size
        param_positive_obj_invariant;
        param_positive_mov_invariant;
        %h; % period between samples
        %parameterization;
        %sol; % previous solution
        %flag_first_time; % boolean to indicate first window
        %window_length; % window size
        %R_FS_0; % Frenet-Serret frame at start of window
    end

    methods
        function obj = OCP_calculate_vector_invariants_rotation(parameters)
            %CLASS_FRENETSERRET_CALCULATION Construct an instance of this class
            %   Detailed explanation goes here

            load_casadi_library();
            import casadi.*

            %% Setting parameters of optimization problem
            max_iters = initialize_parameter(parameters,'max_iters',500); % maximum number of iterations
            window_length = parameters.window.window_length; % window size
            param_positive_obj_invariant = parameters.positive_obj_invariant;
            param_positive_mov_invariant = parameters.positive_mov_invariant;
            rms_error_traj = parameters.weights.rms_error_traj;
            %parameterization = parameters.parameterization; % timebased or geometric

            %% Integrator definition
            % System states
            R_FS    = SX.sym('R_FS',3,3); % rotational Frenet-Serret frame
            R_obj   = SX.sym('R_obj',3,3); % object frame
            x       = [R_FS(:) ; R_obj(:)];

            % System controls (invariants)
            % ir1 % object rotation speed
            % ir2 % curvature speed rotational Frenet-Serret
            % ir3 % torsion speed rotational Frenet-Serret
            u = SX.sym('i',3);
            h = SX.sym('h');

            % Define a symbolic function necessary to integrate invariants in a correct geometric way
            [R_eul_plus1,R_obj_plus1] = integrator_vector_invariants_to_rot(R_FS,R_obj,u,h);
            out_plus1 = [R_eul_plus1(:) ; R_obj_plus1(:)];
            geom_integr = Function('phi', {x,u,h} , {out_plus1});

            %% Create decision variables and parameters for the non-linear optimization problem (NLP)
            opti = casadi.Opti(); % use OptiStack package from Casadi for easy bookkeeping of variables (no cumbersome indexing)

            % Define system states X (unknown object pose + moving frame pose at every time step)
            R_obj   = cell(1,window_length); % object frame
            R_FS = cell(1,window_length);  % translational Frenet-Serret frame
            X = cell(1,window_length);
            for k=1:window_length
                R_obj{k} = opti.variable(3,3); % object frame
                R_FS{k} = opti.variable(3,3); % translational Frenet-Serret frame
                X{k} =  [vec(R_FS{k});vec(R_obj{k})];
            end

            % System controls U (unknown invariants at every time step)
            U = opti.variable(3,window_length-1);

            % System parameters P (known values in optimization that need to be set right before solving)
            %R_FS_0 = opti.parameter(3,3); % initial translational Frenet-Serret frame at first sample of window
            R_obj_m = cell(1,window_length); % measured object orientation
            for k=1:window_length
                R_obj_m{k}  = opti.parameter(3,3); % measured object frame
            end
            h = opti.parameter(1,1);

            % Extra decision variables
            %if strcmp(parameterization,'geometric')
            %    Theta = opti.variable(1,1); % trajectory total length
            %end

            %% Specifying the constraints

            % Constrain rotation matrices to be orthogonal (only needed for one timestep, property is propagated by geometric integrator)
            deltaR_FS = R_FS{1}'*R_FS{1} - eye(3);
            deltaR_obj = R_obj{1}'*R_obj{1} - eye(3);
            opti.subject_to([deltaR_FS(1,1:3) deltaR_FS(2,2:3) deltaR_FS(3,3)] == 0)
            opti.subject_to([deltaR_obj(1,1:3) deltaR_obj(2,2:3) deltaR_obj(3,3)] == 0)

            % Dynamic constraints
            for k=1:window_length-1
                % Integrate current state to obtain next state
                Xk_end = geom_integr(X{k},U(:,k),h);

                % Gap closing constraint
                opti.subject_to(Xk_end==X{k+1});
            end

            % Lower bounds on control
            if param_positive_obj_invariant
                opti.subject_to(U(1,:)>=0);
            end
            if param_positive_mov_invariant
                opti.subject_to(U(2,:)>=0); % lower bounds on control
            end

            %% Specifying the objective

            % Fitting constraint to remain close to measurements
            objective_fit = 0;
            for k=1:window_length
                e_rotation = vec(triu( R_obj_m{k}'*R_obj{k} - eye(3) )); % rotation error
                objective_fit = objective_fit + dot(e_rotation,e_rotation); % apply weighting to error
            end
            opti.subject_to(objective_fit/window_length/rms_error_traj^2 < 1); % 1 degree

            % Regularization constraints to deal with singularities and noise
            objective_reg = 0;
            for k=1:window_length-1
                e_regul_abs = U([2 3],k); % absolute value invariants (force arbitrary invariants to zero)
                objective_reg = objective_reg + dot(e_regul_abs,e_regul_abs);
            end

            opti.subject_to(U(2,end) == U(2,end-1)); % Last sample has no impact on RMS error

            objective = objective_reg/(window_length-1);

            %% Define solver
            opti.minimize(objective);
            opti.solver('ipopt',struct('print_time',1),struct('max_iter',max_iters,'tol',1e-10,'print_level',0));

            %% Save variables
            obj.X.R_FS= R_FS;
            obj.X.R_obj = R_obj;
            obj.U = U;
            obj.P.R_obj_m = R_obj_m;
            %obj.P.R_FS_0 = R_FS_0;
            obj.P.h = h;
            %obj.L = L;
            %obj.parameterization = parameterization;
            obj.window_length = window_length;
            obj.opti = opti;
            obj.param_positive_obj_invariant = param_positive_obj_invariant;
            obj.param_positive_mov_invariant = param_positive_mov_invariant;
            obj.O.objective = objective;
            %obj.flag_first_time = 0;
        end

        function optimization_result = calculate_invariants(obj,meas_Obj_frames,stepsize)
            %% Initialization of invariants and FS frames

            N = obj.window_length;

%             meas_T.Obj_frames = meas_Obj_frames;
%             meas_T.Obj_location = zeros(N,3);

            % Estimate initial pose twist using a finite differences approach
            twist_init = calculate_posetwist_from_discrete_poses(meas_Obj_frames,zeros(3,N),stepsize);
            [invariants_init, FS_init] = initialize_invariants_vector(twist_init(:,1:3),obj.param_positive_obj_invariant);

            %% Set variables

            % Initialize states + controls
            for k=1:N
                obj.opti.set_initial(obj.X.R_FS{k}, FS_init(:,:,k)); %construct_init_FS_from_traj(meas_traj.Obj_location);
                obj.opti.set_initial(obj.X.R_obj{k}, meas_Obj_frames(:,:,k)); % initialized with measurement
            end

            % Initialize controls
            for k=1:N-1
                obj.opti.set_initial(obj.U(:,k), invariants_init(k,1:3)); % initialized with a guess
            end

            % Set values parameters
            %obj.opti.set_value(obj.P.R_FS_0, FS_init(:,:,1)); % initialized with a guess
            for k=1:N
                obj.opti.set_value(obj.P.R_obj_m{k},  meas_Obj_frames(:,:,k));  % initialized with measurement
            end
            obj.opti.set_value(obj.P.h,stepsize);

            % Initialize results structure
            optimization_result.FS_frames = zeros(3,3,N);
            optimization_result.Obj_frames = zeros(3,3,N);
            optimization_result.invariants = zeros(N-1,3);
            optimization_result.L = 0;

            % Solve window

            % Check value objective function in initial values
            %opti.value(opti.f,opti.initial())

            %disp(['window ' num2str(1) '-' num2str(window_length) '  (' num2str(N) ')'])
            solution = obj.opti.solve_limited();
            disp(['solved in ' num2str(solution.stats.iter_count) ' iterations and ' num2str(solution.stats.t_proc_total) ' seconds'])
            %sol.value(opti.g,opti.initial())


            % Fetch the different parts from the solution vector and store in a structure
            disp(' ')
            disp(strcat(['Final value of the objective function: ', num2str(sqrt(solution.value(obj.O.objective))), ' [rad/-]']))
            disp(' ')

            optimization_result.FS_frames = reshape(solution.value([obj.X.R_FS{:}]),3,3,N);
            optimization_result.Obj_frames = reshape(solution.value([obj.X.R_obj{:}]),3,3,N);
            optimization_result.invariants = solution.value(obj.U)';
            %             if strcmp(obj.parameterization,'geometric') % c
            %                 %optimization_result.L = solution.value(obj.L);
            %             end
            optimization_result.invariants = [optimization_result.invariants ; optimization_result.invariants(end,:)];
            optimization_result.solving_time = solution.stats.t_proc_total;
            %obj.sol = solution;
        end
    end
end

