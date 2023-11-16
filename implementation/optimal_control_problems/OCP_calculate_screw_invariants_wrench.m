classdef OCP_calculate_screw_invariants_wrench < handle
    %CASADI_SAI Summary of this class goes here
    %   Detailed explanation goes here

    properties
        opti; % symbolic specification of optimization problem
        X; % all symbolic states at each sample
        U; % all symbolic controls at each sample
        P; % all symbolic parameters at each sample
        O; % value for the objective
        param_positive_obj_invariant;
        param_positive_mov_invariant;
        window_length; % window size
        %T_isa_0; % ISA frame at start of window
        parameters;

        %parameterization;
        %sol; % previous solution
        %flag_first_time; % boolean to indicate first window
    end

    methods

        function obj = OCP_calculate_screw_invariants_wrench(parameters)
            %CASADI_SAI Construct an instance of this class
            %   Detailed explanation goes here

            load_casadi_library();
            import casadi.*

            %% Setting parameters of optimization problem

            max_iters = initialize_parameter(parameters,'max_iters',500); % maximum number of iterations
            window_length = parameters.window.window_length; % window size
            param_positive_obj_invariant = parameters.positive_obj_invariant;
            param_positive_mov_invariant = parameters.positive_mov_invariant;
            rms_error_orientation = parameters.weights.rms_error_orientation;
            rms_error_translation = parameters.weights.rms_error_translation;

            characteristic_length = parameters.weights.L; % scaling factor for rotations
            %% Define a symbolic function necessary to integrate invariants in a correct geometric way

            % System states
            T_isa  = SX.sym('T_isa',3,4); % instantaneous screw axis frame
            x = [T_isa(:)];

            % System controls (invariants)
            u = SX.sym('i',6);
            h = SX.sym('h');

            % Define a geometric integrator for SAI, (meaning rigid-body motion is perfectly integrated assuming constant invariants)
            [T_isa_plus1] = integrator_screw_invariants_to_screw(T_isa,u,h);
            out_plus1 = [T_isa_plus1(:)];
            geom_integr = Function('phi', {x,u,h} , {out_plus1});

            %% Create decision variables and parameters for the non-linear optimization problem (NLP)

            opti = casadi.Opti(); % use OptiStack package from Casadi for easy bookkeeping of variables (no cumbersome indexing)

            % Define system states X (unknown object pose + moving frame pose at every time step)
            T_isa = cell(1,window_length);% instantaneous screw axis frame
            v_obj = cell(1,window_length); % object translation
            omega_obj = cell(1,window_length); % object rotation
            X = cell(1,window_length);
            for k=1:window_length
                T_isa{k}  = opti.variable(3,4); % instantaneous screw axis frame
                v_obj{k} = opti.variable(3,1); % object translation
                omega_obj{k}  = opti.variable(3,1); % object rotation
                X{k} =  [vec(T_isa{k})];
            end

            % System controls U (unknown invariants at every time step)
            U = opti.variable(6,window_length);

            % System parameters P (known values that need to be set right before solving)
            %T_isa_0 = opti.parameter(3,4); % initial ISA frame at first sample of window
            v_obj_m = cell(1,window_length); % measured object translational velocity
            omega_obj_m = cell(1,window_length); % measured object rotational velocity
            for k=1:window_length
                v_obj_m{k} = opti.parameter(3,1); % measured object translational velocity
                omega_obj_m{k}  = opti.parameter(3,1); % measured object rotational velocity
            end
            h=opti.parameter(1);
            % Extra decision variables
            %             if strcmp(parameterization,'geometric')
            %                 L = opti.variable(1,1); % trajectory total length
            %                 Theta = opti.variable(1,1); % trajectory total angle
            %             end

            %% Specifying the constraints

            % Constrain rotation matrices to be orthogonal (only needed for one timestep, property is propagated by geometric integrator)
            opti.subject_to(T_isa{1}(1:3,1:3)'*T_isa{1}(1:3,1:3) - eye(3) == 0);
            %opti.subject_to(R_obj{1}'*R_obj{1} - eye(3) == 0);
            %deltaR_isa = T_isa{1}(1:3,1:3)'*T_isa{1}(1:3,1:3) - eye(3);
            %opti.subject_to(vec(triu(deltaR_isa)) == 0)

            % Dynamic constraints using a multiple shooting approach
            for k=1:window_length-1
                % Integrate current state to obtain next state
                Xk_end = geom_integr(X{k},U(:,k),h); % geometric integrator

                % "Close the gap" constraint for multiple shooting
                opti.subject_to(Xk_end==X{k+1});
            end

            % Twist constraint: measured twist of object in world frame is equal to estimated twist of object from invariants in isa frame
            % t_w : twist of object expressed in world
            % t_isa : twist of object expressed in isa (using invariants omega1 / v1)
            % constraint : t_w - S_w_isa * t_isa == 0 % with S_w_isa, the screw transformation matrix to transform twist from isa to world
            for k=1:window_length
                opti.subject_to([omega_obj{k};v_obj{k}] - transform_screw(T_isa{k},[U(1,k);0;0;U(4,k);0;0]) == 0);
                %opti.subject_to(transform_screw(inverse_pose(T_isa{k}),[omega_obj{k};v_obj{k}]) - [U(1,k);0;0;U(4,k);0;0] == 0);
            end

            % Lower bounds on control
            if param_positive_obj_invariant
                opti.subject_to(U(1,:)>=0);
            end
            if param_positive_mov_invariant
                opti.subject_to(U(2,:)>=0); % lower bounds on control
            end
            % opti.subject_to(U(2,end) == U(2,end-1)); % Last sample has no impact on RMS error

            % Hinge motion constraint
            % slower convergence, move constraint to initialization
            % if isfield(parameters,'hinge_motion')
            %     direction = parameters.hinge_motion.direction_vertical;
            %     for k=1:window_length-1
            %         T_isa_k = T_isa{k};
            %         opti.subject_to(dot(T_isa_k(:,1),direction) >= 0);
            %     end
            % end

            %% Specifying the objective

            % Fitting constraint to remain close to measurements
            objective_fit_force = 0;
            objective_fit_torque = 0;
            for k=1:window_length
                e_translation = v_obj{k} - v_obj_m{k}; % translation error
                e_rotation = omega_obj{k} - omega_obj_m{k}; % rotation error
                objective_fit_force = objective_fit_force + dot(e_rotation,e_rotation); % apply weighting to error
                objective_fit_torque = objective_fit_torque + dot(e_translation,e_translation); % apply weighting to error
            end
            opti.subject_to(objective_fit_force/window_length/(rms_error_orientation^2) < 1); %
            opti.subject_to(objective_fit_torque/window_length/(rms_error_translation^2) < 1);

            % Regularization constraints to deal with singularities and noise
            objective_reg = 0;
            for k=1:window_length
                e_regul_abs = U([2 3 5 6],k); % absolute value invariants (force arbitrary invariants to zero)
                objective_reg = objective_reg + e_regul_abs(1)^2 + e_regul_abs(2)^2 + ...
                    1/characteristic_length^2*(e_regul_abs(3)^2 + e_regul_abs(4)^2);
            end
            objective = objective_reg/(window_length);
            
%             objective_ISA_location = 0;
%             for k=1:window_length
%                 objective_ISA_location = objective_ISA_location + dot(T_isa{k}(1:3,4),T_isa{k}(1:3,4));
%             end
%             objective_ISA_location = objective_ISA_location/(window_length);
%             objective = objective + 10^(-6)*objective_ISA_location;
            
            %% Define solver
            opti.minimize(objective);
            opti.solver('ipopt',struct('print_time',1),struct('max_iter',max_iters,'tol',10e-10,'print_level',1));

            %% Save window variables
            obj.X.v_obj = v_obj;
            obj.X.omega_obj = omega_obj;
            obj.X.T_isa = T_isa;
            obj.U = U;
            %            obj.L = L;
            %            obj.Theta = Theta;
            obj.P.v_obj_m = v_obj_m;
            obj.P.omega_obj_m = omega_obj_m;
            obj.O.objective = objective;
            obj.O.objective_reg = objective_reg;
            %obj.T_isa_0 = T_isa_0;
            %obj.parameterization = parameterization;
            obj.param_positive_obj_invariant = param_positive_obj_invariant;
            obj.param_positive_mov_invariant = param_positive_mov_invariant;
            obj.window_length = window_length;
            obj.P.h = h;
            obj.opti = opti;
            obj.parameters = parameters;

            %obj.flag_first_time = 0;
        end

        function optim_result = calculate_invariants(obj,meas_twist,stepsize)

            N = obj.window_length;
            regul_origin_ASA = obj.parameters.regul_origin_ASA;


            [invariants_init, ISA_frame_init] = initialize_invariants_screw(meas_twist,obj.param_positive_obj_invariant,regul_origin_ASA);

            measured_twist_rotation = meas_twist(:,1:3);
            measured_twist_translation = meas_twist(:,4:6);

            % Initialize states
            for k=1:N
                obj.opti.set_initial(obj.X.T_isa{k}, [ISA_frame_init(1:3,1:3,k),ISA_frame_init(1:3,4,k)]);
                obj.opti.set_initial(obj.X.v_obj{k}, measured_twist_translation(k,:)); % initialized with measurement
                obj.opti.set_initial(obj.X.omega_obj{k}, measured_twist_rotation(k,:)); % initialized with measurement
            end

            % Initialize controls
            for k=1:N-1
                obj.opti.set_initial(obj.U(:,k), invariants_init(k,:)); % initialized with a guess
            end

            % Set values parameters
            %obj.opti.set_value(obj.T_isa_0, T_isa_init(1:3,:,1)); % initialized with a guess
            for k=1:N
                obj.opti.set_value(obj.P.v_obj_m{k}, measured_twist_translation(k,:));  % initialized with measurement
                obj.opti.set_value(obj.P.omega_obj_m{k}, measured_twist_rotation(k,:));  % initialized with measurement
            end
            obj.opti.set_value(obj.P.h,stepsize);

            % Initialize extra decision variables
            %             if strcmp(obj.parameterization,'geometric')
            %                 obj.opti.set_initial(obj.L,L_init); % TODO take from input data
            %                 obj.opti.set_initial(obj.Theta,Theta_init); % TODO take from input data
            %             end

            % Initialize results structure
            optim_result.ISA_frames = zeros(3,4,N);
            optim_result.Obj_rotation = zeros(N,3);
            optim_result.Obj_translation = zeros(N,3);
            optim_result.invariants = zeros(N-1,6);
            %             optim_result.L = 0;
            %             optim_result.Theta = 0;

            % Solve window

            %disp(['window ' num2str(1) '-' num2str(window_length) '  (' num2str(N) ')'])
            solution = obj.opti.solve_limited();
            %disp(['solved in ' num2str(solution.stats.iter_count) ' iterations and '])% num2str(solution.stats.t_wall_total) ' seconds'])
            %sol.value(opti.g,opti.initial())

            disp(' ')
            disp(strcat(['Final value of the objective function: ', num2str(sqrt(solution.value(obj.O.objective))), ' [rad/-]']))
            disp(' ')

            optim_result.ISA_frames = reshape(solution.value([obj.X.T_isa{:}]),3,4,N);
            optim_result.Obj_rotation = solution.value([obj.X.omega_obj{:}])';
            optim_result.Obj_translation = solution.value([obj.X.v_obj{:}])';
            optim_result.invariants = solution.value(obj.U)';
            optim_result.invariants = optim_result.invariants;
            %             if strcmp(obj.parameterization,'geometric')
            %                 optim_result.L = sol.value(obj.L);
            %                 optim_result.Theta = sol.value(obj.Theta);
            %             end


            optim_result.solving_time = solution.stats.t_proc_total;
            %obj.sol = solution;
        end
    end
end

