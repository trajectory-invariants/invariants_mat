classdef OCP_calculate_screw_invariants_pose < handle
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
        parameters;
        %T_isa_0; % ISA frame at start of window
        %parameterization;
        %sol; % previous solution
        %flag_first_time; % boolean to indicate first window
    end

    methods

        function obj = OCP_calculate_screw_invariants_pose(parameters)
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

            %parameterization = initialize_parameter(parameters,'parameterization','timebased'); % timebased or geometric
            %h = parameters.h; % time between samples

            characteristic_length = parameters.weights.L; % scaling factor for rotations
            %weight_doa = 0.5;

            %% Define a symbolic function necessary to integrate invariants in a correct geometric way

            % System states
            T_isa  = SX.sym('T_isa',3,4); % instantaneous screw axis frame
            T_obj = SX.sym('T_obj',3,4); % object frame
            x = [T_isa(:) ; T_obj(:)];

            % System controls (invariants)
            u = SX.sym('i',6);
            h = SX.sym('h');

            % Define a geometric integrator for SAI, (meaning rigid-body motion is perfectly integrated assuming constant invariants)
            [T_isa_plus1,T_obj_plus1] = integrator_screw_invariants_to_pose(T_isa,T_obj,u,h);
            out_plus1 = [T_isa_plus1(:) ; T_obj_plus1(:)];
            geom_integr = Function('phi', {x,u,h} , {out_plus1});

            %% Create decision variables and parameters for the non-linear optimization problem (NLP)

            opti = casadi.Opti(); % use OptiStack package from Casadi for easy bookkeeping of variables (no cumbersome indexing)

            % Define system states X (unknown object pose + moving frame pose at every time step)
            T_isa = cell(1,window_length);% instantaneous screw axis frame
            T_obj = cell(1,window_length); % object frame
            X = cell(1,window_length);
            for k=1:window_length
                T_isa{k}  = opti.variable(3,4); % instantaneous screw axis frame
                T_obj{k}  = opti.variable(3,4); % object frame
                X{k} =  [vec(T_isa{k});vec(T_obj{k})];
            end

            % System controls U (unknown invariants at every time step)
            U = opti.variable(6,window_length-1);

            % System parameters P (known values that need to be set right before solving)
            %T_isa_0 = opti.parameter(3,4); % initial ISA frame at first sample of window
            T_obj_m = cell(1,window_length); % measured object orientation
            h = opti.parameter(1,1);
            for k=1:window_length
                T_obj_m{k}  = opti.parameter(3,4); % measured object frame
            end

            % Extra decision variables
            %             if strcmp(parameterization,'geometric')
            %                 L = opti.variable(1,1); % trajectory total length
            %                 Theta = opti.variable(1,1); % trajectory total angle
            %             end

            %% Specifying the constraints

            % Constrain rotation matrices to be orthogonal (only needed for one timestep, property is propagated by geometric integrator)
            %            opti.subject_to(T_isa{1}(1:3,1:3)'*T_isa{1}(1:3,1:3) - eye(3) == 0);
            %           opti.subject_to(T_obj{1}(1:3,1:3)'*T_obj{1}(1:3,1:3) - eye(3) == 0);
            deltaR_isa = T_isa{1}(1:3,1:3)'*T_isa{1}(1:3,1:3) - eye(3);
            deltaR_obj = T_obj{1}(1:3,1:3)'*T_obj{1}(1:3,1:3) - eye(3);
            opti.subject_to([deltaR_isa(1,1:3) deltaR_isa(2,2:3) deltaR_isa(3,3)] == 0)
            opti.subject_to([deltaR_obj(1,1:3) deltaR_obj(2,2:3) deltaR_obj(3,3)] == 0)

            % Dynamic constraints using a multiple shooting approach
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

            %% Specifying the objective

            % Fitting constraint to remain close to measurements
            objective_fit_position = 0;
            objective_fit_orientation = 0;
            for k=1:window_length
                T_obj_k = T_obj{k};
                T_obj_m_k = T_obj_m{k};
                e_position = T_obj_k(1:3,4) - T_obj_m_k(1:3,4); % position error
                e_rotation = T_obj_m_k(1:3,1:3)'*T_obj_k(1:3,1:3) - eye(3); % rotation error
%                 e_rotation = [e_rotation(1,2);e_rotation(1,3);e_rotation(2,3)];
                e_rotation = vec(triu(e_rotation));
                objective_fit_position = objective_fit_position + dot(e_position,e_position);
                objective_fit_orientation = objective_fit_orientation + dot(e_rotation,e_rotation); % apply weighting to error
            end
            opti.subject_to(objective_fit_orientation/window_length/(rms_error_orientation^2) < 1); %
            opti.subject_to(objective_fit_position/window_length/(rms_error_translation^2) < 1);


            % Regularization constraints to deal with singularities and noise
            objective_reg = 0;
            for k=1:window_length-1
                e_regul_abs = U([2 3 5 6],k); % absolute value invariants (force arbitrary invariants to zero)
                objective_reg = objective_reg + e_regul_abs(1)^2 + e_regul_abs(2)^2 + ...
                    1/characteristic_length^2*(e_regul_abs(3)^2 + e_regul_abs(4)^2);
            end
            objective = objective_reg/(window_length-1);

            %% Define solver
            opti.minimize(objective);
            opti.solver('ipopt',struct('print_time',1,'expand',true),struct('max_iter',max_iters,'tol',10e-8,'print_level',5));

            %% Save window variables
            obj.X.T_obj = T_obj;
            %obj.X.R_obj = R_obj;
            obj.X.T_isa = T_isa;
            obj.U = U;
            %            obj.L = L;
            %            obj.Theta = Theta;
            obj.P.T_obj_m = T_obj_m;

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

        function optim_result = calculate_invariants(obj,meas_poses,stepsize)

            %import casadi.*
            regul_origin_ASA = obj.parameters.regul_origin_ASA;
            

            measured_orientation = meas_poses(1:3,1:3,:);
            measured_position = squeeze(meas_poses(1:3,4,:))';
            N = obj.window_length;

            % Estimate initial screw twist using a finite differences approach
            twist_init = calculate_screwtwist_from_discrete_poses(measured_orientation,measured_position',stepsize);
            [invariants_init, ISA_frame_init] = initialize_invariants_screw(twist_init,obj.param_positive_obj_invariant,regul_origin_ASA);

            % Initialize states
            for k=1:N
                obj.opti.set_initial(obj.X.T_isa{k}, [ISA_frame_init(1:3,1:3,k),ISA_frame_init(1:3,4,k)]);
                obj.opti.set_initial(obj.X.T_obj{k}, meas_poses(1:3,:,k)); % initialized with measurement
            end

            % Initialize controls
            for k=1:N-1
                obj.opti.set_initial(obj.U(:,k), invariants_init(k,:)); % initialized with a guess
            end

            % Set values parameters
            for k=1:N
                obj.opti.set_value(obj.P.T_obj_m{k}, meas_poses(1:3,:,k));  % initialized with measurement
            end
            obj.opti.set_value(obj.P.h,stepsize);

            % Initialize extra decision variables
            %             if strcmp(obj.parameterization,'geometric')
            %                 obj.opti.set_initial(obj.L,L_init); % TODO take from input data
            %                 obj.opti.set_initial(obj.Theta,Theta_init); % TODO take from input data
            %             end

            % Initialize results structure
            optim_result.ISA_frames = zeros(3,3,N);
            optim_result.Obj_frames = zeros(3,3,N);
            optim_result.Obj_location = zeros(N,3);
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

            optim_result.objective = solution.value(obj.O.objective);
            optim_result.objective_reg = solution.value(obj.O.objective_reg);
            optim_result.moving_frames = reshape(solution.value([obj.X.T_isa{:}]),3,4,N);
            optim_result.reconstruction = reshape(solution.value([obj.X.T_obj{:}]),3,4,N);
            optim_result.invariants = solution.value(obj.U)';
            optim_result.invariants = [ optim_result.invariants ; optim_result.invariants(end,:) ];
            %             if strcmp(obj.parameterization,'geometric')
            %                 optim_result.L = sol.value(obj.L);
            %                 optim_result.Theta = sol.value(obj.Theta);
            %             end

            optim_result.solving_time = solution.stats.t_proc_total;
            %obj.sol = solution;
        end
    end
end

