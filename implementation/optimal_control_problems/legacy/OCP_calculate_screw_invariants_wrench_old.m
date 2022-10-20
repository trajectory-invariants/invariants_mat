classdef OCP_calculate_screw_invariants_wrench_old < handle
    %CASADI_SAI Summary of this class goes here
    %   Detailed explanation goes here

    properties
        opti; % symbolic specification of optimization problem
        X; % all symbolic states at each sample
        U; % all symbolic controls at each sample
        P; % all symbolic parameters at each sample
        window_length; % window size
        signed_invariants;
        %T_isa_0; % ISA frame at start of window
        %parameterization;
        %sol; % previous solution
        %flag_first_time; % boolean to indicate first window
    end

    methods

        function obj = OCP_calculate_screw_invariants_wrench_old(parameters)
            %CASADI_SAI Construct an instance of this class
            %   Detailed explanation goes here

            load_casadi_library();
            import casadi.*

            %% Setting parameters of optimization problem

            max_iters = initialize_parameter(parameters,'max_iters',500); % maximum number of iterations
            window_length = parameters.window.window_length; % window size
            param_signed_invariants = parameters.signed_invariants;
            %parameterization = initialize_parameter(parameters,'parameterization','timebased'); % timebased or geometric
            %h = parameters.h; % time between samples

            % Weighting factors used in objective function
            weight_accuracy_force = parameters.weights.weight_accuracy_force; % weight in objective function on measurements
            weight_accuracy_torque = parameters.weights.weight_accuracy_torque; % weight in objective function on measurements
            weight_regul_deriv_obj = parameters.weights.weight_regul_deriv_obj; % regularization on derivative of object invariants ensures noise smoothing
            weight_regul_deriv_mf = parameters.weights.weight_regul_deriv_mf; % regularization on derivative of moving frame invariants ensures noise smoothing
            weight_regul_abs_mf = parameters.weights.weight_regul_abs_mf; % regularization on absolute of moving frame invariants ensures noise smoothing
            L = parameters.weights.L; % scaling factor for rotations

            %weight_doa = 0.5;

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
            U = opti.variable(6,window_length-1);

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
            opti.subject_to(T_isa{1}(1:3,1:3)'*T_isa{1}(1:3,1:3) == eye(3));
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

            %             % Geometry constraints
            %             if strcmp(parameterization,'geometric')
            %                 opti.subject_to(L>=0); % total length is always positive
            %                 opti.subject_to(Theta>=0); % total angle is always positive
            %                 for k=1:window_length-1
            %                     %weight_doa = 0.5;
            %                     %   opti.subject_to(weight_doa*norm(U(1,k))/Theta + (1-weight_doa)*norm(U(4,k))/L == 1) % this constraints demands there is a constant progression in rotation+translation
            %                 end
            %             end

            % Twist constraint: measured twist of object in world frame is equal to estimated twist of object from invariants in isa frame
            % t_w : twist of object expressed in world
            % t_isa : twist of object expressed in isa (using invariants omega1 / v1)
            % constraint : t_w - S_w_isa * t_isa == 0 % with S_w_isa, the screw transformation matrix to transform twist from isa to world
            for k=1:window_length-1
                opti.subject_to([omega_obj{k};v_obj{k}] - transform_screw(T_isa{k},[U(1,k);0;0;U(4,k);0;0]) == 0);
                %opti.subject_to(transform_screw(inverse_pose(T_isa{k}),[omega_obj{k};v_obj{k}]) - [U(1,k);0;0;U(4,k);0;0] == 0);
            end

            % Lower bounds on control
            if ~param_signed_invariants
                opti.subject_to(U(1,:)>=0);
                opti.subject_to(U(2,:)>=0);
            end

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
            objective_fit = 0;
            for k=1:window_length
                e_translation = v_obj{k} - v_obj_m{k}; % translation error
                e_rotation = omega_obj{k} - omega_obj_m{k}; % rotation error
                objective_fit = objective_fit + weight_accuracy_torque*dot(e_translation,e_translation) + weight_accuracy_force*dot(e_rotation,e_rotation); % apply weighting to error
            end

            % Regularization constraints to deal with singularities and noise
            objective_reg = 0;
            for k=1:window_length-1
                if k~=1
                    e_regul_deriv = U(:,k) - U(:,k-1);
                else
                    e_regul_deriv = 0;
                end
                e_regul_abs = U([2 3 5 6],k); % absolute value invariants (force arbitrary invariants to zero)

                weight_regul_deriv = [weight_regul_deriv_obj 0 0  weight_regul_deriv_obj/L^2 0 0]';
                weight_regul_abs = [weight_regul_abs_mf weight_regul_abs_mf weight_regul_abs_mf/L^2 weight_regul_abs_mf/L^2]';

                e_regul_deriv_weighted = weight_regul_deriv.^(1/2).*e_regul_deriv;
                e_regul_abs_weighted = weight_regul_abs.^(1/2).*e_regul_abs;

                objective_reg = objective_reg + dot(e_regul_deriv_weighted,e_regul_deriv_weighted) + dot(e_regul_abs_weighted,e_regul_abs_weighted);% + dot(e_regul_vel_weighted,e_regul_vel_weighted);
            end

            objective = objective_fit + objective_reg;

            %% Define solver
            opti.minimize(objective);
            opti.solver('ipopt',struct('print_time',1),struct('max_iter',max_iters,'tol',10e-7,'print_level',0));

            %% Save window variables
            obj.X.v_obj = v_obj;
            obj.X.omega_obj = omega_obj;
            obj.X.T_isa = T_isa;
            obj.U = U;
            %            obj.L = L;
            %            obj.Theta = Theta;
            obj.P.v_obj_m = v_obj_m;
            obj.P.omega_obj_m = omega_obj_m;
            %obj.T_isa_0 = T_isa_0;
            %obj.parameterization = parameterization;
            obj.window_length = window_length;
            obj.P.h = h;
            obj.opti = opti;
            obj.signed_invariants = param_signed_invariants;
            %obj.flag_first_time = 0;
        end

        function optim_result = calculate_invariants(obj,meas_twist,stepsize)

            N = obj.window_length;
%             meas_twist = smoothdata(meas_twist,'gaussian',10); % ENSURE A SMOOTH INITIALISATION => AVOID INITIALISATIONS WITH FRAMEFLIPS
% 
            %import casadi.*

            measured_twist_rotation = meas_twist(:,1:3);
            measured_twist_translation = meas_twist(:,4:6);

            % Estimate initial screw twist using a finite differences approach
            %twist_init = calculate_screwtwist_from_discrete_poses(measured_orientation,measured_position',stepsize);


            if 1
                % Initialize invariants and moving frames over the whole horizon using discretized analytical formulas
                parameters.signed_invariants = obj.signed_invariants; % optional arguments
                meas_twist = smoothdata(meas_twist,'gaussian',20); % ENSURE A SMOOTH INITIALISATION => AVOID INITIALISATIONS WITH FRAMEFLIPS
                ASA = calculate_ASA_pose(meas_twist(1:round(N),:));
                parameters.direction_vector_x = ASA(1:3,1);
                parameters.direction_vector_y = cross(meas_twist(1,1:3)',meas_twist(round(N/2),1:3)');
                [T_isa_init,invariants_init] = calculate_screw_invariants_from_discrete_twist(meas_twist,stepsize,parameters);
                invariants_init = invariants_init+1e-12;
                %figure; plot(invariants_init(:,2))

            else
                start_ind = round(1,0);
                end_ind = round(N/2,0);
                %figure; hold on; axis equal;
                mean_twist_vec = sum(meas_twist(:,1:3),1)/N;
                %plot3(0,0,0,'r.');
                %plot3(meas_twist(end_ind,1),meas_twist(end_ind,2),meas_twist(end_ind,3),'b.','MarkerSize',20);
                %plot3(mean_twist_vec(1),mean_twist_vec(2),mean_twist_vec(3),'bx','MarkerSize',20);
                %plot3(meas_twist(start_ind:end_ind,1),meas_twist(start_ind:end_ind,2),meas_twist(start_ind:end_ind,3),'b.');

                ASA = calculate_ASA_pose(meas_twist(start_ind:end_ind,:));

                mean_twist = ASA(1:3,1);
                mean_normal = ASA(1:3,2);
                % plot3(mean_twist(1),mean_twist(2),mean_twist(3),'g.'); plot3(mean_normal(1),mean_normal(2),mean_normal(3),'k.');


                for i=1:N
                    T_isa_init(:,:,i) = ASA;
                end
                invariants_init = zeros(N-1,6)+1e-6;
                %invariants_init(:,1) = 0;
            end




            %for i=1:N
            %    T_isa_init(:,:,i)= eye(4);
            %end


            % Initialize states
            for k=1:N
                obj.opti.set_initial(obj.X.T_isa{k}, T_isa_init(1:3,:,k));
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

            optim_result.ISA_frames = reshape(solution.value([obj.X.T_isa{:}]),3,4,N);
            optim_result.Obj_rotation = solution.value([obj.X.omega_obj{:}])';
            optim_result.Obj_translation = solution.value([obj.X.v_obj{:}])';
            optim_result.invariants = solution.value(obj.U)';
            optim_result.invariants = [optim_result.invariants ; optim_result.invariants(end,:)];
            %             if strcmp(obj.parameterization,'geometric')
            %                 optim_result.L = sol.value(obj.L);
            %                 optim_result.Theta = sol.value(obj.Theta);
            %             end

            %     % Add extra starting constraints, necessary for preserving continuity of the solution over different windows
            %     %if ~isfield(parameters,'hinge_motion')
            %     opti.subject_to(p_obj{1} == p_obj_m{1}); % start from given object position
            %     opti.subject_to(R_obj{1} == R_obj_m{1}); % start from given object orientation
            %     opti.subject_to(T_isa{1} == T_isa_0); % start from given ISA frame

            optim_result.solving_time = solution.stats.t_proc_total;
            %obj.sol = solution;
        end
    end
end

