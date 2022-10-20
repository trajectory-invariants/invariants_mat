classdef OCP_calculate_screw_invariants_pose_old < handle
    %CASADI_SAI Summary of this class goes here
    %   Detailed explanation goes here

    properties
        opti; % symbolic specification of optimization problem
        X; % all symbolic states at each sample
        U; % all symbolic controls at each sample
        P; % all symbolic parameters at each sample
        O; % value for the objective

        window_length; % window size
        param_signed_invariants;
        %T_isa_0; % ISA frame at start of window
        %parameterization;
        %sol; % previous solution
        %flag_first_time; % boolean to indicate first window
    end

    methods

        function obj = OCP_calculate_screw_invariants_pose_old(parameters)
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
            weight_accuracy_orientation = parameters.weights.weight_accuracy_orientation; % weight in objective function on measurements
            weight_accuracy_position = parameters.weights.weight_accuracy_position; % weight in objective function on measurements
            weight_regul_deriv_obj = parameters.weights.weight_regul_deriv_obj; % regularization on derivative of object invariants ensures noise smoothing
            weight_regul_deriv_mf = parameters.weights.weight_regul_deriv_mf; % regularization on derivative of moving frame invariants ensures noise smoothing
            weight_regul_abs_mf = parameters.weights.weight_regul_abs_mf; % regularization on absolute of moving frame invariants ensures noise smoothing
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

            %             % Geometry constraints
            %             if strcmp(parameterization,'geometric')
            %                 opti.subject_to(L>=0); % total length is always positive
            %                 opti.subject_to(Theta>=0); % total angle is always positive
            %                 for k=1:window_length-1
            %                     %weight_doa = 0.5;
            %                     %   opti.subject_to(weight_doa*norm(U(1,k))/Theta + (1-weight_doa)*norm(U(4,k))/L == 1) % this constraints demands there is a constant progression in rotation+translation
            %                 end
            %             end

            % Lower bounds on control
            %             opti.subject_to(U(1,:)>=0);
            if ~param_signed_invariants
                opti.subject_to(U(1,:)>=0);
                opti.subject_to(U(2,:)>=0);
            end
            opti.subject_to(U(2,end) == U(2,end-1)); % Last sample has no impact on RMS error

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
                T_obj_k = T_obj{k};
                T_obj_m_k = T_obj_m{k};
                e_position = T_obj_k(1:3,4) - T_obj_m_k(1:3,4); % position error
                e_rotation = T_obj_m_k(1:3,1:3)'*T_obj_k(1:3,1:3) - eye(3); % rotation error
                e_rotation = vec(triu(e_rotation));
                objective_fit = objective_fit + weight_accuracy_position*dot(e_position,e_position) + weight_accuracy_orientation*dot(e_rotation,e_rotation); % apply weighting to error
            end

            % Regularization constraints to deal with singularities and noise
            objective_reg = 0;
            for k=1:window_length-1
                if k~=1
                    e_regul_deriv = U(:,k) - U(:,k-1); % first-order finite backwards derivative (noise smoothing effect)
                    % e_regul_vel = 0;% transform_screw(T_isa{k},[U(1,k);0;0;U(4,k);0;0]) - transform_screw(T_isa{k-1},[U(1,k-1);0;0;U(4,k-1);0;0]);
                    %  e_regul_vel = 0;
                    %elseif k==window_length-1
                    %e_regul_deriv = U(:,window_length-1) - U(:,window_length-2);
                else
                    e_regul_deriv = 0;%2*U(:,k) - U(:,k-1) - U(:,k+1);
                end

                e_regul_abs = U([2 3 5 6],k); % absolute value invariants (force arbitrary invariants to zero)

                weight_regul_deriv = [weight_regul_deriv_obj 0 0  weight_regul_deriv_obj/characteristic_length^2 0 0]';
                weight_regul_abs = [weight_regul_abs_mf weight_regul_abs_mf weight_regul_abs_mf/characteristic_length^2 weight_regul_abs_mf/characteristic_length^2]';

                e_regul_deriv_weighted = weight_regul_deriv.^(1/2).*e_regul_deriv;
                e_regul_abs_weighted = weight_regul_abs.^(1/2).*e_regul_abs;
                %                 e_regul_vel_weighted = [ones(3,1)*weight_regul_deriv(1);ones(3,1)*weight_regul_deriv(4)].^(1/2).*e_regul_vel; % TODO improve

                objective_reg = objective_reg + dot(e_regul_deriv_weighted,e_regul_deriv_weighted) + dot(e_regul_abs_weighted,e_regul_abs_weighted);% + dot(e_regul_vel_weighted,e_regul_vel_weighted);
            end

            objective = objective_fit + objective_reg;

            %% Define solver
            opti.minimize(objective);
            opti.solver('ipopt',struct('print_time',1),struct('max_iter',max_iters,'tol',10e-16,'print_level',0));

            %% Save window variables
            obj.X.T_obj = T_obj;
            %obj.X.R_obj = R_obj;
            obj.X.T_isa = T_isa;
            obj.U = U;
            %            obj.L = L;
            %            obj.Theta = Theta;
            obj.P.T_obj_m = T_obj_m;
            
            obj.O.objective = objective;
            obj.O.objective_fit = objective_fit;
            obj.O.objective_reg = objective_reg;
            
            obj.O.objective = objective;
            obj.O.objective_fit = objective_fit;
            obj.O.objective_reg = objective_reg;

            %obj.T_isa_0 = T_isa_0;
            %obj.parameterization = parameterization;
            obj.window_length = window_length;
            obj.P.h = h;
            obj.opti = opti;
            obj.param_signed_invariants = param_signed_invariants;
            %obj.flag_first_time = 0;
        end

        function optim_result = calculate_invariants(obj,meas_poses,stepsize)

            %import casadi.*

            measured_orientation = meas_poses(1:3,1:3,:);
            measured_position = squeeze(meas_poses(1:3,4,:))';
            N = obj.window_length;

            % Estimate initial screw twist using a finite differences approach
            twist_init = calculate_screwtwist_from_discrete_poses(measured_orientation,measured_position',stepsize);
            %twist_init = smoothdata(twist_init,'gaussian',20); % ENSURE A SMOOTH INITIALISATION => AVOID INITIALISATIONS WITH FRAMEFLIPS


            ASA = calculate_ASA_pose(twist_init(1:round(N/2,0),:));
            end_ind = round(N/2,0);
            mean_twist = sum(twist_init(:,1:3),1)/N;
            mean_twist = ASA(1:3,1);
            mean_normal = ASA(1:3,2);
            %figure; hold on; axis equal; plot3(twist_init(1:end_ind,1),twist_init(1:end_ind,2),twist_init(1:end_ind,3),'.'); plot3(0,0,0,'r.'); plot3(mean_twist(1),mean_twist(2),mean_twist(3),'g.'); plot3(mean_normal(1),mean_normal(2),mean_normal(3),'k.');

            % Initialize invariants and moving frames over the whole horizon using discretized analytical formulas
            parameters.signed_invariants = obj.param_signed_invariants; % optional arguments
            parameters.direction_vector_x = mean_twist;
            parameters.direction_vector_y = ASA(1:3,2);
            [T_isa_init,invariants_init] = calculate_screw_invariants_from_discrete_twist(twist_init,stepsize,parameters);

            %figure; plot(invariants_init(:,2))
            %disp(invariants_init(1,2))

            %   N = obj.window_length;

            invariants_init = zeros(size(invariants_init))+1e-12;
            %for i=1:N
            %    T_isa_init(:,:,i)= eye(4);
            %end


            % Initialize states
            for k=1:N
                obj.opti.set_initial(obj.X.T_isa{k}, ASA(1:3,:));
                obj.opti.set_initial(obj.X.T_obj{k}, meas_poses(1:3,:,k)); % initialized with measurement
            end

            % Initialize controls
            for k=1:N-1
                obj.opti.set_initial(obj.U(:,k), invariants_init(k,:)); % initialized with a guess
            end

            % Set values parameters
            %obj.opti.set_value(obj.T_isa_0, T_isa_init(1:3,:,1)); % initialized with a guess
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
            
            optim_result.objective = solution.value(obj.O.objective);
            optim_result.objective_fit = solution.value(obj.O.objective_fit);
            optim_result.objective_reg = solution.value(obj.O.objective_reg);
            optim_result.objective
            optim_result.objective_fit
            optim_result.objective_reg
            optim_result.objective = solution.value(obj.O.objective);
            optim_result.objective_fit = solution.value(obj.O.objective_fit);
            optim_result.objective_reg = solution.value(obj.O.objective_reg);
            optim_result.objective
            optim_result.objective_fit
            optim_result.objective_reg
            optim_result.ISA_frames = reshape(solution.value([obj.X.T_isa{:}]),3,4,N);
            optim_result.Obj_frames = reshape(solution.value([obj.X.T_obj{:}]),3,4,N);
            optim_result.invariants = solution.value(obj.U)';
            optim_result.invariants = [ optim_result.invariants ; optim_result.invariants(end,:) ];
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

