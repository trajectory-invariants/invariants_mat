classdef OCP_calculate_vector_invariants_position_minimumjerk < handle
    %CLASS_FRENETSERRET_CALCULATION Calculation of Frenet-Serret frames, curvature and torsion using optimal control
    %   Detailed explanation goes here

    properties
        opti; % symbolic specification of optimization problem
        X; % all symbolic states at each sample
        U; % all symbolic controls at each sample
        P; % all symbolic parameters at each sample

        %h; % period between samples
        %parameterization;
        %sol; % previous solution
        %flag_first_time; % boolean to indicate first window
        window_length; % window size
        %R_FS_0; % Frenet-Serret frame at start of window
    end

    methods
        function obj = OCP_calculate_vector_invariants_position_minimumjerk(parameters)
            %CLASS_FRENETSERRET_CALCULATION Construct an instance of this class
            %   Detailed explanation goes here

            load_casadi_library();
            import casadi.*

            %% Setting parameters of optimization problem

            %h = parameters.h; % interval between samples (could be expressed in time, arc length or dimensionless)
            max_iters = initialize_parameter(parameters, 'nb_iters', 400);
            window_length = parameters.window.window_length; % window size
            %parameterization = parameters.parameterization; % timebased or geometric
            %  param_signed_invariants = parameters.signed_invariants;
            param_positive_obj_invariant = parameters.positive_obj_invariant;
            param_positive_mov_invariant = parameters.positive_mov_invariant;

            % Weighting factors used in objective function
            weight_pos = parameters.weights.weight_accuracy; % weight in objective function on measured position
            w = parameters.weights.weight_regularization; % regularization on derivative of invariants ensures noise smoothing
            weight_regul_deriv = [w;w;w;0;0;0]; %parameters.weights.weight_regul_deriv; % regularization on derivative of invariants ensures noise smoothing
            %weight_regul_singularities = parameters.weights.weight_regularization_singularities; % regularization on norm of invariants {ir2, ir3, it2, it3} to force undefined invariants to zero
            weight_regul_singularities = initialize_parameter(parameters.weights,'weight_regularization_singularities',10e-10);

            %% Define a symbolic function necessary to integrate invariants in a correct geometric way

            % System states
            R_FSt = MX.sym('R_FSt',3,3); % translational Frenet-Serret frame
            p_obj = MX.sym('p_obj',3,1); % object position
            i1dot = MX.sym('i1dot',1,1);
            i1 = MX.sym('i1',1,1);
            i2 = MX.sym('i2',1,1);
            x = [R_FSt(:) ; p_obj ; i1dot ; i1 ; i2];

            % System controls (invariants)
            i1ddot = MX.sym('i1ddot');
            i2dot = MX.sym('i2dot');
            i3 = MX.sym('i3');
            u = [i1ddot;i2dot;i3];
            h = MX.sym('h');
            %u = MX.sym('i',3);

            % Define a geometric integrator for eFSI, (meaning rigid-body motion is perfectly integrated assuming constant invariants)
            invariants = [i1;i2;i3];
            [R_FS_plus1,p_obj_plus1] = integrator_vector_invariants_to_pos(R_FSt,p_obj,invariants,h);

            i1dotplus1 = i1dot + i1ddot * h;
            i1plus1 = i1 + i1dot * h + i1ddot * h^2/2;
            i2plus1 = i2 + i2dot * h;

            out_plus1 = [R_FS_plus1(:) ; p_obj_plus1 ;i1dotplus1;i1plus1;i2plus1 ];
            geom_integr = Function('phi', {x,u,h} , {out_plus1});

            %% Create decision variables and parameters for the non-linear optimization problem (NLP)

            opti = casadi.Opti(); % use OptiStack package from Casadi for easy bookkeeping of variables (no cumbersome indexing)

            % Define system states X (unknown object pose + moving frame pose at every time step)
            p_obj = cell(1,window_length); % object position
            R_FSt = cell(1,window_length);  % translational Frenet-Serret frame
            i1dot = cell(1,window_length);
            i1 = cell(1,window_length);
            i2 = cell(1,window_length);
            X = cell(1,window_length);
            for k=1:window_length
                p_obj{k} = opti.variable(3,1); % object position
                R_FSt{k} = opti.variable(3,3); % translational Frenet-Serret frame
                i1dot{k} = opti.variable(1,1);
                i1{k} = opti.variable(1,1);
                i2{k} = opti.variable(1,1);

                X{k} =  [vec(R_FSt{k});p_obj{k};i1dot{k};i1{k};i2{k}];
            end
            L = opti.variable(1,1); % trajectory total length
            % System controls U (unknown invariants at every time step)
            i1ddot = opti.variable(1,window_length-1);
            i2dot = opti.variable(1,window_length-1);
            i3 = opti.variable(1,window_length-1);
            U = [i1ddot;i2dot;i3];%opti.variable(3,window_length-1);

            % System parameters P (known values in optimization that need to be set right before solving)
            R_FS_0 = opti.parameter(3,3); % initial translational Frenet-Serret frame at first sample of window
            p_obj_m = cell(1,window_length); % measured object position
            for k=1:window_length
                p_obj_m{k} = opti.parameter(3,1); % measured object position
            end
            h = opti.parameter(1,1);

            % Extra decision variables
            %             if strcmp(parameterization,'geometric')
            %                 L = opti.variable(1,1); % trajectory total length
            %             end

            %% Specifying the constraints

            % Constrain rotation matrices to be orthogonal (only needed for one timestep, property is propagated by geometric integrator)
            opti.subject_to(R_FSt{1}'*R_FSt{1} == eye(3));

            for k=1:window_length-1
                %opti.subject_to(U(1,k) == Theta) % First rotational invariant is constant and equal to Theta, gives a constant progression in rotation
                %opti.subject_to(i1{k} == L) % First translational invariant is constant and equal to L, gives a constant progression in translation
            end

            % Dynamic constraints
            for k=1:window_length-1
                % Integrate current state to obtain next state
                Xk_end = geom_integr(X{k},U(:,k),h);
                %                 Xk_end = rk4(ode_simp,h,X{k},U(:,k)); % old integrator

                % Gap closing constraint
                opti.subject_to(Xk_end==X{k+1});
            end

            % % Geometry constraints % commented by Ali
            % if strcmp(parameterization,'geometric')
            %     opti.subject_to(L>=0); % total length is always positive
            %     for k=1:window_length-1
            %         opti.subject_to(U(4,k) == L) % First translational invariant is constant and equal to L, gives a constant progression in translation
            %     end
            % end

            % Lower bounds on control
            if param_positive_obj_invariant
                opti.subject_to(U(1,:)>=0);
            end
            if param_positive_mov_invariant
                opti.subject_to(U(2,:)>=0); % lower bounds on control
            end

            % Planar constraint
            %             if isfield(parameters,'contour_tracking_FSt')
            %                 opti.subject_to(U(1,:)>=0); % lower bounds on control
            %
            %                 direction = parameters.contour_tracking_FSt.direction_vertical;
            %                 for k=1:window_length-1
            %                     R_FSt_k = R_FSt{k};
            %                     opti.subject_to(dot(R_FSt_k(:,2),direction) > 0);
            %                 end
            %             end

            %% Specifying the objective

            % Fitting constraint to remain close to measurements
            objective_fit = 0;
            for k=1:window_length
                e_position = p_obj{k} - p_obj_m{k}; % position error
                objective_fit = objective_fit + 1/window_length*weight_pos*dot(e_position,e_position); % apply weighting to error
            end

            % Regularization constraints
            objective_reg = 0;
            for k=1:window_length-1
                jk = jerk_invariant(i1{k},i1dot{k},i1ddot(k),i2{k},i2dot(k),i3(k),0,0,0,0,0,0);

                jk_weighted = weight_regul_deriv.^(1/2).*jk';
                objective_reg = objective_reg + 1/(window_length-1) * dot(jk_weighted,jk_weighted);
            end

            for k=1:window_length-1

                e_movingframe = [i2{k} ; i3{k}];
                objective_reg = objective_reg + 1/(window_length-1) * weight_regul_singularities * dot(e_movingframe,e_movingframe);

            end


            %             % Regularization constraints to deal with singularities and noise
            %             objective_reg = 0;
            %             for k=1:window_length-1
            %                 if k~=1
            %                     e_regul_deriv = U(:,k) - U(:,k-1); % first-order finite backwards derivative (noise smoothing effect)
            %                     %e_regul_vel = [R_FSt{k}*[U(1,k);0;0]-R_FSt{k-1}*[U(1,k-1);0;0]];
            %                 else
            %                     e_regul_deriv = 0;
            %                     %e_regul_vel = 0;
            %                 end
            %                 e_regul_abs = U([2 3],k); % absolute value invariants (force arbitrary invariants to zero)
            %
            %                 % apply weighting to errors, note: we also weigh with ||omega||, a small omega means close to singularity so increase effect regularization
            %                 e_regul_deriv_weighted = weight_regul_deriv.^(1/2).*e_regul_deriv;
            %                 e_regul_abs_weighted = weight_regul_abs.^(1/2).*e_regul_abs;
            %                 %e_regul_vel_weighted = 0;%[ones(3,1)*weight_regul_deriv(1)].^(1/2).*e_regul_vel; % TODO improve
            %
            %                 objective_reg = objective_reg + dot(e_regul_deriv_weighted,e_regul_deriv_weighted) + dot(e_regul_abs_weighted,e_regul_abs_weighted);% + dot(e_regul_vel_weighted,e_regul_vel_weighted);
            %             end

            objective = objective_fit + objective_reg;


            %% Define solver
            opti.minimize(objective);
            opti.solver('ipopt',struct('print_time',1),struct('max_iter',max_iters,'tol',10e-6,'print_level',5));

            %% Save variables

            obj.X.R_FSt = R_FSt;
            obj.X.p_obj = p_obj;
            obj.X.i1dot = i1dot;
            obj.X.i1 = i1;
            obj.X.i2 = i2;

            obj.U.i1ddot = i1ddot;
            obj.U.i2dot = i2dot;
            obj.U.i3 = i3;

            obj.P.p_obj_m = p_obj_m;
            obj.P.R_FS_0 = R_FS_0;
            obj.P.h = h;
            %obj.L = L;
            %obj.parameterization = parameterization;
            obj.window_length = window_length;
            %obj.h = h;
            obj.opti = opti;
%            obj.flag_first_time = 0;
        end

        function optim_result = calculate_invariants(obj,measured_position,stepsize, invariants_init, FS_init)
            N = obj.window_length;
            %             measured_position = meas_traj.Obj_location;
            %
            %             % Estimate initial pose twist using a finite differences approach
            %             twist_init = calculate_posetwist_from_discrete_poses(meas_traj,obj.h);
            %             twist_init = smoothdata(twist_init,'gaussian',40); % ENSURE A SMOOTH INITIALISATION => AVOID INITIALISATIONS WITH FRAMEFLIPS
            %             %plot_descriptor(twist_init,[],'initial twists',h,parameterization,'pose_twist')
            %
            %             % Initialize invariants and moving frames over the whole horizon using discretized analytical formulas
            %             parameters = struct(); % optional arguments
            %             parameters.signed_invariants = 1; % allow all invariants to become either positive or negative, otherwise omega1 and omega2 are always positive but X-axis and Y-axis may flip
            %             [FSt_init,~,invariants_init] = calculate_eFSI_from_discrete_twist(twist_init,obj.h,parameters);
            %             tol = 1e-7;
            %             invariants_init = invariants_init(:,4:6) + tol;

            %         Pdiff = diff(measured_position);
            %         ex = Pdiff / np.linalg.norm(Pdiff,axis=1).reshape(N-1,1)
            %         ex = np.vstack((ex,[ex[-1,:]]))
            %         ey = np.tile( np.array((0,0,1)), (N,1) )
            %         ez = np.array([np.cross(ex[i,:],ey[i,:]) for i in range(N)])
            %

            % for i=1:N-1
            %     ex = measured_position(i+1,:)-measured_position(i,:);
            %     ex = ex'/norm(ex);
            %     ey = [0;0;1];
            %     ez = cross(ex,ey);
            %     FSt_init(:,:,i) = [ex ey ez];
            % end
            % FSt_init(:,:,N) = FSt_init(:,:,N-1);

            % [FSt_init,~,~] = calculate_eFSI_from_discrete_twist(twist_init,obj.h,parameters);
            %L_init = 1;

            tol = 1e-7;


            % Initialize states + controls
            for k=1:N
                obj.opti.set_initial(obj.X.R_FSt{k}, FS_init(:,:,k)); %construct_init_FS_from_traj(meas_traj.Obj_location);
                obj.opti.set_initial(obj.X.p_obj{k}, measured_position(k,:)); % initialized with measurement
                obj.opti.set_initial(obj.X.i1dot{k}, tol);
                obj.opti.set_initial(obj.X.i1{k}, invariants_init(k,1));
                obj.opti.set_initial(obj.X.i2{k}, invariants_init(k,2));

            end

            % Initialize controls
            for k=1:N-1
                obj.opti.set_initial(obj.U.i1ddot(k), tol);
                obj.opti.set_initial(obj.U.i2dot(k), tol);
                obj.opti.set_initial(obj.U.i3(k), invariants_init(k,3));
            end

            % Set values parameters
            obj.opti.set_value(obj.P.R_FS_0, FS_init(:,:,1)); % initialized with a guess
            for k=1:N
                obj.opti.set_value(obj.P.p_obj_m{k}, measured_position(k,:));  % initialized with measurement
            end
            obj.opti.set_value(obj.P.h,stepsize);

            % Initialize extra decision variables
            %             if strcmp(obj.parameterization,'geometric')
            %                 obj.opti.set_initial(obj.L,L_init); % TODO take from input data, e.g. sum(|| p_k - p_k-1 ||)
            %             end

            % Initialize results structure
            optim_result.FS_frames = zeros(3,3,N);
            optim_result.Obj_location = zeros(N,3);
            optim_result.invariants = zeros(N-1,3);
            optim_result.L = 0;

            % Solve window

            % Check value objective function in initial values
            %opti.value(opti.f,opti.initial())

            %disp(['window ' num2str(1) '-' num2str(window_length) '  (' num2str(N) ')'])
            solution = obj.opti.solve_limited();
            disp(['solved in ' num2str(solution.stats.iter_count) ' iterations and ' num2str(solution.stats.t_proc_total) ' seconds'])
            %sol.value(opti.g,opti.initial())


            % Fetch the different parts from the solution vector and store in a structure
            optim_result.FS_frames = reshape(solution.value([obj.X.R_FSt{:}]),3,3,N);
            optim_result.Obj_location = solution.value([obj.X.p_obj{:}])';
            %optimization_result.invariants = solution.value(obj.U)';
            %             if strcmp(obj.parameterization,'geometric') % c
            %                 %optimization_result.L = solution.value(obj.L);
            %             end


            i1_sol = solution.value([obj.X.i1{:}]);
            i1dot_sol = solution.value([obj.X.i1dot{:}]);
            i1ddot_sol = [solution.value([obj.U.i1ddot{:}]) solution.value([obj.U.i1ddot{end}])];
            i2_sol = solution.value([obj.X.i2{:}]);
            i2dot_sol = [solution.value([obj.U.i2dot{:}]) solution.value([obj.U.i2dot{end}])];
            i3_sol = [solution.value([obj.U.i3{:}]) solution.value([obj.U.i3{end}])];

            optim_result.invariants = [i1_sol' i2_sol' i3_sol'];

            optim_result.velocity_invariant = [i1_sol;zeros(2,N)]';
            optim_result.acceleration_invariant = [i1dot_sol;-i1_sol.*i2_sol;zeros(1,N)]';
            optim_result.jerk_invariant = jerk_invariant(i1_sol,i1dot_sol,i1ddot_sol,i2_sol,i2dot_sol,i3_sol,0,0,0,0,0,0);

            optim_result.solving_time = solution.stats.t_proc_total;

            %obj.sol = solution;

        end

        %         function optimization_result = calculate_invariants_online(obj,meas_traj,cut_index)
        %             % n = index corresponding to first sample in window
        %
        %             if ~obj.flag_first_time
        %                 obj.flag_first_time = 1;
        %                 optimization_result = obj.calculate_invariants(meas_traj);
        %
        %                 % Add extra starting constraints, necessary for preserving continuity of the solution over different windows
        %                 obj.opti.subject_to(obj.X.p_obj{1} == obj.P.p_obj_m{1}); % start from given object position
        %                 obj.opti.subject_to(obj.X.R_FSt{1} == obj.P.R_FS_0); % start from given FS frame
        %             else
        %
        %                 % Pass solution of second part of previous window to first part of next window (faster convergence)
        %                 %opti.set_initial(sol.value_variables())
        %
        %                 N = obj.window_length;
        %
        %                 for k=1:N-cut_index
        %                     obj.opti.set_initial(obj.X.R_FSt{k}, obj.sol.value([obj.X.R_FSt{cut_index+k}]));
        %                     obj.opti.set_initial(obj.X.p_obj{k}, obj.sol.value([obj.X.p_obj{cut_index+k}]));
        %                 end
        %                 for k=1:N-cut_index-1
        %                     obj.opti.set_initial(obj.U(:,k),  obj.sol.value(obj.U(:,cut_index+k)));
        %                 end
        %                 %obj.opti.set_initial(obj.L,obj.sol.value(obj.L));
        %
        %                 % Initialize second part of next window
        %                 twist_init = calculate_posetwist_from_discrete_poses(meas_traj,obj.h);
        %                 parameters = struct();
        %                 [FSt_init,~,invariants_init] = calculate_eFSI_from_discrete_twist(twist_init,obj.h,parameters);
        %                 invariants_init = invariants_init(:,4:6);
        %                 measured_position = meas_traj.Obj_location;
        %                 for k=N-cut_index+1:N
        %                     obj.opti.set_initial(obj.X.R_FSt{k}, FSt_init(:,:,k-1));
        %                     obj.opti.set_initial(obj.X.p_obj{k}, measured_position(k-1,:));
        %                 end
        %                 for k=N-cut_index:N-1
        %                     obj.opti.set_initial(obj.U(:,k), invariants_init(k-1,:));
        %                 end
        %
        %                 % Set values of parameters that determine the starting constraints that guarantee continuity
        %                 obj.opti.set_value(obj.P.R_FS_0, orthonormalize_rotation(obj.sol.value([obj.X.R_FSt{1+cut_index}])));
        %                 obj.opti.set_value(obj.P.p_obj_m{1}, obj.sol.value([obj.X.p_obj{1+cut_index}]));
        %
        %                 % Set other parameters equal to the measurements in that window
        %                 for k=2:N
        %                     obj.opti.set_value(obj.P.p_obj_m{k}, measured_position(k-1,:));
        %                 end
        %
        %
        %                 % Solve this window
        %                 solution = obj.opti.solve();
        %                 disp(['solved in ' num2str(solution.stats.iter_count) ' iterations and ' num2str(solution.stats.t_proc_total) ' seconds'])
        %
        %                 % Fetch the different parts from the solution vector and store in a structure
        %                 optimization_result.FS_frames = reshape(solution.value([obj.X.R_FSt{:}]),3,3,N);
        %                 optimization_result.Obj_location = solution.value([obj.X.p_obj{:}])';
        %                 optimization_result.invariants = solution.value(obj.U)';
        %
        %                 obj.sol = solution;
        %
        %             end
        %        end
    end
end

