classdef OCP_calculate_vector_invariants_vector_old < handle
    %CLASS_FRENETSERRET_CALCULATION Calculation of Frenet-Serret frames, curvature and torsion using optimal control
    %   Detailed explanation goes here

    properties
        opti; % symbolic specification of optimization problem
        X; % all symbolic states at each sample
        U; % all symbolic controls at each sample
        P; % all symbolic parameters at each sample
        O; % evaluation of the objective
        window_length; % window size
        param_signed_invariants;
        %h; % period between samples
        %parameterization;
        %sol; % previous solution
        %flag_first_time; % boolean to indicate first window
        %R_FS_0; % Frenet-Serret frame at start of window
    end

    methods
        function obj = OCP_calculate_vector_invariants_vector_old(parameters)
            %CLASS_FRENETSERRET_CALCULATION Construct an instance of this class
            %   Detailed explanation goes here

            load_casadi_library();
            import casadi.*

            %% Setting parameters of optimization problem

            %h = parameters.h; % interval between samples (could be expressed in time, arc length or dimensionless)
            max_iters = initialize_parameter(parameters,'max_iters',500); % maximum number of iterations
            window_length = parameters.window.window_length; % window size
            %parameterization = parameters.parameterization; % timebased or geometric
            param_signed_invariants = parameters.signed_invariants;

            % Weighting factors used in objective function
            weight_accuracy = parameters.weights.weight_accuracy; % weight in objective function on measurements
            weight_regul_deriv_obj = parameters.weights.weight_regul_deriv_obj; % regularization on derivative of object invariants ensures noise smoothing
            weight_regul_deriv_mf = parameters.weights.weight_regul_deriv_mf; % regularization on derivative of moving frame invariants ensures noise smoothing
            weight_regul_abs_mf = parameters.weights.weight_regul_abs_mf; % regularization on absolute of moving frame invariants ensures noise smoothing
            scale_rotation = parameters.weights.scale_rotation; % scaling factor for rotations

            %% Definition integrator
            % System states
            R_FS  = SX.sym('R_FS' ,3,3); % translational Frenet-Serret frame
            x = [R_FS(:)];

            % System controls (invariants)
            % it1 % object translation speed
            % it2 % curvature speed translational Frenet-Serret
            % it3 % torsion speed translational Frenet-Serret
            u = SX.sym('i',3);
            h = SX.sym('h',1);

            % Define a geometric integrator for SAI, (meaning rigid-body motion is perfectly integrated assuming constant invariants)
            [R_FS_plus1] = integrator_vector_invariants_to_vector(R_FS,u,h);
            out_plus1 = [R_FS_plus1(:)];
            geom_integr = Function('F', {x,u,h} , {out_plus1(:)});

            %% Create decision variables and parameters for the non-linear optimization problem (NLP)

            opti = casadi.Opti(); % use OptiStack package from Casadi for easy bookkeeping of variables (no cumbersome indexing)

            % Define system states X (unknown object pose + moving frame pose at every time step)
            c_obj = cell(1,window_length); % object position
            R_FS = cell(1,window_length);  % translational Frenet-Serret frame
            X = cell(1,window_length);
            for k=1:window_length
                c_obj{k} = opti.variable(3,1); % object position
                R_FS{k} = opti.variable(3,3); % translational Frenet-Serret frame
                X{k} =  [vec(R_FS{k})];
            end

            % System controls U (unknown invariants at every time step)
            U = opti.variable(3,window_length-1);

            % System parameters P (known values in optimization that need to be set right before solving)
            %R_FS_0 = opti.parameter(3,3); % initial translational Frenet-Serret frame at first sample of window
            c_obj_m = cell(1,window_length); % measured object position
            for k=1:window_length
                c_obj_m{k} = opti.parameter(3,1); % measured object position
            end
            h = opti.parameter(1,1);

            % Extra decision variables
            %if strcmp(parameterization,'geometric')
            %    L = opti.variable(1,1); % trajectory total length
            %end

            %% Specifying the constraints

            % Constrain rotation matrices to be orthogonal (only needed for one timestep, property is propagated by geometric integrator)
            opti.subject_to(R_FS{1}'*R_FS{1} == eye(3));
            %deltaR_FS = R_FS{1}'*R_FS{1} - eye(3);
            %            opti.subject_to([deltaR_FS(1,1:3) deltaR_FS(2,2:3) deltaR_FS(3,3)] == 0)

            % Dynamic constraints
            %geom_integr = define_geom_integrator_vector_FSI_casadi(h); % Define a symbolic function necessary to integrate invariants in a correct geometric way
            for k=1:window_length-1
                % Integrate current state to obtain next state
                Xk_end = geom_integr(X{k},U(:,k),h);
                %Xk_end = rk4(ode_simp,h,X{k},U(:,k)); % old integrator

                % Gap closing constraint
                opti.subject_to(Xk_end==X{k+1});
            end

            % Twist constraint: relate object velocity in world to object velocity from invariants in moving frame
            for k=1:window_length-1
                opti.subject_to(c_obj{k} - R_FS{k}*[U(1,k);0;0] == 0);
            end

            % % Geometry constraints
            %if strcmp(parameterization,'geometric')
            %    opti.subject_to(L>=0); % total length is always positive
            %    for k=1:window_length-1
            %        opti.subject_to(U(1,k) == L) % First translational invariant is constant and equal to L, gives a constant progression in translation
            %    end
            %end
            
            % Lower bounds on control
            if ~param_signed_invariants
                opti.subject_to(U(1,:)>=0); % lower bounds on control
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
                e_position = c_obj{k} - c_obj_m{k}; % position error
                objective_fit = objective_fit + weight_accuracy*dot(e_position,e_position); % apply weighting to error
            end

            % Regularization constraints to deal with singularities and noise
            objective_reg = 0;
            for k=1:window_length-1
                %                 if k~=1
                %                     e_regul_deriv = U(:,k) - U(:,k-1); % first-order finite backwards derivative (noise smoothing effect)
                %                     %e_regul_vel = [R_FSt{k}*[U(1,k);0;0]-R_FSt{k-1}*[U(1,k-1);0;0]];
                %                 else
                %                     e_regul_deriv = 0;
                %                     %e_regul_vel = 0;
                %                 end
                if k~=1
                    e_regul_deriv = U(:,k) - U(:,k-1);
                else
                    e_regul_deriv = 0;
                end
                e_regul_abs = U([2 3],k); % absolute value invariants (force arbitrary invariants to zero)

                weight_regul_deriv = [scale_rotation*weight_regul_deriv_obj scale_rotation*weight_regul_deriv_mf scale_rotation*weight_regul_deriv_mf]';
                weight_regul_abs = [scale_rotation*weight_regul_abs_mf scale_rotation*weight_regul_abs_mf]';


                % apply weighting to errors, note: we also weigh with ||omega||, a small omega means close to singularity so increase effect regularization
                e_regul_deriv_weighted = weight_regul_deriv.^(1/2).*e_regul_deriv;
                e_regul_abs_weighted = weight_regul_abs.^(1/2).*e_regul_abs;
                %e_regul_vel_weighted = 0;%[ones(3,1)*weight_regul_deriv(1)].^(1/2).*e_regul_vel; % TODO improve

                objective_reg = objective_reg + dot(e_regul_deriv_weighted,e_regul_deriv_weighted) + dot(e_regul_abs_weighted,e_regul_abs_weighted);% + dot(e_regul_vel_weighted,e_regul_vel_weighted);
            end

            objective = objective_fit + objective_reg;


            %% Define solver
            opti.minimize(objective);
            opti.solver('ipopt',struct('print_time',1),struct('max_iter',max_iters,'tol',1e-7,'print_level',0));

            %% Save variables
            obj.X.R_FS = R_FS;
            obj.X.c_obj = c_obj;
            obj.U = U;
            obj.P.c_obj_m = c_obj_m;
            %obj.P.R_FS_0 = R_FS_0;
            %obj.L = L;
            %obj.parameterization = parameterization;
            obj.window_length = window_length;
            obj.P.h = h;
            obj.opti = opti;
            %obj.flag_first_time = 0;
            obj.param_signed_invariants = param_signed_invariants;
            obj.O.objective = objective;
            obj.O.objective_fit = objective_fit;
            obj.O.objective_reg = objective_reg;


        end

        function optimization_result = calculate_invariants(obj,meas_vec_traj,stepsize)

            N = obj.window_length;

            meas_twist = [meas_vec_traj zeros(N,3)];
            meas_twist = smoothdata(meas_twist,'gaussian',10); % ENSURE A SMOOTH INITIALISATION => AVOID INITIALISATIONS WITH FRAMEFLIPS


            %meas_vec_traj = meas_vec_traj.Obj_location;
            %
            % Estimate initial pose twist using a finite differences approach
            %twist_init = calculate_posetwist_from_discrete_poses(meas_traj,obj.h);
            %plot_descriptor(twist_init,[],'initial twists',h,parameterization,'pose_twist')

            if 1
                % Initialize invariants and moving frames over the whole horizon using discretized analytical formulas
                parameters.signed_invariants = obj.param_signed_invariants; % optional arguments
                meas_twist_smoothed = smoothdata(meas_twist,'gaussian',20); % ENSURE A SMOOTH INITIALISATION => AVOID INITIALISATIONS WITH FRAMEFLIPS

%                 meas_twist_smoothed = smoothdata(meas_twist,'gaussian',10); % ENSURE A SMOOTH INITIALISATION => AVOID INITIALISATIONS WITH FRAMEFLIPS
                ASA = calculate_ASA_pose(meas_twist_smoothed(1:round(N/2),:));
                mean_twist = sum(meas_twist_smoothed(1:round(N/2),1:3),1)*2/N;
                parameters.direction_vector_x = ASA(1:3,1);
                parameters.direction_vector_y =  cross(meas_twist_smoothed(1,1:3)',meas_twist_smoothed(round(N/2),1:3)');
                [FSt_init,invariants_init] = calculate_screw_invariants_from_discrete_twist(meas_twist_smoothed,stepsize,parameters);
                invariants_init = invariants_init+1e-12;
                %figure; plot(invariants_init(:,2))
                %1
            else
                start_ind = round(1,0);
                end_ind = round(N/2,0);
                figure; hold on; axis equal;
                mean_twist_vec = sum(meas_twist(:,1:3),1)/N;
                plot3(0,0,0,'r.');
                plot3(meas_twist(end_ind,1),meas_twist(end_ind,2),meas_twist(end_ind,3),'b.','MarkerSize',20);
                plot3(mean_twist_vec(1),mean_twist_vec(2),mean_twist_vec(3),'bx','MarkerSize',20);
                plot3(meas_twist(start_ind:end_ind,1),meas_twist(start_ind:end_ind,2),meas_twist(start_ind:end_ind,3),'b.');

                ASA = calculate_ASA_pose(meas_twist(start_ind:end_ind,:));

                mean_twist = ASA(1:3,1);
                mean_normal = ASA(1:3,2);
                plot3(mean_twist(1),mean_twist(2),mean_twist(3),'g.'); plot3(mean_normal(1),mean_normal(2),mean_normal(3),'k.');


                for i=1:N
                    FSt_init(:,:,i) = ASA;
                end
                invariants_init = zeros(N-1,6)+1e-6;
                %invariants_init(:,1) = 0;
            end

            %L_init = 1;


            %             for k=1:wdow_length
            %                 FSt_init(1:3,1:3,k) = eye(3);
            %             end
            %             invariants_init = 1e-3*ones(wdow_length,3);
            %
            % Initialize states + controls
            for k=1:N
                obj.opti.set_initial(obj.X.R_FS{k}, FSt_init(1:3,1:3)); %construct_init_FS_from_traj(meas_traj.Obj_location);
                obj.opti.set_initial(obj.X.c_obj{k}, meas_twist(k,1:3)); % initialized with measurement
            end

            % Initialize controls
            for k=1:N-1
                obj.opti.set_initial(obj.U(:,k), invariants_init(k,1:3)); % initialized with a guess
            end

            % Set values parameters
            %obj.opti.set_value(obj.P.R_FS_0, FSt_init(:,:,1)); % initialized with a guess
            for k=1:N
                obj.opti.set_value(obj.P.c_obj_m{k}, meas_twist(k,1:3));  % initialized with measurement
            end
            obj.opti.set_value(obj.P.h,stepsize);

            % Initialize extra decision variables
            %             if strcmp(obj.parameterization,'geometric')
            %                 obj.opti.set_initial(obj.L,L_init); % TODO take from input data, e.g. sum(|| p_k - p_k-1 ||)
            %             end

            % Initialize results structure
            optimization_result.FS_frames = zeros(3,3,N);
            optimization_result.Obj_location = zeros(N,3);
            optimization_result.invariants = zeros(N-1,3);
            %optimization_result.L = 0;

            % Solve window

            % Check value objective function in initial values
            %opti.value(opti.f,opti.initial())

            %disp(['window ' num2str(1) '-' num2str(window_length) '  (' num2str(N) ')'])
            solution = obj.opti.solve_limited();
            disp(['solved in ' num2str(solution.stats.iter_count) ' iterations and ' num2str(solution.stats.t_proc_total) ' seconds'])
            %sol.value(opti.g,opti.initial())


            % Fetch the different parts from the solution vector and store in a structure
            solution.value(obj.O.objective)
            solution.value(obj.O.objective_fit)
            solution.value(obj.O.objective_reg)


            optimization_result.FS_frames = reshape(solution.value([obj.X.R_FS{:}]),3,3,N);
            optimization_result.Obj_trajectory = solution.value([obj.X.c_obj{:}])';
            optimization_result.invariants = solution.value(obj.U)';
            %             if strcmp(obj.parameterization,'geometric') % c
            %                 %optimization_result.L = solution.value(obj.L);
            %             end
            optimization_result.invariants = [optimization_result.invariants ; optimization_result.invariants(end,:)];

            %obj.sol = solution;

        end

        function optimization_result = calculate_invariants_online(obj,meas_traj,cut_index)
            % n = index corresponding to first sample in window

            if ~obj.flag_first_time
                obj.flag_first_time = 1;
                optimization_result = obj.calculate_invariants(meas_traj);

                % Add extra starting constraints, necessary for preserving continuity of the solution over different windows
                obj.opti.subject_to(obj.X.p_obj{1} == obj.P.p_obj_m{1}); % start from given object position
                obj.opti.subject_to(obj.X.R_FSt{1} == obj.P.R_FS_0); % start from given FS frame
            else

                % Pass solution of second part of previous window to first part of next window (faster convergence)
                %opti.set_initial(sol.value_variables())

                N = obj.window_length;

                for k=1:N-cut_index
                    obj.opti.set_initial(obj.X.R_FSt{k}, obj.sol.value([obj.X.R_FSt{cut_index+k}]));
                    obj.opti.set_initial(obj.X.p_obj{k}, obj.sol.value([obj.X.p_obj{cut_index+k}]));
                end
                for k=1:N-cut_index-1
                    obj.opti.set_initial(obj.U(:,k),  obj.sol.value(obj.U(:,cut_index+k)));
                end
                %obj.opti.set_initial(obj.L,obj.sol.value(obj.L));

                % Initialize second part of next window
                twist_init = calculate_posetwist_from_discrete_poses(meas_traj,obj.h);
                parameters = struct();
                [FSt_init,~,invariants_init] = calculate_vector_invariants_from_discrete_twist(twist_init,obj.h,parameters);
                invariants_init = invariants_init(:,4:6) + 1e-10;
                for k=1:N
                    FSt_init(1:3,1:3,k) = eye(3);
                end
                %invariants_init = 1e-3*ones(N,3);

                measured_position = meas_traj.Obj_location;
                for k=N-cut_index+1:N
                    obj.opti.set_initial(obj.X.R_FSt{k}, FSt_init(:,:,k-1));
                    obj.opti.set_initial(obj.X.p_obj{k}, measured_position(k-1,:));
                end
                for k=N-cut_index:N-1 % TODO is this correct?
                    obj.opti.set_initial(obj.U(:,k), invariants_init(k-1,:));
                end

                % Set values of parameters that determine the starting constraints that guarantee continuity
                obj.opti.set_value(obj.P.R_FS_0, orthonormalize_rotation(obj.sol.value([obj.X.R_FSt{1+cut_index}])));
                obj.opti.set_value(obj.P.p_obj_m{1}, obj.sol.value([obj.X.p_obj{1+cut_index}]));

                % Set other parameters equal to the measurements in that window
                for k=2:N
                    obj.opti.set_value(obj.P.p_obj_m{k}, measured_position(k-1,:));
                end


                % Solve this window
                solution = obj.opti.solve();
                disp(['solved in ' num2str(solution.stats.iter_count) ' iterations and ' num2str(solution.stats.t_proc_total) ' seconds'])

                % Fetch the different parts from the solution vector and store in a structure
                optimization_result.FS_frames = reshape(solution.value([obj.X.R_FSt{:}]),3,3,N);
                optimization_result.Obj_location = solution.value([obj.X.p_obj{:}])';
                optimization_result.invariants = solution.value(obj.U)';
                %   optimization_result.invariants = [optimization_result.invariants ; optimization_result.invariants(end,:)];

                obj.sol = solution;

            end
        end
    end
end

