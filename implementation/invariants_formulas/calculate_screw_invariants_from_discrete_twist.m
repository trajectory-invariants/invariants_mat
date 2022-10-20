function [T_isa,invariants] = calculate_screw_invariants_from_discrete_twist(twist,h,parameters)
% Calculate approximative invariants and ISA frames from given screw twist data using discretized analytical formulas
%
% Two steps:
% 1) Definition ISA frames by finding the common normal between successive ISAs
% 2) Higher-order invariants (omega2,v2,omega3,v3) are found as the displacement twist from one ISA frame to the next.
% For more information see technical report: https://gitlab.kuleuven.be/robotgenskill/brainstorm/-/blob/master/discretized%20invariants/difference_equations.pdf
%
% Input:  twist = screw twist (Nx6) [omegax omegay omegaz vx vy vz]
%         h     = timestep
%         (Optional) parameters = structure containing parameter values
%                                 parameters.threshold_x_axis: default 0.05 (range=[0,1]); if under x% of max(omega1), reject x-axis since close to singularity
%                                 parameters.threshold_y_axis: default 0.05 (range=[0,1]); if under x% of max(omega2), reject y-axis since close to singuarity
%                                 parameters.normal_vector_x: default ''; specify this 3-vector if you want sign of omega1 to be determined relative to this direction
%                                 parameters.signed_invariants: default 1; put to zero if you want omega1 and omega2 to always be positive (but this may result in more frame flips)
%
% Output: T_isa      = estimated ISA frames (4x4xN) [ e_x e_y e_z | p_isa ]
%         invariants = estimated invariants (Nx6)

N = size(twist,1);
invariants = zeros(N,6);
T_isa = zeros(4,4,N);

% Set values of parameters, if variable parameters is empty the default value is taken
threshold_norotation = initialize_parameter(parameters, 'threshold_x_axis', 0.10);
threshold_parallel = initialize_parameter(parameters, 'threshold_y_axis', 0.10);
bool_signed_invariants = initialize_parameter(parameters, 'signed_invariants', 1);
reference_direction_vector_x = initialize_parameter(parameters, 'direction_vector_x', '');
reference_direction_vector_y = initialize_parameter(parameters, 'direction_vector_y', '');


% Calculating some intermediate results first
Omega = twist(:,1:3); % omega
V = twist(:,4:6); % v
normOmega = sqrt(sum(Omega.^2,2)); % ||omega||
Omegak_X_Omegakplus1 = cross(Omega(1:end-1,:),Omega(2:end,:),2);
normOmegak_X_Omegakplus1 = sqrt(sum(Omegak_X_Omegakplus1.^2,2));
Omegak_X_Vk = cross(Omega,V,2); % Omega x V
p_perp = Omegak_X_Vk ./ repmat(normOmega.^2,1,3); % p_perpendicular: position vector from the origin of the reference frame to the point lying on the screw axis that is closest to the origin
norm_p_perp_diff = sqrt(sum(diff(p_perp).^2,2)); % distance between successive p_perpendicular

% Define thresholds for singularities relative to min/max values
limit_norotation = (max(normOmega)-min(normOmega)) * threshold_norotation + min(normOmega);
limit_parallel = 1e-12 + (max(normOmegak_X_Omegakplus1)-min(normOmegak_X_Omegakplus1)) * threshold_parallel + min(normOmegak_X_Omegakplus1);
limit_parallel_intersection = (max(norm_p_perp_diff)-min(norm_p_perp_diff)) * threshold_parallel + min(norm_p_perp_diff);

% (Optional) visualization thresholds
% figure; hold on;
% plot(normOmega);
% plot(ones(size(normOmega))*limit_norotation);
% figure; hold on;
% plot(normOmegak_X_Omegakplus1);
% plot(ones(size(normOmegak_X_Omegakplus1))*limit_parallel);

first_good_index_sing1 = N;
first_good_index_sing2 = N;

%% Step 1: Definition ISA frame at sample k
for k=1:N-1
    % Load the twist components of sample k and k+1
    omega_k = Omega(k,:);
    omega_kplus1 = Omega(k+1,:);
    %v_k = V(k,:);
    normOmega_k = normOmega(k);

    %% First axis
    % Check first singularity: ||omega|| = 0 (no rotation)
    if normOmega_k < limit_norotation && k > 1
        % Case 1: no rotation
        ex = T_isa(1:3,1,k-1)';
    elseif normOmega_k < limit_norotation || normOmega_k == 0
        % Case 2: no rotation and first sample
        ex = [1 0 0];
    else
        % Case 3: normal case
        ex = omega_k./normOmega_k; % Omega/norm(Omega) [3x1 vector]
        first_good_index_sing1 = min(first_good_index_sing1,k);
    end

    %% Second axis
    % Load p perpendicular, point on ISA closest to origin reference frame
    p_perp_k = p_perp(k,:);
    p_perp_kplus1 = p_perp(k+1,:);

    % Check first and second singularity: ||omega x omegadot|| = 0 (parallel axes)
    if normOmega_k < limit_norotation && k > 1
        % Case 1: no rotation
        common_normal = T_isa(1:3,2,k-1)'; % take from previous sample
        p1 = T_isa(1:3,4,k-1)';
    elseif normOmegak_X_Omegakplus1(k) < limit_parallel && k > 1
        % Case 2: parallel axes
        common_normal = T_isa(1:3,2,k-1)'; % take from previous sample
        p1 = T_isa(1:3,4,k-1)';
        %     elseif normOmegak_X_Omegakplus1(k) < limit_parallel && norm(p_perp_kplus1 - p_perp_k) < limit_parallel_intersection && k > 1
        %         % Case 2: parallel, intersecting axes
        %         common_normal = T_isa(1:3,2,k-1)'; % take from previous sample
        %         p1 = T_isa(1:3,4,k-1)';
        %     elseif normOmegak_X_Omegakplus1(k) < limit_parallel && k > 1
        %         % Case 3: parallel, non-intersecting axes
        %         common_normal = p_perp_kplus1 - p_perp_k;
        %         p1 = p_perp_k; %+ dot((T_isa(:,:,k-1) - p_perp_k),ex)*ex;
    elseif normOmegak_X_Omegakplus1(k) < limit_parallel || normOmegak_X_Omegakplus1(k) == 0
        % Case 4: parallel axes + first sample
        common_normal = [0 1 0];
        p1 = [0 0 0];
    else
        % Case 5: normal case
        [p1,p2] = closest_points_line1_to_line2(omega_k,p_perp_k,omega_kplus1,p_perp_kplus1);

        %common_normal = p2 - p1;
        common_normal = cross(omega_k,omega_kplus1); % alternative

        first_good_index_sing2 = min(first_good_index_sing2,k);
    end

    % Derive Y axis from common normal
    ey = common_normal/norm(common_normal);

    % Ensure Y axis is orthogonal to X axis (Gram-Schmidt process)
    ey = ey - dot(ey,ex)*ex;
    ey = ey/norm(ey);

    if bool_signed_invariants && ~isempty(reference_direction_vector_x) && dot(ex,reference_direction_vector_x) < 0
        ex = -ex;
    end

    if bool_signed_invariants && ~isempty(reference_direction_vector_y) && dot(ey,reference_direction_vector_y) < 0
        ey = -ey;
    end

    % Flip axes or not, this is to avoid unnecessary frame flips (peaks in the invariants)
    %   if bool_signed_invariants && k > max(first_good_index_sing1,first_good_index_sing2)
    % reference direction = some external reference axis
    %         if ~isempty(reference_direction_vector_x) && dot(ex,reference_direction_vector_x) < 0
    %             ex = -ex;
    % reference direction = previous axis
    %         elseif k > 1 && dot(ex,T_isa(1:3,1,k-1)) < 0
    %             ex = -ex;
    %         end
    % reference direction = previous axis
%     if bool_signed_invariants && k > 1 && dot(ey,T_isa(1:3,2,k-1)) < 0
%         ey = -ey;
%     end
    %    end

    % Define ISA frame
    T_isa(:,:,k) = [ex' ey' cross(ex',ey') p1' ; 0 0 0 1];
end
T_isa(:,:,N) = T_isa(:,:,N-1);

% Correct bad indices
first_good_index = max(first_good_index_sing1,first_good_index_sing2);
while first_good_index-1 ~= 0
    T_isa(:,:,first_good_index-1) = T_isa(:,:,first_good_index);
    first_good_index = first_good_index - 1;
end



%% Step 2: extraction of invariants
for k=1:N-1

    % Set values of first invariants, using dot products to get the signed value
    omega_k = Omega(k,:);
    v_k = V(k,:);
    ex = T_isa(1:3,1,k)';
    invariants(k,1) = dot(omega_k,ex); % omega1
    invariants(k,4) = dot(v_k,ex); % v1

    % Screw displacement between successive ISAs
    %DeltaT = logm(T_isa(:,:,k)^-1*T_isa(:,:,k+1));
    DeltaT = logm_pose(inverse_pose(T_isa(:,:,k))*T_isa(:,:,k+1));

    %disp(DeltaT);

    omega_2 = DeltaT(1,3)/h;
    omega_3 = -DeltaT(2,3)/h;
    v_2 = DeltaT(2,4)/h;
    v_3 = DeltaT(1,4)/h;

    %     omega_z = -DeltaT(1,2)/h;
    %     v_z = DeltaT(3,4)/h;
    %     disp(['good: ' num2str(omega_2)])
    %     disp(['bad: ' num2str(omega_z)])

    % When invariants are exactly zero, the optimization returns an error
    % Therefore, in that case, we give them a small value
    %     if abs(omega_2) > 1e-10
    invariants(k,2) = omega_2;
    invariants(k,3) = omega_3;
    invariants(k,5) = v_2;
    invariants(k,6) = v_3;
    %     else
    %         invariants(k,[2 3 5 6]) = 1e-10*[1 1 1 1];
    %     end
end
invariants(N-1,:) = invariants(N-2,:);
invariants(N,:) = invariants(N-1,:);
end

function [D,E] = closest_points_line1_to_line2(a,A,b,B)
% Find point on line 1 that lies closest to line 2
% formula taken from http://morroworks.palitri.com/Content/Docs/Rays%20closest%20point.pdf
%
% Input: a = direction vector of line1
%        b = direction vector of line2
%        A = arbitrary point lying on line1
%        B = arbitrary point lying on line2
% Output: D = closest point lying on line1 to line2
%         E = closest point lying on line2 to line1
C = B-A;
D = A + a*(-dot(a,b)*dot(b,C)+dot(a,C)*dot(b,b)) / (dot(a,a)*dot(b,b)-dot(a,b)*dot(a,b));
E = B + b*(dot(a,b)*dot(a,C)-dot(b,C)*dot(a,a)) / (dot(a,a)*dot(b,b)-dot(a,b)*dot(a,b));
end

function parameter_value = initialize_parameter(structure, field_name, default_value)
% Return parameter value from structure corresponding to field_name. If it doesn't exist, return default value.
%
% Helper function that deals with initialization of parameters.
% Why? Because I want the function to work if the user didn't fill out the entire structure.
%
% Input: structure
%        field_name
%        default_value
% Output: parameter_value
%
if ~isfield(structure,field_name) % doesn't exist, return default value
    parameter_value = default_value;
else
    parameter_value = structure.(field_name);
end
end