function [R_FS,R_eul,invariants] = calculate_vector_invariants_from_discrete_twist(twist,h,parameters)
% Calculate invariants and FS frames from given posetwist data using discretized analytical formulas
%
% First axes e_tx/e_rx + ir1,it1 follow from velocity vectors, second and third axes follow from change in direction of velocity vector.
% Afterwards the higher-order invariants ir2,ir3 and it2,it3 are found as the displacement twist from one FS frame to the next.
%
% Input:  twist = pose twist (Nx6) [omega|v]
%         h     = timestep
%         parameters = structure containing parameter values
%                 parameters.threshold_trans_x_axis: default 0.05 (range=[0,1]); if under x% of max(it1), reject trans x-axis since close to singularity
%                 parameters.threshold_trans_y_axis: default 0.05 (range=[0,1]); if under x% of max(it2), reject trans y-axis since close to singuarity
%                 parameters.threshold_rot_x_axis: default 0.05 (range=[0,1]); if under x% of max(ir1), reject rot x-axis since close to singularity
%                 parameters.threshold_rot_y_axis: default 0.05 (range=[0,1]); if under x% of max(it2), reject rot y-axis since close to singuarity
%                 parameters.signed_invariants: default 1; put to zero if you want omega1 and omega2 to always be positive (but this may result in more frame flips)
%                 parameters.maindirection_trans_x_axis: default ''; specify this 3-vector if you want sign of it1 to be determined relative to this direction
%                 parameters.maindirection_trans_y_axis: default ''; specify this 3-vector if you want sign of it2 to be determined relative to this direction
%                 parameters.maindirection_rot_x_axis: default ''; specify this 3-vector if you want sign of ir1 to be determined relative to this direction
%                 parameters.maindirection_rot_y_axis: default ''; specify this 3-vector if you want sign of ir2 to be determined relative to this direction
%
% Output: T_isa      = estimated ISA frames (4x4xN) [ e_x e_y e_z | p_isa ]
%         invariants = estimated invariants (Nx6)

N = size(twist,1);
invariants = zeros(N,6);
R_FS = zeros(3,3,N);
R_eul = zeros(3,3,N);

% Deal with setting parameter values
threshold_trans_x_axis = initialize_parameter(parameters, 'threshold_trans_x_axis', 0.15);
threshold_trans_y_axis = initialize_parameter(parameters, 'threshold_trans_y_axis', 0.15);
threshold_rot_x_axis = initialize_parameter(parameters, 'threshold_rot_x_axis', 0.15);
threshold_rot_y_axis = initialize_parameter(parameters, 'threshold_rot_y_axis', 0.15);
signed_invariants = initialize_parameter(parameters, 'signed_invariants', 1);
maindirection_trans_x_axis = initialize_parameter(parameters, 'maindirection_trans_x_axis', '');
maindirection_trans_y_axis = initialize_parameter(parameters, 'maindirection_trans_y_axis', '');
maindirection_rot_x_axis = initialize_parameter(parameters, 'maindirection_rot_x_axis', '');
maindirection_rot_y_axis = initialize_parameter(parameters, 'maindirection_rot_y_axis', '');

% Calculating some intermediate results
Omega=twist(:,1:3);
V=twist(:,4:6);
normOmega = sqrt(sum(Omega.^2,2));
normV = sqrt(sum(V.^2,2));
Omegak_X_Omegakplus1 = cross(Omega(1:end-1,:),Omega(2:end,:),2);
normOmegak_X_Omegakplus1 = sqrt(sum(Omegak_X_Omegakplus1.^2,2));
Vk_X_Vkplus1 = cross(V(1:end-1,:),V(2:end,:),2);
normVk_X_Vkplus1 = sqrt(sum(Vk_X_Vkplus1.^2,2));

limit_ir1 = (max(normOmega)-min(normOmega))*threshold_rot_x_axis+min(normOmega);
limit_ir2 = (max(normOmegak_X_Omegakplus1)-min(normOmegak_X_Omegakplus1))*threshold_rot_y_axis+min(normOmegak_X_Omegakplus1);
limit_it1 = (max(normV)-min(normV))*threshold_trans_x_axis+min(normV);
limit_it2 = (max(normVk_X_Vkplus1)-min(normVk_X_Vkplus1))*threshold_trans_y_axis+min(normVk_X_Vkplus1);

% Visualization limits
% figure; hold on;
% plot(normOmega);
% plot(ones(size(normOmega))*limit_ir1);
% figure; hold on;
% plot(normOmegak_X_Omegakplus1);
% plot(ones(size(normOmegak_X_Omegakplus1))*limit_ir2);
% figure; hold on;
% plot(normV);
% plot(ones(size(normV))*limit_it1);
% figure; hold on;
% plot(normVk_X_Vkplus1);
% plot(ones(size(normVk_X_Vkplus1))*limit_it2);

% Determine FSt_k and FSr_k
for k=1:N-1
    
    % Load twist
    omega_k = Omega(k,:);
    omega_kplus1 = Omega(k+1,:);
    normOmega_k = normOmega(k);
    v_k = V(k,:);
    v_kplus1 = V(k+1,:);
    normV_k = normV(k);
    
    %% Calculate x-axis
    % Reject new x-axis when omega is small (singularity)
    if signed_invariants && (k > 1 && normOmega_k <= limit_ir1)
        ex_r = R_eul(:,1,k-1)'; % take previous x-axis
    elseif normOmega_k < limit_ir1
        ex_r = [1 0 0];
    else
        ex_r = omega_k./(normOmega_k); % Omega/norm(Omega) [vector]
    end
    if signed_invariants && (k > 1 && normV_k <= limit_it1)
        ex_t = R_FS(:,1,k-1)'; % take previous x-axis
    elseif normV_k <= limit_it1
        ex_t = [1 0 0];
    else
        ex_t = v_k./(normV_k); % V/norm(V) [vector]
    end
    
    % Flip x-axis or not, this is to avoid unnecessary frame flips (peaks in the invariants)
    if signed_invariants % invariants allowed to be negative
        if ~isempty(maindirection_trans_x_axis) && dot(ex_t,maindirection_trans_x_axis) < 0 % reference direction = some external reference axis
            ex_t = -ex_t;
        elseif k > 1 && (dot(ex_t,R_FS(:,1,k-1)) < 0) % reference direction = previous axis
            ex_t = -ex_t;
        end
        if ~isempty(maindirection_rot_x_axis) && dot(ex_r,maindirection_rot_x_axis) < 0 % reference direction = some external reference axis
            ex_r = -ex_r;
        elseif k > 1 && (dot(ex_r,R_eul(:,1,k-1)) < 0) % reference direction = previous axis
            ex_r = -ex_r;
        end
    end
    
    %% Calculate y-axis
    v_x_vplus1 = cross(v_k,v_kplus1);
    norm_v_x_vplus1 = norm(v_x_vplus1);
    
    omega_x_omegaplus1 = cross(omega_k,omega_kplus1);
    norm_omega_x_omegaplus1 = norm(omega_x_omegaplus1);
    
    % Reject new y-axis when i2 is small (singularity)
    if signed_invariants && (k > 1 && norm_v_x_vplus1 <= limit_it2)
        ey_t = R_FS(:,2,k-1)'; % take previous y-axis
    elseif norm_v_x_vplus1 <= limit_it2
        ey_t = [0 0 1];
    else
        ey_t = v_x_vplus1/norm_v_x_vplus1;
    end
    if signed_invariants && (k > 1 && norm_omega_x_omegaplus1 <= limit_ir2)
        ey_r = R_eul(:,2,k-1)'; % take previous y-axis
    elseif norm_omega_x_omegaplus1 <= limit_ir2
        ey_r = [0 0 1];
    else
        ey_r = omega_x_omegaplus1/norm_omega_x_omegaplus1;
    end
    
    % Make orthogonal (Gram-Schmidt)
    ey_t = ey_t - dot(ey_t,ex_t)*ex_t;
    ey_t = ey_t/(norm(ey_t));
    ey_r = ey_r - dot(ey_r,ex_r)*ex_r;
    ey_r = ey_r/(norm(ey_r));
    
    % Flip y-axis or not, this is to avoid unnecessary frame flips (peaks in the invariants)
    if signed_invariants % invariants allowed to be negative
        if ~isempty(maindirection_trans_y_axis) && dot(ey_t,maindirection_trans_y_axis) < 0 % reference direction = some external reference axis
            ey_t = -ey_t;
        elseif k > 1 && (dot(ey_t,R_FS(:,2,k-1)) < 0) % reference direction = previous axis
            ey_t = -ey_t;
        end
        if ~isempty(maindirection_rot_y_axis) && dot(ey_r,maindirection_rot_y_axis) < 0 % reference direction = some external reference axis
            ey_r = -ey_r;
        elseif k > 1 && (dot(ey_r,R_eul(:,2,k-1)) < 0) % reference direction = previous axis
            ey_r = -ey_r;
        end
    end
    
    %% Determine first invariants + set moving frame
    
    % Set values of first invariants, use dot products to get the signed value
    invariants(k,1) = dot(omega_k,ex_r); % ir1
    invariants(k,4) = dot(v_k,ex_t); % it1
    
    % Make rotation matrices of the unit vectors
    R_FS(:,:,k) = [ex_t' ey_t' cross(ex_t',ey_t')];
    R_eul(:,:,k) = [ex_r' ey_r' cross(ex_r',ey_r')];
end
R_FS(:,:,N) = R_FS(:,:,N-1);
R_eul(:,:,N) = R_eul(:,:,N-1);

% Between successive moving frames, determine the rotational velocity vector
% The second and third invariants correspond with elements of this rotational velocity vector
for k=1:N-1
    % Rotation | very messy, make dedicated code for translation only
    if any(any(isnan(R_eul(:,:,k))))
        DeltaRr = zeros(3,3);
        R_eul(:,:,k) = eye(3);
        if k == N-1
            R_eul(:,:,k+1) = eye(3);
        end
    else
        DeltaRr = logm(R_eul(:,:,k)'*R_eul(:,:,k+1));
    end
    i_r2 = DeltaRr(1,3)/h;
    i_r3 = -DeltaRr(2,3)/h;
    
    % Translation
    DeltaRt = logm(R_FS(:,:,k)'*R_FS(:,:,k+1));
    
    i_t2 = DeltaRt(1,3)/h;
    i_t3 = -DeltaRt(2,3)/h;
    
    invariants(k,2) = i_r2;
    invariants(k,3) = i_r3;
    invariants(k,5) = i_t2;
    invariants(k,6) = i_t3;
end
invariants(N,:) = invariants(N-1,:);

% When invariants are exactly zero, optimization returns error (undefined
% gradients), so therefore we add a small value
%invariants = invariants + 1e-10;

end

function parameter_value = initialize_parameter(structure, field_name, default_value)
% Helper function that deals with initializing parameters
if ~isfield(structure,field_name)
    parameter_value = default_value;
else
    parameter_value = structure.(field_name);
end
end