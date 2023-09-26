function [R_FS_plus1] = integrator_vector_invariants_to_vector(R_FS,invariants,h)
% Integrate the given Frenet-Serret invariants (assuming they are constant over the integration window)
% This is a geometric integrator that preserves the properties of the SE(3) group
%
% Input:   R_FS         = current rotation matrix of the FS frame (3x3)
%          invariants   = frenet-serret invariants (Nx3) [ir1|ir2|ir3]
%          h            = integration step
% Output:  R_FS_plus1   = updated rotation matrix of the FS frame (3x3)


i1 = invariants(1);
i2 = invariants(2);
i3 = invariants(3);

omegat = [i3;0;i2];
omegat_norm = norm_2(omegat);

% next FS frame
deltaR = eye(3)+sin(omegat_norm*h)/omegat_norm*skew(omegat)+(1-cos(omegat_norm*h))/omegat_norm^2 * skew(omegat)*skew(omegat);
R_FS_plus1 = R_FS*deltaR;


% %rotation
% deltaR_o = eye(3) + sin(omegao_norm*h)/omegao_norm*skew(omegao) + (1-cos(omegao_norm*h))/omegao_norm^2 * skew(omegao)*skew(omegao);
% deltap_o = v_o*h;
% 
% p_center = [sum(pose_twist(1,:))/(M/3);sum(pose_twist(2,:))/(M/3);sum(pose_twist(3,:))/(M/3)];
% %p_markers_plus1 = repmat(p_center,1,M/3) + [deltaR_o deltap_o] * [(p_markers-repmat(p_center,1,M/3)) ; ones(1,M/3)]; % pdot = omega x p + v
% p_markers_plus1 = pose_twist + h*[skew(omegao) v_o] * [(pose_twist-repmat(p_center,1,M/3)) ; ones(1,M/3)]; % pdot = omega x p + v
% 

%integr2 = Function('phi', {x,[u;L;Theta]} , {out_plus1});

% % System parameters (for measurements)
% twist_obj_m = SX.sym('twist_obj_m',6,1);
% p = [twist_obj_m(:)];
% np = length(p);
% weight_x0 = SX.sym('weight_x0');
% 
% % State dynamics equations of the form: dx/dt = f(x,u,t)
% dR_FS = R_FS*skew([i6;i5;0]);
% dR_eul = R_eul*skew([i3;i2;0]);
% 
% % twist_ref = SX.sym('twist_ref',6,1);
% % twist_ref(1:3) = R_eul*[i1;0;0];
% % twist_ref(4:6) = R_FS*[i4;0;0];
% % twist_skew = skew_T(twist_ref);
% % 
% % p_center = [sum(pose_twist(1,:))/(M/3);sum(pose_twist(2,:))/(M/3);sum(pose_twist(3,:))/(M/3)];
% % dp_markers = twist_skew(1:3,:) * [(pose_twist-repmat(p_center,1,M/3)) ; ones(1,M/3)]; % pdot = omega x p + v
% 
% rhs = [dR_FS(:) ; dR_eul(:)];% ; dp_markers(:)];
% 
% % Define as a set of ordinary differential equations
% ode_simp = Function('ode_simp',{x,u},{rhs});
% 
% % Define a simple Runge-Kutta integrator for integrating the dynamics
% % the second argument is the number of auxiliary integration steps (useful especially for e.g. highly nonlinear systems)
% integr = simpleRK(ode_simp,1);
