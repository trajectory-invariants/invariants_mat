function [R_FSt_plus1,p_obj_plus1] = integrator_vector_invariants_to_pos(R_FSt,p_obj,invariants,h)
% Integrate the given Frenet-Serret invariants (assuming they are constant over the integration window)
% This is a geometric integrator that preserves the properties of the SO(3) group
%
% Input:   R_FSt_plus1  = current rotation matrix of FS frame (3x3)
%          p_obj        = currentposition vector of object (3x1)
%          invariants   = frenet-serret invariants (Nx3) [it1|it2|it3]
%          h            = integration step
% Output:  R_FSt_plus1  = updated rotation matrix of  FS frame (3x3)
%          p_obj_plus1  = updated position vector of object (3x1)

% Construct rotational velocity of translational Frenet-Serret frames expressed in their respective frame
invariants_FSt = [invariants(3);0;invariants(2)];

% Construct translational velocity of object expressed in the corresponding Frenet-Serret frame
invariants_trans = [invariants(1);0;0];

% We choose to combine translation in one
invariants_trans_screw = [invariants_FSt ; invariants_trans];

% Update rotation of translational Frenet-Serret frame
delta_T_translation = expm_pose(invariants_trans_screw,h);
T_translation = [R_FSt p_obj ; 0 0 0 1]*delta_T_translation; % right multiplication
R_FSt_plus1 = T_translation(1:3,1:3);
p_obj_plus1 = T_translation(1:3,4);

end

% % Continous state dynamics equations of the form: dx/dt = f(x,u,t)
% dR_FS = R_FS*skew([i6;i5;0]);
% dR_eul = R_eul*skew([i3;i2;0]);
% dR_obj = skew(R_eul*[i1;0;0])*R_obj;
% dp_obj = R_FS*[i4;0;0];
% rhs = [dR_FS(:) ; dR_eul(:) ; dR_obj(:) ; dp_obj];
%
% % Define ordinary differential equations (Function(name,x,u,xdot))
% ode_simp = Function('ode_simp',{x,u},{rhs});