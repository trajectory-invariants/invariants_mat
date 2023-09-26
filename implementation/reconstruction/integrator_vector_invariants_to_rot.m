function [R_FSr_plus1,R_obj_plus1] = integrator_vector_invariants_to_rot(R_FSr,R_obj,invariants,h)
% Integrate the given Frenet-Serret invariants (assuming they are constant over the integration window)
% This is a geometric integrator that preserves the properties of the SO(3) group
%
% Input:   R_FSr        = current rotation matrix of rotational FS frame (3x3)
%          R_obj        = current rotation matrix of object (3x3)
%          invariants   = frenet-serret invariants (Nx3) [ir1|ir2|ir3]
%          h            = integration step
% Output:  R_FSr_plus1  = updated rotation matrix of rotational FS frame (3x3)
%          R_obj_plus1  = updated rotation matrix of object (3x3)

% Construct rotational velocity of rotational and translational Frenet-Serret frames expressed in their respective frame
invariants_FSr = [invariants(3);0;invariants(2)];

% Construct rotational and translational velocity of object expressed in the corresponding Frenet-Serret frame
invariants_rot = [invariants(1);0;0;];

% Transform screw twist of object from ISA to world
invariants_rot_world = R_FSr*invariants_rot;

% Update orientation of rotational Frenet-Serret frame
% delta_R_FSr = exp_rot(invariants_FSr,h); % expm(skew(invariants_FSr)*h)
delta_R_FSr = expm_rot(invariants_FSr,h); % expm(skew(invariants_FSr)*h)
R_FSr_plus1 = R_FSr*delta_R_FSr; % right multiplication

% Update orientation of object frame
delta_R_obj = expm_rot(invariants_rot_world,h); % expm(skew(invariants_rot_world)*h)
R_obj_plus1 = delta_R_obj*R_obj; % left multiplication

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