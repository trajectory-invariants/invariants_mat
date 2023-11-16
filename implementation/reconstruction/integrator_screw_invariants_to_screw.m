function [T_isa_plus1,invariants_obj_world] = integrator_screw_invariants_to_screw(T_isa,invariants,h)
% Integrate the given screw axis invariants (assuming they are constant over the integration window)
% This is a geometric integrator that preserves the properties of the SE(3) group
%
% Input:   T_isa      = pose of ISA frame (3x4)
%          invariants = screw axis invariants (Nx6) [omega|omega_kappa|omega_tau|v|v_b|v_t]
%          h          = integration step
% Output:  T_isa_plus1 = updated pose of ISA frame (3x4)

% Construct screw twist of ISA expressed in ISA frame
invariants_isa = [invariants(3);0;invariants(2);invariants(6);0;invariants(5)];

% Update pose of ISA frame
delta_T_isa = expm_pose(invariants_isa,h);
T_isa_plus1 = [T_isa;0 0 0 1]*delta_T_isa; % right multiplication
T_isa_plus1 = T_isa_plus1(1:3,:);

% Construct screw twist of object expressed in ISA frame
invariants_obj = [invariants(1);0;0;invariants(4);0;0];

% Transform screw twist of object from ISA to world
invariants_obj_world = transform_screw(T_isa,invariants_obj);