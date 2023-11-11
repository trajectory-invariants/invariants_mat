function geom_integr = cas_integrator_screw_invariants()

import casadi.*

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