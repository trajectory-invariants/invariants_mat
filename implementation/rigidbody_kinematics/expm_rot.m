function deltaR = expm_rot(omega,deltat)
% Find change in rotation matrix SO(3) after applying a given rotational
% velocity omega so(3) in the given time deltat
%
% Implemented here using rodrigues rotation formula (see Murray 1994).
% This is a closed-form expression for the matrix exponential operator
% applied on the rotational velocity: expm( skew(omega)*dt );
%
% Input: omega      = input rotational velocity (3x1)
%        deltat     = timestep
% Output: deltaR    = change in rotation matrix (3x3)

omega_norm = norm(omega);
e_omega = omega/omega_norm;

% Rodrigues rotation formula: deltaR = I + sin(theta)/||omega|| * omega^ + (1-cos(theta))/||omega||^2 * omega*omega'
deltaR = eye(3)+sin(omega_norm*deltat)*skew(e_omega)+(1-cos(omega_norm*deltat))*skew(e_omega)*skew(e_omega);


