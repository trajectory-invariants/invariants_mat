function deltaT = expm_pose(twist,deltat)
% Find discrete change in pose SE(3) after applying a given twist se(3) in
% the given time deltat
%
% Implemented here using rodrigues rotation formula (see Murray 1994).
% This is a closed-form expression for the matrix exponential operator
% applied on the twist corresponding to "expm( skew_T(twist)*dt )"
%
% Input: twist      = input twist (6x1)
%        deltat     = timestep
% Output: deltaT    = change in pose (4x4)

omega = twist(1:3);
omega_norm = norm(omega);
v = twist(4:6);

%if_else(omega_norm=0)

% Rodrigues rotation formula: deltaR = I + sin(theta)/||omega|| * omega^ + (1-cos(theta))/||omega||^2 * omega*omega'
deltaR = eye(3)+sin(omega_norm*deltat)/omega_norm*skew(omega)+(1-cos(omega_norm*deltat))/omega_norm^2 * skew(omega)*skew(omega);

% Rodrigues rotation formula for translation: deltap = (I - deltaR) * omega^ * v/||omega||^2 + omega*omega' * v/||omega||^2 * dt
deltap = (eye(3)-deltaR)*skew(omega)*v/omega_norm^2 + omega*omega'*v/omega_norm^2*deltat;

deltaT = [deltaR deltap; 0 0 0 1];


