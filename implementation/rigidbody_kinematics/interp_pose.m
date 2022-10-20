function T_interpolated = interp_pose(x,T,xq)
% Linearly interpolate the given pose matrices T(x) to obtain T(xq)
% A constant twist is assumed between successive samples of x
% This function uses the same syntax as the built-in Matlab function "interp1"
%
% Inputs:  x  = array of N samples for which T is defined
%          T  = pose matrices [4x4xN] given at samples x
%          xq = array of M new samples for which T needs to be interpolated.
% Outputs: T_interpolated = rotation matrices [4x4xM] given at samples xq
%

if xq(1) < x(1) || xq(end) > x(end)
    error('Cannot interpolate beyond first or last sample!')
end

M = length(xq);
T_interpolated = zeros(4,4,M);

j=1; % index of original sample array x
for i=1:M % index of new sample array xq

    % Find which sample in x is directly after xq(i)
    while xq(i) > x(j+1)
        j=j+1;
    end

    x0 = x(j); % previous sample
    x1 = x(j+1); % next sample

    T0 = T(:,:,j); % previous rotation matrix
    T1 = T(:,:,j+1); % next rotation matrix

    if x1-x0 ~= 0
        % Three steps of linear interpolation:
        % (1) find total displacement [dtheta dx] between T0 and T1 using
        %     T1 = T0*deltaT  and [dtheta dx] = logm(deltaT)
        % (2) find fraction of displacement [dtheta dx]_new = (x-x0)/(x1-x0)*[dtheta dx]
        % (3) apply fraction of displacement
        %     Tnew = T0*expm([dtheta dx]_new)
        T_new = T0 * expm( (xq(i)-x0)/(x1-x0) * logm(inverse_pose(T0)*T1) );
    else
        T_new = T0;
    end
    T_interpolated(:,:,i) = T_new;
end