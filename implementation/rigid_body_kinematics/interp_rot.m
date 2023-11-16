function R_interpolated = interp_rot(x,R,xq)
% Interpolate the given rotation matrices R(x) to obtain R(xq).
%
% A constant rotational velocity is assumed between successive samples
% This function uses the same syntax as the built-in Matlab function "interp1"
%
% Inputs:  x  = array of N samples at which R is defined
%          R  = rotation matrices [3x3xN] at samples x
%          xq = array of M new samples for which R needs to be interpolated.
% Outputs: R_interpolated = rotation matrices [3x3xM] given at samples xq
%

if xq(1) < x(1) || xq(end) > x(end)
    error('Cannot interpolate beyond first or last sample!')
end

M = length(xq);
R_interpolated = zeros(3,3,M);

j=1; % index of original sample array x
for i=1:M % index of new sample array xq

    % Find which sample in x is directly after xq(i)
    while xq(i) > x(j+1)
        j=j+1;
    end

    x0 = x(j); % previous sample
    x1 = x(j+1); % next sample

    R0 = R(:,:,j); % previous rotation matrix
    R1 = R(:,:,j+1); % next rotation matrix

    if x1-x0 ~= 0
        % Three steps of linear interpolation:
        % (1) find total displacement [theta] between R0 and R1 using
        %     R1 = R0*deltaR  and [theta] = logm(deltaR)
        % (2) find fraction of displacement [thetanew] = (x-x0)/(x1-x0)*[theta]
        % (3) apply fraction of displacement
        %     Rnew = R0*expm([thetanew])
        R_new = R0 * expm( (xq(i)-x0)/(x1-x0) * logm(R0'*R1) );
    else
        R_new = R0;
    end
    R_interpolated(:,:,i) = R_new;
end