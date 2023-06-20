function [s_normalized,s_n] = calculate_normalized_anglelength(R,N_des)
% Calculate normalized angle length vector from the rotation trajectory R

N = size(R,3);
omega = zeros(N-1,3);
for j = 1 : N-1
    DeltaR = logm(R(:,:,j+1)*R(:,:,j)');
    omega(j,1:3) = [ -DeltaR(2,3) DeltaR(1,3) -DeltaR(1,2) ];
end
omeganorm = sqrt(sum(omega.^2,2));
s = [ 0 ; cumsum(omeganorm)];

s_normalized = s./s(end);

s_n = linspace(0,s_normalized(end),N_des);