function [s_normalized,s_n] = calculate_normalized_arclength(p,N_des)
% Calculate normalized arc length vector from the position trajectory p

vnorm = sqrt(sum(diff(p).^2,2));
s = [ 0 ; cumsum(abs(vnorm))];

s_normalized = s./s(end);

s_n = linspace(0,s_normalized(end),N_des);