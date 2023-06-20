function [T_tcp_w,wrench_tcp_tcp] = transform_data_to_tcp(T_tr_w,wrench_lc_lc,T_tcp_tr,T_tcp_lc)
% This functions tranforms motion and wrench data to the TCP frame {tcp}
% Notation:
%     p_a_b = position of a wrt b
%     R_a_b = rotation of a wrt b
%     T_a_b = pose of a wrt b
%     w_d_c = wrench between objects, expressed in d, with ref point in c
% 
% Input:
%     T_tr_w:               pose of {tr} wrt {w}                [4x4xN]
%     wrench_lc_lc          wrench expressed in {lc}            [6xN]
%     T_tcp_tr              pose of {tcp} wrt {tr}              [4x4xN]
%     T_tcp_lc              pose of {tcp} wrt {lc}              [4x4xN]
% Output:
%     T_tcp_w               pose of {tcp} wrt {w}               [4x4xN]
%     wrench_tcp_tcp        wrench expressed in {tcp}           [6xN]

%% Calculation

N = size(T_tr_w,3);
T_lc_tcp = inverse_pose(T_tcp_lc);
T_tcp_w = zeros(4,4,N);
wrench_tcp_tcp = zeros(N,6);
for j = 1 : N
    T_tcp_w(:,:,j) = T_tr_w(:,:,j)*T_tcp_tr;
    wrench_tcp_tcp(j,:) = transform_screw(T_lc_tcp,wrench_lc_lc(j,:)')';
end










