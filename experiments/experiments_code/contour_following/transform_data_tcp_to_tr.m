function [T_tr_w,wrench_w_lc_tr_tr] = transform_data_tcp_to_tr(T_tcp_tr,pose_exp,wrench_exp)
%%
% This function provides trnaformation of pose and wrench from {tcp} to {tr}.
% This transformation is applied to pose and wrench to further challenge the invariance of the proposed invariant descriptors.

% Input
%     T_tcp_tr:             pose of {tcp} wrt {tr}
%     configuration:        configuration of the setup (explained in Fig. 4 and TABLE IV of the paper)
%     pose_exp:             measured pose of {tcp} wrt {w}
%     wrench_exp:           measured interaction wrench between tool and environment expressed in {tcp}
% Output
%     T_tr_w:               pose of {tr} wrt {tr}
%     wrench_w_lc_tr_tr:    measured interaction wrench between tool and environment expressed in {tr}

%%
N = size(pose_exp,1);

for j = 1 : N
    T_tcp_w(:,:,j) = [quat2rotm(pose_exp(j,4:7)),pose_exp(j,1:3)';0 0 0 1];
    T_tr_w(:,:,j) = T_tcp_w(:,:,j)*inverse_pose(T_tcp_tr);
    wrench_w_lc_tr_tr(j,:) = transform_screw(T_tcp_tr,wrench_exp(j,:)')';
end









