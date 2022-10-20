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
    T_tcp_w(:,:,j)  = [quat2rotm(pose_exp(j,4:7)),pose_exp(j,1:3)';0 0 0 1];
    T_tr_w(:,:,j)   = T_tcp_w(:,:,j)*inverse_pose(T_tcp_tr);
    wrench_w_lc_tr_tr(j,:) = S_transform(T_tcp_tr,wrench_exp(j,:));
end









% function [T_w_tr,wrench_w_lc_tr_tr] = transform_data_tcp_to_tr(exp_type,pose_exp,wrench_exp)
% N = length(pose_exp);
% 
% if strcmp(exp_type,'1')
%     for j = 1 : N
%         T_w_tcp(:,:,j)  = [quat2rotm(pose_exp(j,4:7)),pose_exp(j,1:3)';0 0 0 1];
%         % Rotation tcp frame w.r.t tracker frame
%         R_tr_tcp        = rotx(135);
%         % Position tracker frame -> tcp frame expressed in the tracker frame
% %         p_tr_tcp        = [0,-57.02-42.67,55.59]./1000;
%         p_tr_tcp        = [0,-57.02-42.67,35.5]./1000;
%         T_tr_tcp        = [R_tr_tcp,p_tr_tcp';0 0 0 1];
%         T_w_tr(:,:,j)   = T_w_tcp(:,:,j)*inverse_pose(T_tr_tcp);
% 
%         wrench_w_lc_tr_tr(j,:) = S_transform(T_tr_tcp,wrench_exp(j,:));
%     end
% elseif strcmp(exp_type,'2')
%     for j = 1 : N
%         T_w_tcp(:,:,j)  = [quat2rotm(pose_exp(j,4:7)),pose_exp(j,1:3)';0 0 0 1];
%         % Rotation tcp frame w.r.t tracker frame
%         R_tr_tcp        = roty(180)*rotx(135);
%         % Position tracker frame -> tcp frame expressed in the tracker frame
% %         p_tr_tcp        = [0,-57.02-42.67,55.59]./1000;
%         p_tr_tcp        = [0,-57.02-42.67,35.5]./1000;
%         T_tr_tcp        = [R_tr_tcp,p_tr_tcp';0 0 0 1];
%         T_w_tr(:,:,j)   = T_w_tcp(:,:,j)*inverse_pose(T_tr_tcp);
% 
%         wrench_w_lc_tr_tr(j,:) = S_transform(T_tr_tcp,wrench_exp(j,:));
%     end
% elseif strcmp(exp_type,'3')
%     for j = 1 : N
%         T_w_tcp(:,:,j)  = [quat2rotm(pose_exp(j,4:7)),pose_exp(j,1:3)';0 0 0 1];
%         % Rotation tcp frame w.r.t tracker frame
%         R_tr_tcp        = roty(180)*rotx(135);
%         % Position tracker frame -> tcp frame expressed in the tracker frame
% %         p_tr_tcp        = [0,-57.02-42.67,55.59]./1000;
%         p_tr_tcp        = [0,-57.02-42.67,35.5]./1000;
%         T_tr_tcp        = [R_tr_tcp,p_tr_tcp';0 0 0 1];
%         T_w_tr(:,:,j)   = T_w_tcp(:,:,j)*inverse_pose(T_tr_tcp);
% 
%         wrench_w_lc_tr_tr(j,:) = S_transform(T_tr_tcp,wrench_exp(j,:));
%     end
% elseif strcmp(exp_type,'4')
%     for j = 1 : N
%         T_w_tcp(:,:,j)  = [quat2rotm(pose_exp(j,4:7)),pose_exp(j,1:3)';0 0 0 1];
%         % Rotation tcp frame w.r.t tracker frame
%         R_tr_tcp        = rotx(135);
%         % Position tracker frame -> tcp frame expressed in the tracker frame
% %         p_tr_tcp        = [0,-57.02-42.67,55.59]./1000;
%         p_tr_tcp        = [0,-57.02-42.67,35.5]./1000;
%         T_tr_tcp        = [R_tr_tcp,p_tr_tcp';0 0 0 1];
%         T_w_tr(:,:,j)   = T_w_tcp(:,:,j)*inverse_pose(T_tr_tcp);
% 
%         wrench_w_lc_tr_tr(j,:) = S_transform(T_tr_tcp,wrench_exp(j,:));
%     end
% elseif strcmp(exp_type,'5')
%     for j = 1 : N
%         T_w_tcp(:,:,j)  = [quat2rotm(pose_exp(j,4:7)),pose_exp(j,1:3)';0 0 0 1];
%         % Rotation tcp frame w.r.t tracker frame
%         R_tr_tcp        = rotx(135);
%         % Position tracker frame -> tcp frame expressed in the tracker frame
% %         p_tr_tcp = [0,-261.69,55.59]./1000;
%         p_tr_tcp = [0,-261.69,35.5]./1000;
%         T_tr_tcp        = [R_tr_tcp,p_tr_tcp';0 0 0 1];
%         T_w_tr(:,:,j)   = T_w_tcp(:,:,j)*inverse_pose(T_tr_tcp);
% 
%         wrench_w_lc_tr_tr(j,:) = S_transform(T_tr_tcp,wrench_exp(j,:));
%     end
% elseif strcmp(exp_type,'6')
%     for j = 1 : N
%         T_w_tcp(:,:,j)  = [quat2rotm(pose_exp(j,4:7)),pose_exp(j,1:3)';0 0 0 1];
%         % Rotation tcp frame w.r.t tracker frame
%         R_tr_tcp = roty(90)*rotx(135); % tr towards tcp R1 * R2
%         % Position tracker frame -> tcp frame expressed in the tracker frame
% %         p_tr_tcp = [0,-261.69,55.59]./1000;
%         p_tr_tcp = [0,-261.69,35.5]./1000;
%         T_tr_tcp        = [R_tr_tcp,p_tr_tcp';0 0 0 1];
%         T_w_tr(:,:,j)   = T_w_tcp(:,:,j)*inverse_pose(T_tr_tcp);
% 
%         wrench_w_lc_tr_tr(j,:) = S_transform(T_tr_tcp,wrench_exp(j,:));
%     end
% elseif strcmp(exp_type,'7')
%     for j = 1 : N
%         T_w_tcp(:,:,j)  = [quat2rotm(pose_exp(j,4:7)),pose_exp(j,1:3)';0 0 0 1];
%         % Rotation tcp frame w.r.t tracker frame
%         R_tr_tcp = roty(180)*rotx(135); % tr towards tcp R1 * R2
%         % Position tracker frame -> tcp frame expressed in the tracker frame
% %         p_tr_tcp = [0,-261.69,55.59]./1000;
%         p_tr_tcp = [0,-261.69,35.5]./1000;
%         T_tr_tcp        = [R_tr_tcp,p_tr_tcp';0 0 0 1];
%         T_w_tr(:,:,j)   = T_w_tcp(:,:,j)*inverse_pose(T_tr_tcp);
% 
%         wrench_w_lc_tr_tr(j,:) = S_transform(T_tr_tcp,wrench_exp(j,:));
%     end
% elseif strcmp(exp_type,'8')
%     for j = 1 : N
%         T_w_tcp(:,:,j)  = [quat2rotm(pose_exp(j,4:7)),pose_exp(j,1:3)';0 0 0 1];
%         % Rotation tcp frame w.r.t tracker frame
%         R_tr_tcp = roty(-90)*rotx(135); % tr towards tcp R1 * R2
%         % Position tracker frame -> tcp frame expressed in the tracker frame
% %         p_tr_tcp = [0,-261.69,55.59]./1000;
%         p_tr_tcp = [0,-261.69,35.5]./1000;
%         T_tr_tcp        = [R_tr_tcp,p_tr_tcp';0 0 0 1];
%         T_w_tr(:,:,j)   = T_w_tcp(:,:,j)*inverse_pose(T_tr_tcp);
% 
%         wrench_w_lc_tr_tr(j,:) = S_transform(T_tr_tcp,wrench_exp(j,:));
%     end
% end




