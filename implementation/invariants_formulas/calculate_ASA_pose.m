function T_ASA = calculate_ASA_pose(screw_trajectory)
%%
% This function calculates the pose of the ASA frame (Average Screw Axis) 
% of a given screw trajectory
%
% Input:
%   screw_trajectory (Nx6)  : Columns 1-3 contain the directional part of the screw.
%                             Columns 4-6 contain the translational part of the screw.
% Output:
%   T_ASA       (4x4)  : homogeneous pose matrix containing the orientation
%                        and position of the ASA-frame
%
% Reference: 
% Ancillao, A., Vochten, M., Verduyn, A., De Schutter, J., Aertbelien, E., 
% An optimal method for calculating an average screw axis for a joint, with 
% improved sensitivity to noise and providing an analysis of the dispersion 
% of the instantaneous axes. PLOS ONE, vol. 17, no. 10, pp. 1–22, 2022
% 

N = size(screw_trajectory,1);
outer = zeros(3,3);
cross_product = zeros(3,1);
skew_product = zeros(3,3);
for k = 1:N
    outer = outer + screw_trajectory(k,1:3)'*screw_trajectory(k,1:3);
    cross_product = cross_product + cross(screw_trajectory(k,1:3)',screw_trajectory(k,4:6)');
    skew_product = skew_product + skew(screw_trajectory(k,1:3)')*skew(screw_trajectory(k,1:3)');
end
outer = outer/N;
cross_product = cross_product/N;
skew_product = skew_product/N;

[U_ASA,~,~] = svd(outer);
position_ASA = -skew_product\cross_product;
T_ASA = eye(4);
T_ASA(1:3,1:3) = U_ASA;
T_ASA(1:3,4) = position_ASA;

end