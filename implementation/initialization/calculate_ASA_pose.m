function T_ASA = calculate_ASA_pose(screw_trajectory,regul_origin,prior_origin)
%%
% This function calculates the pose of the ASA frame (Average Screw Axis) 
% for a given screw trajectory.
%
% Interpretation of the ASA frame:
% - The first axis corresponds to the average direction of the screw axis 
% across the whole trajectory.
% - The second axis corresponds to the direction with the most variation in 
% the orientation of the screw axis with respect to the average.
% - The third axis is fully determined by the first two axes.
% - The origin of the ASA frame corresponds to the approximate intersection
% point of all the screw axes.
%
% Input:
%   screw_trajectory (Nx6)  : Columns 1-3 contain the directional part of the screw.
%                             Columns 4-6 contain the moment part of the screw.
%   regul_origin       : (optional) regularization weight on prior value for origin (default: 1e-10)
%   prior_origin (3x1) : (optional) prior value for origin for singular case (default: [0;0;0])
%
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

% Deal with missing arguments in the function
if nargin < 2
    regul_origin = 1e-6;
    prior_origin = [0;0;0];
elseif nargin < 3
    prior_origin = [0;0;0];
end

% Calculate different average sums from the screw components
average_vector = zeros(3,1);
outer_product_sum = zeros(3,3);
cross_product_sum = zeros(3,1);
skew_product_sum = zeros(3,3);
for k = 1:N
    d = screw_trajectory(k,1:3); % direction component (rot. vel or force)
    m = screw_trajectory(k,4:6); % moment component (transl. vel or moment)

    average_vector = average_vector + d';
    outer_product_sum = outer_product_sum + d'*d;
    cross_product_sum = cross_product_sum + cross(d',m');
    skew_product_sum = skew_product_sum + skew(d)*skew(d);
end
outer_product_sum = outer_product_sum/N;
cross_product_sum = cross_product_sum/N;
skew_product_sum = skew_product_sum/N;
average_vector = average_vector/N;

% Determine ASA frame orientation
[U_ASA,~,~] = svd(outer_product_sum); % retrieve matrix of singular vectors
if norm(average_vector) == 0 % no preferred sign
    e1 = U_ASA(:,1); % first axis = first singular vector
else
    % first axis = first singular vector that is sign-corrected by average vector 
    e1 = U_ASA(:,1) * sign(dot(U_ASA(:,1),average_vector));
end
e2 = U_ASA(:,2); % second axis = second singular vector
e3 = cross(e1,e2); % third axis

% Determine ASA frame origin
po = (-skew_product_sum + regul_origin*eye(3)) \ (cross_product_sum + regul_origin*prior_origin);

% Return result as homogeneous pose matrix
T_ASA = [e1 e2 e3 po; 0 0 0 1];

end