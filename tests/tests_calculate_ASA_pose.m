% Test whether special screw trajectories return a valid result when
% calculating the Average Screw Axis (ASA) frame.

addpath("..\implementation\invariants_formulas")

% Zero trajectory
screw_trajectory1 = zeros(100,6);
ASA1 = calculate_ASA_pose(screw_trajectory1);

% Parallel intersecting screw axes
screw_trajectory2 = repmat([1 zeros(1,5)],100,1);
ASA2 = calculate_ASA_pose(screw_trajectory2);

% General case
screw_trajectory3 = randn(100,6);
ASA3 = calculate_ASA_pose(screw_trajectory3);
