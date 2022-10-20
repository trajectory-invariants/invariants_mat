function [start_segment,end_segment] = detect_velocity_segments(velocity,vel_thr)
% This function segments the part of motion which has translational
% velocity.
% 
% Input:
%     velocity:         translational velocity of the desired point             [Nx3]
%     vel_thr:          desired threshold for the translational velocity        [1]
% Output:
%     start_segment:    start of the translational part of motion               [1]
%     end_segment:      end of the translational part of motion                 [1]

%% finding segments based on velocity
N = length(velocity);

start_segment = zeros(N,1);
end_segment = zeros(N,1);
norm_velocity(1,:) = norm(velocity(1,:));
for i = 2 : N
    norm_velocity(i,:) = norm(velocity(i,:));
    if norm_velocity(i,:) > vel_thr && norm_velocity(i-1,:) < vel_thr && i < 0.5*N
        start_segment(i,1) = i;
    elseif norm_velocity(i,:) < vel_thr && norm_velocity(i-1,:) > vel_thr && i > 0.5*N
        end_segment(i,1) = i-1;
    end
end

start_segment = start_segment(start_segment~=0);
end_segment = end_segment(end_segment~=0);

if ~isempty(start_segment)
    start_segment = start_segment(1);
else
    start_segment = 1;
end

if ~isempty(end_segment)
    end_segment = end_segment(end);
else
    end_segment = N;
end