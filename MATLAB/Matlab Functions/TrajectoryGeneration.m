function [traj_position, traj_velocity, traj_acceleration, traj_times] = TrajectoryGeneration(start_pos, end_pos, resolution, acceleration_limits)
% INPUTS:
% - start_pos: 3x3 matrix, initial X, Y, rotZ with columns of position, velocity, acceleration
% - end_pos: 3x3 matrix, destination X, Y, rotZ with columns of position, velocity, acceleration
% - resolution: scalar, points per unit time in output
% - acceleration_limits: 3x1 vector, maximum limits for X, Y, and rotZ acceleration
%
% OUTPUT:
% - traj_position: 3xVAR matrix, list of 3x1 vectors containing positions
% - traj_velocity: 3xVAR matrix, list of 3x1 vectors containing velocities
% - traj_acceleration: 3xVAR matrix, list of 3x1 vectors containing accelerations
% - traj_times: 1xVAR, list of timestamps for trajectory

% Calculate completion time
traj_duration = AccelerationLimitedCompletionTime(start_pos, end_pos, acceleration_limits);

% Generate list of time points with desired resolution
traj_times = 0:1/resolution:traj_duration;

% Calculate polynomials and generate points along trajctory
[traj_position, traj_velocity, traj_acceleration] = QuinticPolynomial(start_pos, end_pos, traj_times);
end