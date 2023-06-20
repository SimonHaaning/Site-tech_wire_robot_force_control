function [position, velocity, acceleration] = QuinticPolynomial(start_state, end_state, times)
% INPUTS:
% - start_state: VAR1x3 matrix, initial values with columns of position, velocity, acceleration
% - end_state: VAR1x3 matrix, destination values with columns of position, velocity, acceleration
% - times: 1xVAR2, list of scalar timestamps for evaluated polynomial output 
%
% OUTPUT:
% - position: VAR1xVAR2 matrix, list of vectors containing positions at all times
% - velocity: VAR1xVAR2 matrix, list of vectors containing velocities at all times
% - acceleration: VAR1xVAR2 matrix, list of vectors containing accelerations at all times

% Calculate coefficients with time unit
a0 = start_state(:,1);
a1 = start_state(:,2);
a2 = start_state(:,3)/2;
a3 = -(20*start_state(:,1) - 20*end_state(:,1) + 8*times(length(times))*end_state(:,2) + 12*times(length(times))*start_state(:,2) - times(length(times))^2*end_state(:,3) + 3*times(length(times))^2*start_state(:,3))/(2*times(length(times))^3);
a4 = (30*start_state(:,1) - 30*end_state(:,1) + 14*times(length(times))*end_state(:,2) + 16*times(length(times))*start_state(:,2) - 2*times(length(times))^2*end_state(:,3) + 3*times(length(times))^2*start_state(:,3))/(2*times(length(times))^4);
a5 = -(12*start_state(:,1) - 12*end_state(:,1) + 6*times(length(times))*end_state(:,2) + 6*times(length(times))*start_state(:,2) - times(length(times))^2*end_state(:,3) + times(length(times))^2*start_state(:,3))/(2*times(length(times))^5);

% Generate trajectory polynomials
position = a0 + a1.*times + a2.*times.^2 + a3.*times.^3 + a4.*times.^4 + a5.*times.^5;
velocity = a1 + a2.*(2*times) + a3.*(3*times.^2) + a4.*(4*times.^3) + a5.*(5*times.^4);
acceleration = a2.*(2) + a3.*(6*times) + a4.*(12*times.^2) + a5.*(20*times.^3);
end