function [f_optimal, F, Mz, converge_iter, Err] = InverseWireDynamics(F_desired, Mz_desired, wire_directions, moment_arms)
% INPUTS:
% - F_desired: 2x1 vector, the desired X and Y forces
% - Mz_desired: scalar, the desired moment around Z
% - wire_directions: 3x5 matrix, columms are unit vectors for wire directions
% - moment_arms: 3x5 matrix, columms are vectors to motor position
%
% OUTPUT:
% - f_opt: 1x5 vector, optimized motor force values
% - F: 3x1 vector, resultant X and Y forces
% - Mz: scalar, resultant moment around Z
% - converge_iter: scalar, iterations before convergence
%
% INTERNAL PARAMETERS:
f_min = 100;        % Minimum wire force
f_max = 10000;      % Maximum wire force
step_size_Fx = 0.5; % Step size (learning rate) for Fx
step_size_Fy = 0.5; % Step size (learning rate) for Fx
step_size_Mz = 1;   % Step size (learning rate) for Mz
F_tolerance = 1;    % Accepted deviation from desired force
Mz_tolerance = 0.01;% Accepted deviation from desired moment
max_iter = 8000;    % Maximum iterations of gradient descent

% Initialize f for first iteration
f = f_min;

% Initialize convergence counter in case it does not converge
converge_iter = max_iter + 1;

% Calculate the gradients with respect to each f value
dF_df = wire_directions;
dM_df = cross(moment_arms, wire_directions);

for i = 1:max_iter
    % Calculate the total force and moment using the current f values
    F = sum(f .* wire_directions, 2);
    M = sum(cross(moment_arms, wire_directions .* f), 2);
    Mz = M(3);

    % Calculate error
    F_err = F(1:2, :) - F_desired(1:2, :);
    Mz_err = Mz - Mz_desired;
    Err = [F_err; Mz_err]; % Combined for output

    % Check for convergence
    if norm(F_err) < F_tolerance
        if abs(Mz_err) < Mz_tolerance
            converge_iter = i;
            break;
        end
    end

    % Update f values using gradient descent
    f = f - step_size_Fx * F_err(1) * dF_df(1, :) - step_size_Fy * F_err(2) * dF_df(2, :) - step_size_Mz * Mz_err * dM_df(3, :);

    % Apply constraints to f values
    f = max(f_min, min(f_max, f));
end

% Return optimized f values
f_optimal = f;
end