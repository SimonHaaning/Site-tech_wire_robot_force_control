function completion_time = AccelerationLimitedCompletionTime(start_pos, end_pos, acceleration_limits)
% INPUTS:
% - start_pos: 3x3 matrix, initial X, Y, rotZ with columns of position, velocity, acceleration
% - end_pos: 3x3 matrix, destination X, Y, rotZ with columns of position, velocity, acceleration
% - acceleration_limits: 3x1 vector, columns containing maximum limits for X, Y, and rotZ acceleration
%
% OUTPUT:
% - completion_time: scalar, highest required completion time
%
% INTERNAL PARAMETERS:
tolerance = 0.999; % Tolerance scalar for acceleration limit
completion_time = 1; % completion time for polynomial in seconds
%
% NOTE: Function will only use the 

% Calculate polynomial for acceleration with completion time of 1 second
[~,~,acceleration] = QuinticPolynomial(start_pos, end_pos, 0:completion_time/100:completion_time);

if any(max(abs(acceleration), [], 2) > acceleration_limits)
    % Determine upper and lower bound for completion time
    while any(max(abs(acceleration), [], 2) > acceleration_limits)
        completion_time = completion_time * 2;
        [~,~,acceleration] = QuinticPolynomial(start_pos, end_pos, 0:completion_time/100:completion_time); % Recalculate acceleration
    end
    
    % Check to see if binary search is redundant
    if all(max(abs(acceleration), [], 2) <= acceleration_limits) && any(max(abs(acceleration), [], 2) >= acceleration_limits*tolerance)
        return
    end

    % Define upper and lower bound for completion time
    completion_time_upper = completion_time;
    completion_time_lower = completion_time / 2;
    
    % Perform binary search to refine completion time
    while 1
        completion_time = completion_time_lower + (completion_time_upper - completion_time_lower)/2; % Pick new completion time between upper and lower bound
        [~,~,acceleration] = QuinticPolynomial(start_pos, end_pos, 0:completion_time/100:completion_time); % Recalculate acceleration
        if all(max(abs(acceleration), [], 2) <= acceleration_limits) && any(max(abs(acceleration), [], 2) >= acceleration_limits*tolerance)
            break
        elseif any(max(abs(acceleration), [], 2) > acceleration_limits)
            completion_time_lower = completion_time; % At least one acceleration is too large, setting new lower bound for completion time
        elseif all(max(abs(acceleration), [], 2) < acceleration_limits*tolerance)
            completion_time_upper = completion_time; % All accelerations are too small, setting new lower bound
        end
    end
end
end