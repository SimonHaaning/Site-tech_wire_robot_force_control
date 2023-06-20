function SimulateMotionFromForces(start_pos, motor_forces, times, anchors_positions, motors_pos_local)
% INPUTS:
% - start_pos: 3x3 matrix, initial X, Y, rotZ with columns of position, velocity, acceleration
% - motor_forces: 5xVAR matrix, list of motor forces at times
% - times: 1xVAR vector, list of times
% - anchor_positions: 3x5 matrix, anchor positions
% - motors_pos_local: 3x5 matrix, motor positions relative to CoM
%
% INTERNAL PARAMETERS:
m_frame = 300; % (kg)
I_frame = 101.85669; % (kg*m^2)
positions = zeros(3, length(times));
velocities = zeros(3, length(times));
accelerations = zeros(3, length(times));
%Mz_traj = zeros(1, length(times)); DEBUG
%F_traj = zeros(3, length(times)); DEBUG

% Fill out initial state
positions(:,1) = start_pos(:, 1);
velocities(:,1) = start_pos(:, 2);
accelerations(:,1) = start_pos(:, 3);

for i = 2:length(times)
    % Calculate wire directions of previous position
    [wires_dir, ~] = InverseWireKinematics([positions(1:2, i-1); 0], positions(3, i-1), anchors_positions, motors_pos_local);
    
    % Calculate sum of forces on CoM
    F_sum = sum(wires_dir .* motor_forces(:, i)', 2) + [0; -9.82*m_frame; 0];
    %F_traj(:, i) = F_sum;

    % Determine moment arm orientations
    frame_ang = positions(3, i-1);
    R_frame = [cos(frame_ang), -sin(frame_ang), 0; 
               sin(frame_ang),  cos(frame_ang), 0; 
                            0,               0, 1];
    moment_arms = R_frame*motors_pos_local;

    % Calculate sum of moments about z
    M = cross(moment_arms, wires_dir .* motor_forces(:, i)');
    Mz_sum = sum(M(3,:));
    %Mz_traj(i) = Mz_sum;

    % Determine time step size
    dt = times(i) - times(i-1);

    % Update acceleration, velocity, and position using Newton's laws
    accelerations(:, i) = [F_sum(1:2); Mz_sum]./[m_frame; m_frame; I_frame];
    velocities(:, i) = velocities(:, i-1) + accelerations(:, i)*dt;
    positions(:, i) = positions(:, i-1) + velocities(:, i-1)*dt + 0.5*accelerations(:, i)*dt^2;
end

% Generate figure
subplot(3, 6, [1,2])
plot(times, positions(1,:))
title("X position trajectory");
xlabel("time (s)")
xlim([0, max(times)])
ylabel("m")
ylim([floor_deci(min(positions(1,:), [], "all"), 0.1), ceil_deci(max(positions(1,:), [], "all"), 0.1)])

subplot(3, 6, [3, 4]);
plot(times, velocities(1,:))
title("X velocity trajectory");
xlabel("time (s)")
xlim([0, max(times)])
ylabel("m/s")
ylim([floor_deci(min(velocities(1,:), [], "all"), 0.1), ceil_deci(max(velocities(1,:), [], "all"), 0.1)])

subplot(3, 6, [5, 6]);
plot(times, accelerations(1,:))
title("X acceleration trajectory");
xlabel("time (s)")
xlim([0, max(times)])
ylabel("m/s^2")
ylim([floor_deci(min(accelerations(1,:), [], "all"), 0.1), ceil_deci(max(accelerations(1,:), [], "all"), 0.1)])

subplot(3, 6, [7,8])
plot(times, positions(2,:))
title("Y position trajectory");
xlabel("time (s)")
xlim([0, max(times)])
ylabel("m")
ylim([floor_deci(min(positions(2,:), [], "all"), 0.1), ceil_deci(max(positions(2,:), [], "all"), 0.1)])

subplot(3, 6, [9, 10]);
plot(times, velocities(2,:))
title("Y velocity trajectory");
xlabel("time (s)")
xlim([0, max(times)])
ylabel("m/s")
ylim([floor_deci(min(velocities(2,:), [], "all"), 0.1), ceil_deci(max(velocities(2,:), [], "all"), 0.1)])

subplot(3, 6, [11, 12]);
plot(times, accelerations(2,:))
title("Y acceleration trajectory");
xlabel("time (s)")
xlim([0, max(times)])
ylabel("m/s^2")
ylim([floor_deci(min(accelerations(2,:), [], "all"), 0.1), ceil_deci(max(accelerations(2,:), [], "all"), 0.1)])

subplot(3, 6, [13,14])
plot(times, positions(3,:))
title("Z position trajectory");
xlabel("time (s)")
xlim([0, max(times)])
ylabel("rad")
ylim([floor_deci(min(positions(3,:), [], "all"), 0.1), ceil_deci(max(positions(3,:), [], "all"), 0.1)])

subplot(3, 6, [15, 16]);
plot(times, velocities(3,:))
title("rotZ velocity trajectory");
xlabel("time (s)")
xlim([0, max(times)])
ylabel("rad/s")
ylim([floor_deci(min(velocities(3,:), [], "all"), 0.1), ceil_deci(max(velocities(3,:), [], "all"), 0.1)])

subplot(3, 6, [17, 18]);
plot(times, accelerations(3,:))
title("rotZ acceleration trajectory");
xlabel("time (s)")
xlim([0, max(times)])
ylabel("rad/s^2")
ylim([floor_deci(min(accelerations(3,:), [], "all"), 0.1), ceil_deci(max(accelerations(3,:), [], "all"), 0.1)])

% Plot forces and moments (for debug)
%subplot(4,2, [1,2])
%plot(times, F_traj(1,:))
%title("X force trajectory");
%xlabel("time (s)")
%xlim([0, max(times)])
%ylabel("N")
%ylim([min(F_traj(1,:), [], "all"), max(F_traj(1,:), [], "all")])

%subplot(4,2, [3,4])
%plot(times, F_traj(2,:))
%title("Y force trajectory");
%xlabel("time (s)")
%xlim([0, max(times)])
%ylabel("N")
%ylim([min(F_traj(2,:), [], "all"), max(F_traj(2,:), [], "all")])

%subplot(4,2, [5,6])
%plot(times, F_traj(3,:))
%title("Z force trajectory");
%xlabel("time (s)")
%xlim([0, max(times)])
%ylabel("N")
%ylim([min(F_traj(3,:), [], "all"), max(F_traj(3,:), [], "all")])

%subplot(4,2, [7,8])
%plot(times, Mz_traj)
%title("Z moments trajectory");
%xlabel("time (s)")
%xlim([0, max(times)])
%ylabel("N*m")
%ylim([min(Mz_traj, [], "all"), max(Mz_traj, [], "all")])
end