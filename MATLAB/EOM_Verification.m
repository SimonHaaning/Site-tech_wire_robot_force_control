clear     % Clear workspace
clc       % Clear command window
close all % Close all figures

% Anchor positions static frame (m)
a1 = [  0; 4; 0];
a2 = [  2; 5; 0];
a3 = [  4; 4; 0];
a4 = [  4; 0; 0];
a5 = [  0; 0; 0];
anchors_pos = [a1, a2, a3, a4, a5];

% Motor positions relative to CoM (m)
m1 = [-0.5;  0.4; 0.3];
m2 = [   0;  0.4; 0.3];
m3 = [ 0.5;  0.4; 0.3];
m4 = [ 0.5; -0.4; 0.3];
m5 = [-0.5; -0.4; 0.3];
motors_pos = [m1, m2, m3, m4, m5];

% Inertial parameters
m_frame = 300; % (kg)
I_frame = 101.85669; % (kg*m^2)

% External forces
F_g = [0; -9.82*m_frame; 0]; % Gravity
%F_f = [] % Friction

% IMU direction data (wire 1 & 3 direction)
w1_dir = [-0.6985; 0.7141; -0.0466]; % Centered for verify
w3_dir = [ 0.6985; 0.7141; -0.0466]; % Centered for verify

% Desired accelerations
q_ddot = [0; 0; 0];     % Translatory acceleration not possible along z
theta_ddot = [0; 0; 0]; % Angular acceleration only possible around z

%% Forward Kinematics
[CoM_pos, frame_ang] = ForwardWireKinematics(w1_dir, w3_dir, anchors_pos, motors_pos);

%% Inverse Kinematics
CoM_pos = [2; 2; 0];
frame_ang = 3.0;
[wires_dir, ~] = InverseWireKinematics(CoM_pos, frame_ang, anchors_pos, motors_pos)

%% Inverse Dynamics
% Calculate desired force and moment
F_desired = q_ddot*m_frame - F_g;
M_desired = theta_ddot*I_frame;

% Determine moment arm orientations
R_frame = [cos(frame_ang), -sin(frame_ang), 0; 
           sin(frame_ang),  cos(frame_ang), 0; 
                        0,               0, 1];
moment_arms = R_frame*motors_pos;

% Calculate optimal motor forces to reach desired total forces and moments
[f_opt, ~, ~, converg_iter, Error] = InverseWireDynamics(F_desired, M_desired(3), wires_dir, moment_arms);

%% Calculate statics in square workspace
% Calculate desired force and moment
F_desired = q_ddot*m_frame - F_g;
M_desired = theta_ddot*I_frame;

% No angular displacement
frame_ang = 0;

% Determine moment arm orientations
R_frame = [cos(frame_ang), -sin(frame_ang), 0; 
           sin(frame_ang),  cos(frame_ang), 0; 
                        0,               0, 1];
moment_arms = R_frame*motors_pos;

% CoM position increments
resolution = 10;
x_coords = linspace(0, 4, resolution);
y_coords = linspace(-2, 4, resolution);

% Output arrays
Out_f = zeros(resolution, resolution, 5);
Out_E = zeros(resolution, resolution, 3);
Out_iter = zeros(resolution, resolution);
for i = 1:resolution
    for j = 1:resolution
        % CoM position
        CoM_pos = [x_coords(i); y_coords(j); 0]; 
        
        % Inverse kinematics
        [wires_dir, ~] = InverseWireKinematics(CoM_pos, frame_ang, anchors_pos, motors_pos);

        % Inverse dynamics
        [Out_f(j,i,:), ~, ~, Out_iter(j,i), Out_E(j,i,:)] = InverseWireDynamics(F_desired, M_desired(3), wires_dir, moment_arms);
    end
    % scuffed Progress counter
    %clc
    %completion = vpa(i/resolution*100, 3)
end

% Plot results
fig1 = figure;
tick_location = 1:resolution/10:resolution;

subplot(2, 3, 1);
imagesc(abs(Out_E(:, :, 1))) 
title("Horizontal force errors");
xlabel("CoM X pos")
xticks(tick_location)
xticklabels(x_coords)
ylabel("CoM Y pos")
yticks(tick_location)
yticklabels(y_coords)
axis xy
colorbar

subplot(2, 3, 2);
imagesc(abs(Out_E(:, :, 2))) 
title("Vertical force errors");
xlabel("CoM X pos")
xticks(tick_location)
xticklabels(x_coords)
ylabel("CoM Y pos")
yticks(tick_location)
yticklabels(y_coords)
axis xy
colorbar

subplot(2, 3, 3);
imagesc(abs(Out_E(:, :, 3))) 
title("Moment errors");
xlabel("CoM X pos")
xticks(tick_location)
xticklabels(x_coords)
ylabel("CoM Y pos")
yticks(tick_location)
yticklabels(y_coords)
axis xy
colorbar

subplot(2, 3, 4);
imagesc(max(Out_f,[],3)-min(Out_f,[],3)) 
title("Difference between max and min motor force");
xlabel("CoM X pos")
xticks(tick_location)
xticklabels(x_coords)
ylabel("CoM Y pos")
yticks(tick_location)
yticklabels(y_coords)
axis xy
colorbar

%subplot(2, 3, 5);
%imagesc(min(Out_f,[],3)) 
%title("Minimum motor force");
%xlabel("CoM X pos")
%xticks(tick_location)
%xticklabels(coords)
%ylabel("CoM Y pos")
%yticks(tick_location)
%yticklabels(coords)
%axis xy
%colorbar

subplot(2, 3, 6);
imagesc(Out_iter) 
title("Iterations before convergence");
xlabel("CoM X pos")
xticks(tick_location)
xticklabels(x_coords)
ylabel("CoM Y pos")
yticks(tick_location)
yticklabels(y_coords)
axis xy
colorbar

%% Calculate along trajectory
start_pos = [[1.9999; 2; 0], [0; 0; 0], [0; 0; 0]];  % Start of trajectory X, Y, rotZ
end_pos = [[2.0001; -4; 0], [0; 0; 0], [0; 0; 0]];    % End of trajectory X, Y, rotZ
resolution = 100; % Points per second of the trajectory
acc_limit = [0.05; 0.05; 0.001]; % Maximum acceleration allowed for X, Y, rotZ

% Call trajectory function (Returns lists of pos, vel, acc, times)
[traj_pos, traj_vel, traj_acc, traj_times] = TrajectoryGeneration(start_pos, end_pos, resolution, acc_limit);

len = length(traj_times);

% Output arrays
Out_f = zeros(5, len);
Out_F = zeros(3, len);
Out_Mz = zeros(1, len);
Out_E = zeros(3, len);
Out_iter = zeros(1, len);

tic();
for i = 1:len
    % Determine moment arm orientations
    frame_ang = traj_pos(3, i);
    R_frame = [cos(frame_ang), -sin(frame_ang), 0; 
               sin(frame_ang),  cos(frame_ang), 0; 
                            0,               0, 1];
    moment_arms = R_frame*motors_pos;
    
    % Inverse kinematics
    CoM_pos = [traj_pos(1:2, i); 0];
    [wires_dir, ~] = InverseWireKinematics(CoM_pos, frame_ang, anchors_pos, motors_pos);

    % Calculate desired force and moment
    F_desired = [traj_acc(1:2, i); 0]*m_frame - F_g;
    Mz_desired = traj_acc(3, i)*I_frame;

    % Inverse dynamics 
    [Out_f(:,i), Out_F(:,i), Out_Mz(i), Out_iter(i), Out_E(:,i)] = InverseWireDynamics(F_desired, Mz_desired, wires_dir, moment_arms);

    % scuffed Progress counter
    %clc
    %i/len
end
toc()

% Plot forces results
fig2 = figure;
subplot(5, 2, 1);
plot(traj_times, Out_f(1, :))
title("Motor 1 force");
xlabel("time (s)")
xlim([0, max(traj_times)])
ylabel("force (N)")
ylim([0, 10000])

subplot(5, 2, 3);
plot(traj_times, Out_f(2, :))
title("Motor 2 force");
xlabel("time (s)")
xlim([0, max(traj_times)])
ylabel("force (N)")
ylim([0, 10000])

subplot(5, 2, 5);
plot(traj_times, Out_f(3, :))
title("Motor 3 force");
xlabel("time (s)")
xlim([0, max(traj_times)])
ylabel("force (N)")
ylim([0, 10000])

subplot(5, 2, 7);
plot(traj_times, Out_f(4, :))
title("Motor 4 force");
xlabel("time (s)")
xlim([0, max(traj_times)])
ylabel("force (N)")
ylim([0, 10000])

subplot(5, 2, 9);
plot(traj_times, Out_f(5, :))
title("Motor 5 force");
xlabel("time (s)")
xlim([0, max(traj_times)])
ylabel("force (N)")
ylim([0, 10000])

subplot(5, 2, 2);
plot(traj_times, Out_F(1, :))
title("Sum of X forces");
xlabel("time (s)")
xlim([0, max(traj_times)])
ylabel("force (N)")
ylim([floor(min(Out_F(1,:), [], "all")), ceil(max(Out_F(1,:), [], "all"))])

subplot(5, 2, 4);
plot(traj_times, Out_F(2, :) + F_g(2))
title("Sum of Y forces");
xlabel("time (s)")
xlim([0, max(traj_times)])
ylabel("force (N)")
ylim([floor(min(Out_F(2,:) + F_g(2), [], "all")), ceil(max(Out_F(2,:) + F_g(2), [], "all"))])

subplot(5, 2, 6);
plot(traj_times, Out_Mz)
title("Sum of Z moments");
xlabel("time (s)")
xlim([0, max(traj_times)])
ylabel("moments (N*m)")
ylim([floor(min(Out_Mz, [], "all")), ceil(max(Out_Mz, [], "all"))])

% Plot trajectory results
fig3 = figure;
subplot(6, 6, [1, 2, 3, 4, 7, 8, 9, 10, 13, 14, 15, 16, 19, 20, 21, 22]);
hold on
plot(traj_pos(1,:),traj_pos(2,:))
PlotRobot([traj_pos(1:2,1); 0], traj_pos(3,1), anchors_pos, motors_pos, 'g')
PlotRobot([traj_pos(1:2,len); 0], traj_pos(3,len), anchors_pos, motors_pos, 'r')
hold off
title("Position trajectory");
xlabel("CoM X (m)")
ylabel("CoM Y (m)")
axis equal

subplot(6, 6, [5, 6]);
plot(traj_times, traj_vel(1,:))
title("X velocity trajectory");
xlabel("time (s)")
xlim([0, max(traj_times)])
ylabel("m/s")
ylim([min(traj_vel, [], "all"), max(traj_vel, [], "all")])

subplot(6, 6, [11, 12]);
plot(traj_times, traj_vel(2,:))
title("Y velocity trajectory");
xlabel("time (s)")
xlim([0, max(traj_times)])
ylabel("m/s")
ylim([min(traj_vel, [], "all"), max(traj_vel, [], "all")])

subplot(6, 6, [17, 18]);
plot(traj_times, traj_vel(3,:))
title("rotZ velocity trajectory");
xlabel("time (s)")
xlim([0, max(traj_times)])
ylabel("rad/s")
ylim([min(traj_vel, [], "all"), max(traj_vel, [], "all")])

subplot(6, 6, [23, 24]);
plot(traj_times, traj_acc(1,:))
title("X acceleration trajectory");
xlabel("time (s)")
xlim([0, max(traj_times)])
ylabel("m/s^2")
%ylim([min(traj_acc, [], "all"), max(traj_acc, [], "all")])
ylim([-acc_limit(1), acc_limit(1)])

subplot(6, 6, [29, 30]);
plot(traj_times, traj_acc(2,:))
title("Y acceleration trajectory");
xlabel("time (s)")
xlim([0, max(traj_times)])
ylabel("m/s^2")
%ylim([min(traj_acc, [], "all"), max(traj_acc, [], "all")])
ylim([-acc_limit(2), acc_limit(2)])

subplot(6, 6, [35, 36]);
plot(traj_times, traj_acc(3,:))
title("rotZ acceleration trajectory");
xlabel("time (s)")
xlim([0, max(traj_times)])
ylabel("rad/s^2")
%ylim([min(traj_acc, [], "all"), max(traj_acc, [], "all")])
ylim([-acc_limit(3), acc_limit(3)])

%% Show robot motion from trajectory forces
figure;
SimulateMotionFromForces(start_pos, Out_f, traj_times, anchors_pos, motors_pos);
