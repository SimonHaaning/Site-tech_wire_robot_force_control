clear   % Clear workspace
clc     % Clear command window
%% Parameters
% Physics parameters
syms g
%g = 9.82; % Gravitational force constant (N/kg)

% Inertial parameters
syms m_frame I_zframe
%m_frame = 10; % Mass of the robot frame and its contents (kg)
%I_zframe = 10;% Moment of inertia around z-axis through frame CoM (kg*m^2)

% Wire Parameters
syms E A0
% TODO: Determine Youngs modulus for used cables
%E = 190; % Young's modulus for 304 steel (GPa)
% TODO: Determine crossectional area of wires
% For now estimated as 5mm diameter
%A0 = (5/2)^2*pi; % Crossectional area of wire (mm^2)

% Forces
F_gframe = m_frame * g;     % Force of gravity on the frame (N)
syms Fm1 Fm2 Fm3 Fm4 Fm5    % Motor forces (N)
%Fm1 = 1000; Fm2 = 1000; Fm3 = 1000; Fm4 = 1000; Fm5 = 1000;
F_motors = [Fm1; Fm2; Fm3; Fm4; Fm5];

% Robot generalised coordinates (currently assuming CoM is middle of frame)
syms qx qy qx_dot qy_dot thetaz omegaz
%qx = 10; qy = 10; thetaz = 0; omegaz = 0;
q = [qx, qy, 0];            % Position COM (m)
q_dot = [qx_dot, qy_dot, 0];% Velocity COM (m/s)
theta = [0, 0, thetaz];     % Orientation (rad)
omega = [0, 0, omegaz];     % Angular velocity (rad/s)

% Anchor positions in general coordinates
syms a1x a1y a1z % top left corner
syms a2x a2y a2z % top middle
syms a3x a3y a3z % top right corner
syms a4x a4y a4z % bottom right corner
syms a5x a5y a5z % bottom left corner
a1x = 0;    a1y = 20;   a1z = 0; % (m)
a2x = 10;   a2y = 20;   a2z = 0; % (m)
a3x = 20;   a3y = 20;   a3z = 0; % (m)
a4x = 20;   a4y = 0;    a4z = 0; % (m)
a5x = 0;    a5y = 0;    a5z = 0; % (m)
pos_anchors = [
[a1x, a1y, a1z]; 
[a2x, a2y, a2z]; 
[a3x, a3y, a3z]; 
[a4x, a4y, a4z]; 
[a5x, a5y, a5z];
];

% Motor positions relative to center of frame
syms d_framex d_framey d_framez % Frame dimensions (m)
%d_framex = 1; d_framey = 0.8; d_framez = 0.3;

pos_motors_local = [
[- d_framex/2*cos(theta(3)) - d_framey/2*sin(theta(3)), - d_framex/2*sin(theta(3)) + d_framey/2*cos(theta(3)), d_framez]; % top left corner
[                           - d_framey/2*sin(theta(3)),                              d_framey/2*cos(theta(3)), d_framez]; % top middle
[  d_framex/2*cos(theta(3)) - d_framey/2*sin(theta(3)),   d_framex/2*sin(theta(3)) + d_framey/2*cos(theta(3)), d_framez]; % top right corner
[  d_framex/2*cos(theta(3)) + d_framey/2*sin(theta(3)),   d_framex/2*sin(theta(3)) - d_framey/2*cos(theta(3)), d_framez]; % bottom right corner
[- d_framex/2*cos(theta(3)) + d_framey/2*sin(theta(3)), - d_framex/2*sin(theta(3)) - d_framey/2*cos(theta(3)), d_framez]; % bottom left corner
];
%% Inverse kinematics
% Vectors between motor positions to anchor points from frame position
wires = pos_anchors - repmat(q, 5, 1) + pos_motors_local;

%% Jacobian (Inverse method)
% TODO: Everything in the scrips is currently transposed
% To fix this all matricies and vectors need to have , swapped for ; and
% the same for ; to , also all calls for example wire_dir(1, :) needs to
% have parameters swapped to wire_dir(:, 1)
% This is a lot of work for comparatively no gain :(

J_inverse = [transpose([diff(wires(1,:),qx); diff(wires(1,:),qy); diff(wires(1,:),thetaz)]);
             transpose([diff(wires(2,:),qx); diff(wires(2,:),qy); diff(wires(2,:),thetaz)]);
             transpose([diff(wires(3,:),qx); diff(wires(3,:),qy); diff(wires(3,:),thetaz)]);
             transpose([diff(wires(4,:),qx); diff(wires(4,:),qy); diff(wires(4,:),thetaz)]);
             transpose([diff(wires(4,:),qx); diff(wires(5,:),qy); diff(wires(5,:),thetaz)]);];
J_inverse*[q_dot(1); q_dot(2); omega(3)];

%% Forward kinematics
% Wire directions (from IMU maybe) to center position of frame

% Wire direction of wires at two top corners
syms wire_dir_1_x wire_dir_1_y wire_dir_1_z wire_dir_3_x wire_dir_3_y wire_dir_3_z
wire_dir = [wire_dir_1_x, wire_dir_1_y, wire_dir_1_z;
            wire_dir_3_x, wire_dir_3_y, wire_dir_3_z];

frame_pose = forwardKinematics(wire_dir, pos_anchors, pos_motors_local);
%% Jacobian (Forward method) May not be very relevant
J_forward = [diff(frame_pose(1),wire_dir(1,1)), diff(frame_pose(1),wire_dir(1,2)), diff(frame_pose(1),wire_dir(1,3)), diff(frame_pose(1),wire_dir(2,1)), diff(frame_pose(1),wire_dir(2,2)), diff(frame_pose(1),wire_dir(2,3));
             diff(frame_pose(2),wire_dir(1,1)), diff(frame_pose(2),wire_dir(1,2)), diff(frame_pose(2),wire_dir(1,3)), diff(frame_pose(2),wire_dir(2,1)), diff(frame_pose(2),wire_dir(2,2)), diff(frame_pose(2),wire_dir(2,3));
             diff(frame_pose(3),wire_dir(1,1)), diff(frame_pose(3),wire_dir(1,2)), diff(frame_pose(3),wire_dir(1,3)), diff(frame_pose(3),wire_dir(2,1)), diff(frame_pose(3),wire_dir(2,2)), diff(frame_pose(3),wire_dir(2,3));
             diff(frame_pose(4),wire_dir(1,1)), diff(frame_pose(4),wire_dir(1,2)), diff(frame_pose(4),wire_dir(1,3)), diff(frame_pose(4),wire_dir(2,1)), diff(frame_pose(4),wire_dir(2,2)), diff(frame_pose(4),wire_dir(2,3))];

%% Potential energy
T = F_motors;   % Wire tensions is equivalent to the motor forces

% Calculate total potential energy of the wire system
U_wires = potentialEnergyWires(T, wires, E, A0);

% Calculate gravitational potential energy of body
q_g = [0, q(2), 0];
U_g = q_g * F_gframe;

% Calculate total potential energy of system
U_system = norm(U_wires + U_g);
%% Kinetic energy
% Case of general motion

% Calculate kinetic energy
K_frame_trans =  1/2 * m_frame * (q_dot(1)^2 + q_dot(2)^2);
K_frame_rot = 1/2 * I_zframe * omega(3)^2;
K_frame = K_frame_rot + K_frame_trans;

% Kinetic energy of the system
K_system = K_frame;
%% Langrangian
L = K_system - U_system;

%% Equations of motion
syms t
% Generalised coordinates
q_gen = [q(1:2), theta(3)];
q_dot_gen = [q_dot(1:2), omega(3)];

% Euler-Langrangian equations
for i = 1:3
    eqn(i) = diff(diff(L, q_dot_gen(i))) - diff(L, q_gen(i));
end
eqn;

%% Newton method
% Following example from book
% F = m*q_ddot

clear   % Clear workspace
clc     % Clear command window

% Parameters %
% Robot COM
syms qx qy qz qtheta
qx = 2.2929; qy = 2.2929; qz = 0;
qtheta = 0;
q = [qx; qy; qz];

% Inertial parameters
syms m % Robot mass
m = 10;

% Anchors
syms a1x a1y a1z % top left corner
syms a2x a2y a2z % top middle
syms a3x a3y a3z % top right corner
syms a4x a4y a4z % bottom right corner
syms a5x a5y a5z % bottom left corner
a1x = 0;    a1y = 10;   a1z = 0; % (m)
a2x = 5;    a2y = 10;   a2z = 0; % (m)
a3x = 10;   a3y = 10;   a3z = 0; % (m)
a4x = 10;   a4y = 0;    a4z = 0; % (m)
a5x = 0;    a5y = 0;    a5z = 0; % (m)
pos_anchors = [[a1x; a1y; a1z],[a2x; a2y; a2z],[a3x; a3y; a3z],[a4x; a4y; a4z],[a5x; a5y; a5z]];

% Motors
syms d_framex d_framey d_framez % Frame dimensions (m)
d_framex = 1; d_framey = 0.8; d_framez = 0.3;
pos_motors_local = [
[- d_framex/2*cos(qtheta) - d_framey/2*sin(qtheta), - d_framex/2*sin(qtheta) + d_framey/2*cos(qtheta), d_framez]; % top left corner
[                         - d_framey/2*sin(qtheta),                            d_framey/2*cos(qtheta), d_framez]; % top middle
[  d_framex/2*cos(qtheta) - d_framey/2*sin(qtheta),   d_framex/2*sin(qtheta) + d_framey/2*cos(qtheta), d_framez]; % top right corner
[  d_framex/2*cos(qtheta) + d_framey/2*sin(qtheta),   d_framex/2*sin(qtheta) - d_framey/2*cos(qtheta), d_framez]; % bottom right corner
[- d_framex/2*cos(qtheta) + d_framey/2*sin(qtheta), - d_framex/2*sin(qtheta) - d_framey/2*cos(qtheta), d_framez]; % bottom left corner
];
pos_motors_local = transpose(pos_motors_local);

% External forces
syms g           % Gravitational force magnitude
g = 9.82;
g_v = [0; -g; 0];% Gravity vector (-y = down)
F_g = m*g_v;     % Gravity force

%% Inverse kinematics (pos(q) -> wire lengths(rho))
pos_motors = repmat(q, 1, 5) + pos_motors_local
rho = sqrt(dot((pos_motors - pos_anchors), (pos_motors - pos_anchors))); % Wire lengths

% Wire directions
wdir = (pos_anchors - pos_motors)./rho;

%% Forward dynamics (forces -> acceleration)
syms Fm1 Fm2 Fm3 Fm4 Fm5    % Motor forces (N)
Fm1 = 298.2; Fm2 = 383.4; Fm3 = 298.2; Fm4 = 501.7; Fm5 = 501.7;
F_motors = [Fm1, Fm2, Fm3, Fm4, Fm5];

% Sum of all cable forces (negative F as forces pull not push)
F_cables = sum(F_motors .* wdir, 2);
F_norm = [0; 0; -F_cables(3, :)]; % Normal force from wall
q_ddot = (F_cables + F_g + F_norm)/m;

% Sum of all cable moments
M = cross(pos_motors_local, wdir .* F_motors);
Mz_sum = sum(M(3,:));
%% Inverse dynamics (acceleration -> forces)
syms qx_ddot qy_ddot qz_ddot
qx_ddot = 0; qy_ddot = 0; qz_ddot = 0;
q_ddot = [qx_ddot; qy_ddot; qz_ddot];

f_init = F_motors;

% Parameters for solve_forces
f_min = 100;    % Minimum wire force
f_max = 10000;  % Maximum wire force
stp_siz = 0.1;  % Step size for gradient descent
tol = 0.01;    % Accepted deviation from desired result
max_iter = 10000;% Maximum iterations of gradient descent

F = q_ddot*m + [0; 0; -100];
M = 0;

[f, F, Mz, E] = solve_forces(F, M, wdir, F_g, pos_motors_local, f_min, f_max, max_iter, stp_siz, tol);

if norm(E(1:2)) <= tol
    if abs(E(3)) <= tol
    display("Found suitable forces")
    display(f)
    display(F)
    display(Mz)
    else
        display("Did not converge")
        display(E)
    end
else
    display("Did not converge")
    display(E)
end

%% Functions
function [f_opt, F, Mz, Err] = solve_forces(F_desired, Mz_desired, dirWire, F_gravity, pos_motors_local, f_min, f_max, num_iter, step_size, tolerance)
% INPUTS:
% - dirWire: 5x3 matrix, where each row is a unit vector
% - F_gravity: 3x1 vector
% - pos_motors_local: 5x3 matrix, where each row is a moment arm vector
% - F_desired: 2x1 vector for the desired X and Y forces
% - Mz_desired: scalar for the desired moment around Z
% - f_min: scalar for the minimum value of f
% - f_max: scalar for the maximum value of f
% - num_iter: number of iterations to run gradient descent
% - step_size: step size (learning rate) for gradient descent
% - tolerance: tolerance for convergence
%
% OUTPUT:
% - f_opt: 1x5 vector of optimized force values

% Initialize f randomly within [f_min, f_max]
%f = f_min + (f_max - f_min) * rand(1, 5);
f = f_max;

% Calculate the gradients of the cost function with respect to each f value
dF_df = dirWire(1:2, :);
dM_df = cross(pos_motors_local, dirWire);

c = num_iter;

for i = 1:num_iter
    % Calculate the total force and moment using the current f values
    F = sum(f .* dirWire(1:2, :), 2) + F_gravity(1:2);
    M = sum(cross(pos_motors_local, dirWire .* f), 2);
    Mz = M(3);

    % Calculate error
    F_err = F - F_desired(1:2, :);
    Mz_err = Mz - Mz_desired;
    Err = [F_err; Mz_err];

    % Check for convergence
    if norm(F_err) < tolerance
        if abs(Mz_err) < tolerance
            display("here")
            c = i;
            break;
        end
    end

    % Update f values using gradient descent
    f = f - step_size * (F_err' * dF_df + Mz_err * dM_df(3, :));

    % Apply constraints to f values
    f = max(f_min, min(f_max, f));
end

% Return optimized f values
f_opt = f;
end

function [f, F, F_err, Mz_err] = solve_forces_OLD(dirW, pos_motors_local, F_desired, F_grav, f_init, f_min, f_max, Mz_desired, step_size, tolerance, max_iter)
    % Initialize force magnitudes
    %f = f_max * rand(1, 5);
    % Random initial points lead to some wild results (strange minima)
    f = f_init;

    % Iterate until convergence or max iterations
    for iter = 1:max_iter
        F = sum(f .* dirW, 2) + F_grav; % Compute force vector
        F_err = F - F_desired; % Compute difference from goal force

        M = sum(cross(pos_motors_local, dirW .* f), 2); % Compute moment vectors
        Mz_sum = sum(M(3,:)); % Compute sum of z moments
        Mz_err = Mz_sum - Mz_desired;

        % Check for convergence
        if norm(F_err) < tolerance
            break;
        end

        % Compute gradient
        FG = zeros(1, 5);
        for i = 1:5
            FG(i) = dirW(:,i)' * F_err;
        end
        %MG = sum(pos_motors_local .* repmat(dirW .* sign(Mz_err), 1, 3), 1)'

        % Update forces
        f = f - step_size * FG;

        % Enforce constraints
        for i = 1:5
            % Force range
            if f(i) < f_min
                f(i) = f_min;
            elseif f(i) > f_max
                f(i) = f_max;
            end
        end
    end
end

function coordinates = forwardKinematics(WireDirections, pos_anchors, pos_motors_local)
    % Arguments:
    %   WireDirections is 2 unit vectors from IMU (anchor to motor)
    %   MotorToWall is distance from motors to the wall in z direction
    
    % Determine cable vectors (known distance from motor to wall)
    wire_scaled = WireDirections.*pos_motors_local([1,3],3)./WireDirections(:, 3);

    % Determine motor points
    pos_motors = wire_scaled + pos_anchors([1,3], :);
    
    % Determine vector between motor points
    between_motors = pos_motors(1, :) - pos_motors(2, :);

    % Determine thetaz as angle of vector between motor points
    % TODO: This assumes the motors are both placed at corners
    % If this is not true the angle needs an offset
    theta = atan2(between_motors(1), between_motors(2));

    % Determine the coordinates of the center point
    syms thetaz
    frame_center = pos_motors(1, :) - subs(pos_motors_local(1, :), thetaz, theta);

    coordinates = [frame_center, theta];
end

function U_total = potentialEnergyWires(TensionForces, TensionedWireVectors, YoungsModulus, WireCrossectionalArea)
    % Function to determine the total potential energy of the wires
    % Arguments:
    %   Tension force (equal to the motor force ignoring friction)
    %   Tension vectors from motor point to anchor point

    % Modelled wires as spring (motor force to determine potential energy)
    % using Hookes law (k = T/L) we determine potential energy stored in wires
    % Potential energy is calculated as U = 1/2 * T * ΔL
    % Derived from the work done to change the length of wire
    % ΔL = T * L / (A0 * E) from stress/strain relatio
    % Direction T_dir is used to resolve the potential along xyz

    % Calculate length of wire
    % (redundant step if wire deflection is ignored)
    % If wire deflection is important this length should be a param
    L = sqrt(TensionedWireVectors(:, 1).^2 + TensionedWireVectors(:, 2).^2 + TensionedWireVectors(:, 3).^2);

    % Normalise tension vector for force direction
    T_dir = TensionedWireVectors .* 1./L;

    % Calculate potential energy of each wire
    U = TensionForces.^2 .* L .* -T_dir / (2 * WireCrossectionalArea/1e6 * YoungsModulus*1e9);

    % Sum the potential energy of all the wires
    U_total = sum(U, 1);
end