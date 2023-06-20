measured_gravity = [0.0; 0.0; 0.0];%[0.57735; 0.57735; 0.57735];
actual_gravity = [0; -1; 0];

measured_wire_direction = [1; 0; 0];

% Step 1: Calculate rotation axis
rotation_axis = cross(measured_gravity, actual_gravity);
rotation_axis = rotation_axis / norm(rotation_axis); % Normalize the rotation axis

% Step 2: Calculate rotation angle
cos_theta = dot(measured_gravity, actual_gravity);
sin_theta = norm(rotation_axis); % Since gravity vectors are normalized, sin(theta) = norm(rotation_axis)
rotation_angle = atan2(sin_theta, cos_theta);

% Step 3: Construct rotation matrix
I = eye(3); % Identity matrix
K = [0, -rotation_axis(3), rotation_axis(2);
     rotation_axis(3), 0, -rotation_axis(1);
     -rotation_axis(2), rotation_axis(1), 0];
rotation_matrix = I + sin(rotation_angle) * K + (1 - cos(rotation_angle)) * K^2;

% Step 4: Transform measured_wire_direction
actual_wire_direction = rotation_matrix * measured_wire_direction

f_dir = [-1, 1, 1, 1, -1];
syms f1 f2 f3 f4 f5
f = [f1, f2, f3, f4, f5];
F = sum(f .* f_dir)
g = gradient(sum(f .* f_dir))

syms mx my mz ux uy uz
moment_arm = [mx, my, mz];
f_dir = [ux, uy, uz];
M = cross(moment_arm, f1 * f_dir);
g2 = gradient(M(3))


% Known anchor points
P1 = [0, 2];  % Replace with the coordinates of known point 1
P2 = [2, 2];  % Replace with the coordinates of known point 2

% Unit vectors from corners A and B to known points
V1 = [-0, 1]/norm([-0, 1]); % P1 / norm(P1);  % Normalize vector V1
V2 = [2, 1]/norm([2, 1]); % P2 / norm(P2);  % Normalize vector V2

% Midpoint between the known anchor points
M = (P1 + P2) / 2;

% Scaling factors
s1 = norm(P1 - P2) / 2;  % Distance between the corners and midpoint
s2 = s1;  % Assuming equal scaling factors for simplicity

% Calculate the frame corner positions
CornerA = M - s1 * V1;
CornerB = M + s2 * V2;

% Display the results
disp('Corner A:')
disp(CornerA)
disp('Corner B:')
disp(CornerB)

deg2rad(20.9)
rad2deg(atan2(0.93890715, -0.34417057))-90

%% attempt 2
clear
clc

%measured_gravity = [0.072414; 0.010100; 0.997244]; % Lying flat on table
%measured_gravity = [0.997244; 0.010100; 0.072414]; % Pointing straight up
%measured_gravity = [0.635593; 0.070714; 0.768669]; % 45 ish degrees
measured_gravity_w1 = [0.812318; -0.460643; 0.357748]; %-58 grader/-1.012 rad
measured_gravity_w2 = [0.823835; 0.535035; 0.186868]; %56 grader/0.977 rad
actual_gravity = [0; 1; 0];  % should maybe not be negative it measures +1 Z when Z points up
measured_wire_direction = [1; 0; 0];

rad2deg(atan2(measured_gravity_w1(1), measured_gravity_w1(2)))-180
rad2deg(atan2(measured_gravity_w2(1), measured_gravity_w2(2)))

actual_wire_direction_w1 = transformWireDirection2D(measured_gravity_w1, actual_gravity)
actual_wire_direction_w2 = transformWireDirection2D(measured_gravity_w2, actual_gravity)

% P_unknown = anchor1 - offset1 - actual_wire_direction_w1 * unknown_scalar1;
% P_unknown = anchor2 - offset2 - actual_wire_direction_w2 * unknown_scalar2;
anchor1 = [0.0; 3.355];%, -0.20646]
anchor2 = [2.9; 3.355];%, -0.20646]
offset1 = [-0.36785; 0.25047];%, -0.00996]
offset2 = [0.41995; 0.16827];%, -0.00996]

% Define the two vector lines
p1 = anchor1 - offset1;  % Origin point of the first line
v1 = actual_wire_direction_w1;  % Direction vector of the first line
p2 = anchor2 - offset2;  % Origin point of the second line
v2 = actual_wire_direction_w2;  % Direction vector of the second line

% Find the determinant of the system of equations
det_A = det([v1, -v2])

% Check if the lines intersect
if det_A ~= 0
    % Solve for the intersection point
    t = det([p2 - p1, -v2]) / det_A
    intersection_point = p1 + t*v1;
    % The intersection point is a column vector [x; y]
    % Display the result
    disp(['The two lines intersect at point (' num2str(intersection_point(1)) ', ' num2str(intersection_point(2)) ')']);
else
    % The lines are parallel and do not intersect
    disp('The two lines are parallel and do not intersect.');
end

%% Functions
function wire_direction_static_frame = transformWireDirection(measured_gravity, actual_gravity)
    wire_direction_dynamic_frame = [1; 0; 0];

    % Normalize the input vectors
    measured_gravity = measured_gravity / norm(measured_gravity);
    actual_gravity = actual_gravity / norm(actual_gravity);

    % Calculate the dot product
    d = dot(measured_gravity, actual_gravity);

    % Calculate the cross product
    c = cross(measured_gravity, actual_gravity)

    % Check if the vectors are parallel
    c_squared = norm(c)^2;
    if c_squared == 0
        error('Vectors are parallel. Cannot determine transformation.');
    end

    % Construct the skew-symmetric cross-product matrix
    M = [0, -c(3), c(2);
         c(3), 0, -c(1);
         -c(2), c(1), 0];

    % Calculate the transformation matrix
    T = eye(3) + M + (1 / (1 + d)) * (M^2);

    % Tranform the wire direction into static frame
    wire_direction_static_frame = T * wire_direction_dynamic_frame;
end

function wire_direction_static_frame = transformWireDirection2D(measured_gravity, actual_gravity)
    wire_direction_dynamic_frame = [1; 0];
    measured_gravity = [measured_gravity(1); measured_gravity(2); 0];

    % Normalize the input vectors
    measured_gravity = measured_gravity / norm(measured_gravity);
    actual_gravity = actual_gravity / norm(actual_gravity);

    % Calculate the cross product
    c = cross(measured_gravity, actual_gravity);

    % Calculate the dot product
    d = dot(measured_gravity, actual_gravity);

    % Check if the vectors are parallel
    c_squared = norm(c)^2;
    if c_squared == 0
        error('Vectors are parallel. Cannot determine transformation.');
    end

    % Calculate the transformation angle
    theta = atan2(c(3), d);

    % Calculate the transformation matrix
    T = [cos(theta), -sin(theta);
         sin(theta), cos(theta)];

    wire_direction_static_frame = T * wire_direction_dynamic_frame;
end