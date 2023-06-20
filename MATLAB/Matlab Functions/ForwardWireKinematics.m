function [CoM_position, frame_angle]  = ForwardWireKinematics(wire1_direction, wire3_direction, anchor_positions, motors_pos_local)
% INPUTS:
% - wire1_direction: 3x1 vector, unit vector for wire 1 direction (IMU)
% - wire3_direction: 3x1 vector, unit vector for wire 3 direction (IMU)
% - anchor_positions: 3x5 matrix, anchor positions
% - motors_pos_local: 3x5 matrix, motor positions relative to CoM
%
% OUTPUT:
% - CoM_position: 3x1 vector, location of the frame CoM
% - frame_angle: scalar, frame angle around Z
%
% NOTE: Assumed wires do not deflect

% Determine wire vectors (known distance from motor to wall)
wire_directions = [wire1_direction, wire3_direction];
wire_scalars = [motors_pos_local(3, 1), motors_pos_local(3, 3)]./wire_directions(3, :);
wire_vectors = -wire_directions.*wire_scalars;

% Determine motor points
pos_motors = anchor_positions(:, [1,3]) - wire_vectors;

% Determine vector between motor points (motor1 -> 3)
between_motors = pos_motors(:, 2) - pos_motors(:, 1);

% Determine thetaz as angle of vector between motor points
% NOTE: This assumes the motors are both placed at corners
% If this is not true the angle needs an offset
frame_angle = atan2(between_motors(2), between_motors(1));

% Rotation matrix for frame orientation
R_frame = [cos(frame_angle), -sin(frame_angle), 0; 
           sin(frame_angle),  cos(frame_angle), 0; 
                          0,                 0, 1];

% Determine the CoM pose, by using motor1 offset
CoM_position = pos_motors(:, 1) - R_frame * motors_pos_local(:, 1);
end