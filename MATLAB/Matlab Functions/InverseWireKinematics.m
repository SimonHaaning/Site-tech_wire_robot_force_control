function [wire_directions, wire_lengths] = InverseWireKinematics(CoM_position, frame_angle, anchor_positions, motors_pos_local)
% INPUTS:
% - CoM_position: 3x1 vector, location of the frame CoM
% - frame_angle: scalar, frame angle around Z
% - anchor_positions: 3x5 matrix, anchor positions
% - motors_pos_local: 3x5 matrix, motor positions relative to CoM
%
% OUTPUT:
% - wire_directions: 3x5 matrix, columms are unit vectors for wire directions
% - wire_lengths: 1x5 vector, wire lengths
%
% NOTE: Assumed wires do not deflect

% Rotation matrix for frame orientation
R_frame = [cos(frame_angle), -sin(frame_angle), 0; 
           sin(frame_angle),  cos(frame_angle), 0; 
                          0,                 0, 1];

% Calculate position of motors in global frame
motor_positions = repmat(CoM_position, 1, 5) + R_frame*motors_pos_local;

% Calculate wire vectors
wire_vectors = anchor_positions - motor_positions;

% Calculate wire lengths
wire_lengths = sqrt(dot(wire_vectors, wire_vectors));

% Wire directions
wire_directions = wire_vectors./wire_lengths;
end