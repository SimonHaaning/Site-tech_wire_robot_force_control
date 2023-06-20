function PlotRobot(CoM_pos, frame_ang, anchors_pos, motors_pos, color)

% Determine motor positions
R_frame = [cos(frame_ang), -sin(frame_ang), 0; 
           sin(frame_ang),  cos(frame_ang), 0; 
                        0,               0, 1];
motors_pos_world = CoM_pos + R_frame*motors_pos;

% Plot frame
plot(motors_pos_world(1,[1,2,3,4,5,1]), motors_pos_world(2,[1,2,3,4,5,1]), color);
for i = 1:5
    % Plot wires
    plot([anchors_pos(1,i), motors_pos_world(1,i)], [anchors_pos(2,i), motors_pos_world(2,i)], color, LineStyle="--")
end
end