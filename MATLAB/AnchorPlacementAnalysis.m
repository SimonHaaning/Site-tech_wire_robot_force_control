clear     % Clear workspace
clc       % Clear command window
close all % Close all figures

%% System parameters
% Anchor positions static frame (m)
%a1 = [  0; 4; 0];
%a2 = [  2; 5; 0];
%a3 = [  4; 4; 0];
%a4 = [  4; 0; 0];
%a5 = [  0; 0; 0];
%anchors_pos = [a1, a2, a3, a4, a5];

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

%% Workspace parameters
% CoM limits
WS_lims = [0.5, 3.5;  % X limits
           0.5, 3.5]; % Y limits

% Anchor maximums
