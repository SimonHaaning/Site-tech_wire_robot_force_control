import numpy as np
from Python_Functions.PlotterFunctions import draw_robot_2d
import matplotlib.pyplot as plt

# Anchor positions static frame (m)
a1 = [0.0, 4.0, 0.0]
a2 = [2.0, 5.0, 0.0]
a3 = [4.0, 4.0, 0.0]
a4 = [4.0, 0.0, 0.0]
a5 = [0.0, 0.0, 0.0]
anchor_positions = np.array([a1, a2, a3, a4, a5], np.float64)  # 5x3 matrix

# Motor positions relative to CoM (m)
m1 = [-0.5, 0.4,  0.3]
m2 = [0.0,  0.4,  0.3]
m3 = [0.5,  0.4,  0.3]
m4 = [0.5,  -0.4, 0.3]
m5 = [-0.5, -0.4, 0.3]
motors_pos = np.array([m1, m2, m3, m4, m5], np.float64)  # 5x3 matrix

com_position = np.array([1, 2, 0], np.float64)  # 3x1 vector
frame_angle = 0.0

fig = draw_robot_2d(com_position, frame_angle, anchor_positions, motors_pos)
fig.savefig('wire_directions.pdf', format='pdf', bbox_inches='tight')
