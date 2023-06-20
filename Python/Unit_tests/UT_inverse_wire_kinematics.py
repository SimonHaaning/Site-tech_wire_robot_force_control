import numpy as np
from Python_Functions.KinematicsFunctions import inverse_wire_kinematics

# Anchor positions static frame (m)
a1 = [0.0, 4.0, 0.0]
a2 = [2.0, 5.0, 0.0]
a3 = [4.0, 4.0, 0.0]
a4 = [4.0, 0.0, 0.0]
a5 = [0.0, 0.0, 0.0]
anchors_pos = np.array([a1, a2, a3, a4, a5], np.float64)  # 5x3 matrix

# Motor positions relative to CoM (m)
m1 = [-0.5, 0.4,  0.3]
m2 = [0.0,  0.4,  0.3]
m3 = [0.5,  0.4,  0.3]
m4 = [0.5,  -0.4, 0.3]
m5 = [-0.5, -0.4, 0.3]
motors_pos = np.array([m1, m2, m3, m4, m5], np.float64)  # 5x3 matrix

com_pos = np.array([2, 2, 0], np.float64)  # 3x1 vector
angle = 0.0

com_pos = np.array([com_pos] * 1000)
angle = np.array([angle] * 1000)
wire_dir, wire_len, _ = inverse_wire_kinematics(com_pos, angle, anchors_pos, motors_pos)  # 502ms to solve invkin for 1e6 points
print(f"Wire directions:\n{wire_dir}\n\nWire lengths:\n{wire_len}")
