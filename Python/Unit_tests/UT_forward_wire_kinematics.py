import numpy as np
from Python_Functions.KinematicsFunctions import forward_wire_kinematics, inverse_wire_kinematics

# Anchor positions static frame (m)
a1 = [0.0, 4.0, 0.0]
a2 = [2.0, 5.0, 0.0]
a3 = [4.0, 4.0, 0.0]
a4 = [4.0, 0.0, 0.0]
a5 = [0.0, 0.0, 0.0]
anchors_pos = np.array([a1, a2, a3, a4, a5], np.float64)  # 5x3 matrix

# Motor positions relative to CoM (m)
m1 = [-0.5, 0.4, 0.3]
m2 = [0.0, 0.4, 0.3]
m3 = [0.5, 0.4, 0.3]
m4 = [0.5, -0.4, 0.3]
m5 = [-0.5, -0.4, 0.3]
motors_pos = np.array([m1, m2, m3, m4, m5], np.float64)  # 5x3 matrix

com_pos = np.array([0.0, 0.0, 0], np.float64)  # 3x1 vector
angle = 0.0

com_pos = np.array([com_pos] * 1)
angle = np.array([angle] * 1)

wire_directions, wire_len, _ = inverse_wire_kinematics(com_pos, angle, anchors_pos, motors_pos)

# Wire directions (sensor readings from wire 1 and 3)
#wire_directions = np.array([[-0.67763093, 0.72280632, -0.13552619],
#                            [0.67763093, 0.72280632, -0.13552619]], np.float64)
#wire_directions = np.array([wire_directions, wire_directions, wire_directions, wire_directions])

com_pos, angle = forward_wire_kinematics(wire_directions[:, [0, 2]], anchors_pos, motors_pos)
print(f"CoM position:\n{com_pos}")
print(f"angle:\n{angle}")
