import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as colors
from Python_Functions.KinematicsFunctions import inverse_wire_kinematics

# Anchor positions static frame (m)
a1 = [0.0, 3.355, -0.20646]
a2 = [1.45, 3.355, -0.20646]
a3 = [2.9, 3.355, -0.20646]
a4 = [2.9, 0.0, -0.20646]
a5 = [0.0, 0.0, -0.20646]
anchors_pos = np.array([a1, a2, a3, a4, a5], np.float64)  # 5x3 matrix

# Motor positions relative to CoM (m)
m1 = [-0.36785, 0.25047,  0.00996]
m2 = [0.00065,  0.25047,  0.00996]
m3 = [0.41995,  0.16827,  0.00996]
m4 = [0.33960,  -0.31953, 0.00996]
m5 = [-0.45005, -0.23873, 0.00996]
motors_pos = np.array([m1, m2, m3, m4, m5], np.float64)  # 5x3 matrix
ANGLE_OFFSET = -0.117
r_frame = np.array([[np.cos(ANGLE_OFFSET), -np.sin(ANGLE_OFFSET), 0],
                    [np.sin(ANGLE_OFFSET), np.cos(ANGLE_OFFSET), 0],
                    [0, 0, 0]])
motors_pos = np.matmul(motors_pos, r_frame)

# Inputs
safety_limit = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])  # np.array([[0.5, 0.5, 0.0], [0.5, 0.5, 0.0]])
lower_limits = np.subtract(a5, m5) + safety_limit[0]
upper_limits = np.subtract(a3, m3) - safety_limit[1] - np.array([0, 0.0, 0])
print(f"Workspace:\nWidth(X):\t{upper_limits[0]-lower_limits[0]}\nHeight(Y):\t{upper_limits[1]-lower_limits[1]}")

x_res = 100
y_res = 100
x = np.linspace(lower_limits[0], upper_limits[0], x_res)
y = np.linspace(lower_limits[1], upper_limits[1], y_res)
xx, yy = np.meshgrid(x, y)
workspace = np.stack((xx, yy), axis=-1).reshape(-1, 2).transpose()  # All coordinates in workspace

# Shape the array for kinematics (SHOULD BE FIXED TO DO THIS AUTOMATICALLY)
com_position = np.concatenate((workspace[:2, :], np.zeros_like(workspace[0, :][np.newaxis, :])), axis=0).transpose()
angles = np.zeros_like(workspace[0, :])

# Determine wire directions along trajectory
wire_dir, wire_len, moment_arms = inverse_wire_kinematics(com_position, angles, anchors_pos, motors_pos)

# Make mask where wires point in the wrong way
mask = None
np.where(wire_dir[:, 0, 1])

data1 = np.max(wire_dir[:, :, 2], axis=1).reshape(x_res, y_res)
# norm = colors.LogNorm(vmin=500, vmax=10000)
masked_data = np.ma.masked_array(data1, mask)
plt.pcolormesh(xx, yy, masked_data, norm=norm, cmap='jet')
plt.colorbar()
plt.xlabel('CoM X coordinate')
plt.ylabel('CoM Y coordinate')
plt.xlim((np.minimum(anchors_pos[:, 0]), np.maximum(anchors_pos[:, 0])))
plt.ylim((np.minimum(anchors_pos[:, 2]), np.maximum(anchors_pos[:, 2])))
plt.savefig(f'WireZComponentInWorkspace.pdf', format='pdf', bbox_inches='tight')
plt.show()
