import numpy as np
from Python_Functions.KinematicsFunctions import inverse_wire_kinematics
from Python_Functions.DynamicsFunctions import inverse_wire_dynamics, forward_wire_dynamics
from Python_Functions.TrajectoryGenerationFunctions import trajectory_generation
import matplotlib.pyplot as plt
import matplotlib.colors as colors
import time

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
ANGLE_OFFSET = -0.117#-0.16
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
workspace = np.stack((xx, yy), axis=-1).reshape(-1,2).transpose()  # All coordinates in workspace

#final_state = np.array([[1, 0, 0],
#                        [1, 0, 0],
#                        [0, 0, 0]], np.float64)
#start_state = np.array([[3.2, 0, 0],
#                        [3.2, 0, 0],
#                        [0, 0, 0]], np.float64)
#acceleration_limits = np.array([0.025, 0.025, 0.001])
#time_resolution = 100

# Compute trajectory
#position, velocity, acceleration, times = trajectory_generation(start_state, final_state, time_resolution, acceleration_limits)

# Compute forces
m_frame = 59.5  # kg
I_frame = 12.5369  # kg*m^2
F_g = np.array([0, -9.82*m_frame, 0])  # force from gravity
F_adjust = np.array([0.0*m_frame, 0.0*m_frame, 0])
frame_forces_motion = np.multiply(np.array([m_frame, m_frame, I_frame])[:, np.newaxis], np.zeros((3, x_res*y_res))).transpose()
frame_forces_all = np.subtract(frame_forces_motion, np.subtract(F_g, F_adjust))

# Shape the array for kinematics (SHOULD BE FIXED TO DO THIS AUTOMATICALLY)
com_position = np.concatenate((workspace[:2, :], np.zeros_like(workspace[0, :][np.newaxis, :])), axis=0).transpose()
angles = np.zeros_like(workspace[0, :])

# Determine wire directions along trajectory
wire_dir, wire_len, moment_arms = inverse_wire_kinematics(com_position, angles, anchors_pos, motors_pos)

# Determine wire forces
before = time.time()
motor_forces, F, _, iterations, err, err_array = inverse_wire_dynamics(frame_forces_all, wire_dir, moment_arms, max_iter=10000, wire_forces_max=10000, wire_forces_min=500, initial_wire_force=500)
after = time.time()
print(f"Iterations:\n{iterations}")
print(after-before)

# Create mask
err2 = np.max(np.abs(err), axis=1).reshape(x_res, y_res)
mask = np.where(err2 <= 100, 0, 1)

#data = np.min(motor_forces, axis=1).reshape(x_res, y_res)
data = np.max(motor_forces, axis=1).reshape(x_res, y_res)
norm = colors.LogNorm(vmin=500, vmax=10000)
masked_data = np.ma.masked_array(data, mask)
plt.pcolormesh(xx, yy, masked_data, norm=norm, cmap='jet')
plt.colorbar()
plt.xlabel('CoM X coordinate')
plt.ylabel('CoM Y coordinate')
plt.xlim((0, 2.9))
plt.ylim((0, 3.855))
#plt.savefig('MinimumWireForceMaxInit.pdf', format='pdf', bbox_inches='tight')
#plt.savefig('MaximumWireForceMinInit.pdf', format='pdf', bbox_inches='tight')
plt.savefig('NotMovedCentralAnchorUp.pdf', format='pdf', bbox_inches='tight')
plt.show()

# Force against wall
data = np.where(np.abs(F[:, 2])<=9000, np.abs(F[:, 2]), np.nan).reshape(x_res, y_res)
masked_data = np.ma.masked_array(data, mask)
plt.pcolormesh(xx, yy, masked_data, cmap='jet')
plt.colorbar()
plt.xlabel('CoM X coordinate')
plt.ylabel('CoM Y coordinate')
plt.xlim((0, 2.9))
plt.ylim((0, 3.355))
plt.savefig('ForceAgainstWall.pdf', format='pdf', bbox_inches='tight')
plt.show()

fig, axs = plt.subplots(1, 3)
im1 = axs[0].imshow(np.flip(np.abs(err).reshape(x_res, y_res, 3)[:, :, 0], axis=0))
axs[0].set_title('X')
im2 = axs[1].imshow(np.flip(np.abs(err).reshape(x_res, y_res, 3)[:, :, 1], axis=0))
axs[1].set_title('Y')
im3 = axs[2].imshow(np.flip(np.abs(err).reshape(x_res, y_res, 3)[:, :, 2], axis=0))
axs[2].set_title('Z')
fig.colorbar(im1, ax=axs.ravel().tolist())
plt.show()

