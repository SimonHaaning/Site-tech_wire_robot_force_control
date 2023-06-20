import numpy as np
from Python_Functions.KinematicsFunctions import inverse_wire_kinematics
from Python_Functions.DynamicsFunctions import forward_wire_dynamics
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm
from tqdm import tqdm

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

# Position in the workspace
com_pos = np.array([[2, 3, 0]], np.float64)  # 3x1 vector
angle = np.array([0.0])

# Compute forces
m_frame = 300  # kg
I_frame = 101.85669  # kg*m^2
F_g = np.array([0, -9.82*m_frame, 0])  # force from gravity

# Calculate kinematics
wire_dir, _, moment_arms = inverse_wire_kinematics(com_pos, angle, anchors_pos, motors_pos)

# Randomly sample coordinates
# Set the random seed for reproducibility
np.random.seed(42)

# Generate a matrix of random elements with shape (10000, 5, 1)
random_forces = np.random.rand(10000, 5, 1) * 10000
#random_forces = np.sort(random_forces, axis=0)

# Attempt random solutions
wire_forces_sum, wire_moments_sum = forward_wire_dynamics(random_forces, wire_dir, moment_arms)
print(wire_forces_sum.shape)

err_x = abs(np.zeros_like(wire_forces_sum[:, 0]) - wire_forces_sum[:, 0])
err_y = abs(np.zeros_like(wire_forces_sum[:, 1]) - wire_forces_sum[:, 1])
err_z = abs(np.zeros_like(wire_moments_sum[:]) - wire_moments_sum[:])

fig, axs = plt.subplots(1, 3, figsize=(10, 5))

sort_idx_x = np.argsort(err_x)  # Sort the points based on their error values
random_forces_sorted_x = random_forces[sort_idx_x]
s0 = axs[0].scatter(random_forces_sorted_x[:, 0, 0], random_forces_sorted_x[:, 4, 0], c=err_x[sort_idx_x][::-1], norm=LogNorm())
axs[0].set_title('X')

sort_idx_y = np.argsort(err_y)  # Sort the points based on their error values
random_forces_sorted_y = random_forces[sort_idx_y]
s1 = axs[1].scatter(random_forces_sorted_y[:, 0, 0], random_forces_sorted_y[:, 4, 0], c=err_y[sort_idx_y][::-1], norm=LogNorm())
axs[1].set_title('Y')

sort_idx_z = np.argsort(err_z)  # Sort the points based on their error values
random_forces_sorted_z = random_forces[sort_idx_z]
s2 = axs[2].scatter(random_forces_sorted_z[:, 0, 0], random_forces_sorted_z[:, 4, 0], c=err_z[sort_idx_z][::-1], norm=LogNorm())
axs[2].set_title('Z')

fig.colorbar(s2, ax=axs.ravel().tolist())
plt.show()

#forces = np.zeros((1, 5, 1))
#for i in tqdm(range(len(f)), position=4):
#    for j in tqdm(range(len(f)), position=3):
#        for k in tqdm(range(len(f)), position=2):
#            for l in tqdm(range(len(f)), position=1):
#                for m in tqdm(range(len(f)), position=0):
#                    forces[0, :, 0] = np.array([f[i], f[j], f[k], f[l], f[m]])
#                    wire_forces_sum, wire_moments_sum = forward_wire_dynamics(forces, wire_dir, moment_arms)
#                    #print(wire_forces_sum.shape)
