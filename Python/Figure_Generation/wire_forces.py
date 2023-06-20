import numpy as np
from Python_Functions.PlotterFunctions import draw_robot_2d
from Python_Functions.KinematicsFunctions import inverse_wire_kinematics
from Python_Functions.DynamicsFunctions import inverse_wire_dynamics
import matplotlib.pyplot as plt

# Anchor positions static frame (m)
a1 = [0.0, 4.0, 0.0]
a2 = [2.0, 4.0, 0.0]
a3 = [4.0, 4.0, 0.0]
a4 = [4.0, 0.0, 0.0]
a5 = [0.0, 0.0, 0.0]
anchor_positions = np.array([a1, a2, a3, a4, a5], np.float64)  # 5x3 matrix

# Motor positions relative to CoM (m)
m1 = [-0.50, 0.4,  0.3]
m2 = [-0.00,  0.4,  0.3]
m3 = [0.50,  0.4,  0.3]
m4 = [0.50,  -0.4, 0.3]
m5 = [-0.50, -0.4, 0.3]
motors_pos = np.array([m1, m2, m3, m4, m5], np.float64)  # 5x3 matrix

com_position = np.array([[2.2, 1.4, 0]], np.float64)  # 3x1 vector
frame_angle = np.array([0.0], np.float64)
acceleration = np.array([[4, 4, 0]]).transpose()

# Compute forces
m_frame = 100  # kg
I_frame = 101.85669  # kg*m^2
F_g = np.array([0, -9.82*m_frame, 0])  # force from gravity
frame_forces_motion = np.multiply(np.array([m_frame, m_frame, I_frame])[:, np.newaxis], acceleration).transpose()
frame_forces_all = np.subtract(frame_forces_motion, F_g)

# Determine wire directions along trajectory
wire_dir, wire_len, moment_arms = inverse_wire_kinematics(com_position, frame_angle, anchor_positions, motors_pos)

# Determine wire forces
motor_forces, combined_forces, combined_moments, iterations, _, _ = inverse_wire_dynamics(frame_forces_all, wire_dir, moment_arms)
print(f"Iterations:\n{iterations}\n")

# create a new figure instance
fig, ax = plt.subplots()

# Compute motor positions
r_frame = np.array([[np.cos(frame_angle[0]), -np.sin(frame_angle[0]), 0], [np.sin(frame_angle[0]), np.cos(frame_angle[0]), 0], [0, 0, 0]])
rotated_motors_pos = np.matmul(motors_pos, r_frame) + com_position[0]

# Compute wires
wires = anchor_positions - rotated_motors_pos

# Draw frame
plt.fill(rotated_motors_pos[:, 0], rotated_motors_pos[:, 1], color='k')
plt.text(com_position[0, 0], com_position[0, 1] + 0.11, 'CoM', fontsize=10, color='g', va='center', ha='center')
plt.scatter(com_position[0, 0], com_position[0, 1], color='g', marker='x')

# Draw motors and wires
plt.plot([rotated_motors_pos[:, 0], anchor_positions[:, 0]], [rotated_motors_pos[:, 1], anchor_positions[:, 1]], color='r', zorder=-1)
plt.scatter(anchor_positions[:, 0], anchor_positions[:, 1], color='r', s=15)
plt.scatter(rotated_motors_pos[:, 0], rotated_motors_pos[:, 1], color='r', s=15)

# Draw forces
force_scale = 900
forces = np.round(motor_forces[0].transpose()[0], 3)
plt.quiver(com_position[0, 0], com_position[0, 1], frame_forces_motion[0, 0]/force_scale, frame_forces_motion[0, 1]/force_scale, color='c', angles='xy', scale_units='xy', scale=1)
plt.quiver(com_position[0, 0], com_position[0, 1], F_g[0]/force_scale, F_g[1]/force_scale, color='y', angles='xy', scale_units='xy', scale=1)

# Add text to vectors
for i, (x, y) in enumerate(zip(rotated_motors_pos[:, 0], rotated_motors_pos[:, 1])):
    mid_x, mid_y = (x + anchor_positions[i, 0]) / 2, (y + anchor_positions[i, 1]) / 2
    plt.scatter(mid_x, mid_y, color='w', s=150, zorder=100)
    plt.text(mid_x, mid_y - 0.01, str(i + 1), fontsize=10, va='center', ha='center', zorder=101)

# Adjust axes
plt.xlim(np.min(anchor_positions[:, 0]) - 0.5, np.max(anchor_positions[:, 0]) + 0.5)
plt.ylim(np.min(anchor_positions[:, 1]) - 0.5, np.max(anchor_positions[:, 1]) + 0.5)
plt.xlabel('X coordinate (m)')
plt.ylabel('Y coordinate (m)')
plt.gca().set_aspect('equal', adjustable='box')

norms = np.linalg.norm(wires, 2, axis=1)
wire_directions = np.divide(wires, norms[:, np.newaxis])

for i in range(len(forces)):
    plt.quiver(rotated_motors_pos[i, 0], rotated_motors_pos[i, 1], wire_directions[i, 0]*forces[i]/force_scale, wire_directions[i, 1]*forces[i]/force_scale, color='y', angles='xy', scale_units='xy', scale=1)

plt.show()
fig.savefig('wire_forces.pdf', format='pdf', bbox_inches='tight')
