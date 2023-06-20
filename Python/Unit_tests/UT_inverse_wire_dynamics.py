import numpy as np
from Python_Functions.KinematicsFunctions import inverse_wire_kinematics
from Python_Functions.DynamicsFunctions import inverse_wire_dynamics
from Python_Functions.TrajectoryGenerationFunctions import trajectory_generation
from Python_Functions.PlotterFunctions import motor_force_plotter, trajectory_plotter, wire_vel_plotter
import matplotlib.pyplot as plt

# Anchor positions static frame (m)
a1 = [0.0, 3.355, -0.20646]
a2 = [1.45, 3.355, -0.20646]
a3 = [3.08, 3.355, -0.20646]
a4 = [2.9, 0.0, -0.20646]
a5 = [0.0, 0.0, -0.20646]
ANCHORS_POS = np.array([a1, a2, a3, a4, a5])  # 5x3 matrix

# Motor positions relative to CoM (m)
m1 = [-0.36739, 0.26598, -0.00939]
m2 = [0.03239, 0.26634, -0.00939]
m3 = [0.43927, 0.16520, -0.00939]
m4 = [0.33961, -0.34145, -0.00939]
m5 = [-0.46848, -0.24180, -0.00939]
MOTORS_POS = np.array([m1, m2, m3, m4, m5])  # 5x3 matrix
ANGLE_OFFSET = -0.1#741  # Frame angle offset measured by IMU
# Rotate motor positions
r_frame = np.array([[np.cos(ANGLE_OFFSET), -np.sin(ANGLE_OFFSET), 0],
                    [np.sin(ANGLE_OFFSET), np.cos(ANGLE_OFFSET), 0],
                    [0, 0, 0]])
MOTORS_POS_ROTATED = np.matmul(MOTORS_POS, r_frame)

# Inputs
start_state = np.array([[1, 0, 0],
                        [1, 0, 0],
                        [0, 0, 0]], np.float64)
final_state = np.array([[2, 0, 0],
                        [2, 0, 0],
                        [0, 0, 0]], np.float64)
ACCELERATION_LIMITS = np.array([0.01, 0.01, 0.001])
time_resolution = 100

# Compute trajectory
position, velocity, acceleration, times = trajectory_generation(start_state, final_state, time_resolution, ACCELERATION_LIMITS)
#trajectory_plotter(position, velocity, acceleration, times)

# Compute forces
m_frame = 300  # kg
I_frame = 101.85669  # kg*m^2
F_g = np.array([0, -9.82*m_frame, 0])  # force from gravity
frame_forces_motion = np.multiply(np.array([m_frame, m_frame, I_frame])[:, np.newaxis], acceleration).transpose()
frame_forces_all = np.subtract(frame_forces_motion, F_g)

# Shape the array for kinematics (SHOULD BE FIXED TO DO THIS AUTOMATICALLY)
com_position = np.concatenate((position[:2, :], np.zeros_like(position[0, :][np.newaxis, :])), axis=0).transpose()
angles = position[2, :]

# Determine wire directions along trajectory
wire_dir, wire_len, moment_arms = inverse_wire_kinematics(com_position, angles, ANCHORS_POS, MOTORS_POS_ROTATED)

# Determine wire forces
motor_forces, _, _, iterations, _, _ = inverse_wire_dynamics(frame_forces_all, wire_dir, moment_arms)
print(f"Iterations:\n{iterations}\n")

wire_vel_plotter(np.diff(wire_len, axis=0), times)
# motor_force_plotter(motor_forces, times)
plt.waitforbuttonpress()
