import numpy as np
from Python_Functions.KinematicsFunctions import inverse_wire_kinematics
from Python_Functions.DynamicsFunctions import inverse_wire_dynamics, forward_wire_dynamics
from Python_Functions.TrajectoryGenerationFunctions import trajectory_generation
#from Python_Functions.PlotterFunctions import motor_force_plotter, trajectory_plotter
import matplotlib.pyplot as plt
import time

# Anchor positions static frame (m)
a1 = [0.0, 3.355, -0.20646]
a2 = [1.45, 3.355, -0.20646]
a3 = [2.9, 3.355, -0.20646]
a4 = [2.9, 0.0, -0.20646]
a5 = [0.0, 0.0, -0.20646]
ANCHORS_POS = np.array([a1, a2, a3, a4, a5])  # 5x3 matrix

# Motor positions relative to CoM (m)
m1 = [-0.36785, 0.25047, -0.00996]
m2 = [0.00065, 0.25047, -0.00996]
m3 = [0.41995, 0.16827, -0.00996]
m4 = [0.33960, -0.31953, -0.00996]
m5 = [-0.45005, -0.23873, -0.00996]
MOTORS_POS = np.array([m1, m2, m3, m4, m5])  # 5x3 matrix
ANGLE_OFFSET = -0.117

# Inputs
final_state = np.array([[1.3, 0, 0],
                        [1.3, 0, 0],
                        [ANGLE_OFFSET, 0, 0]], np.float64)
start_state = np.array([[1.2, 0, 0],
                        [1.2, 0, 0],
                        [ANGLE_OFFSET, 0, 0]], np.float64)
acceleration_limits = np.array([0.025, 0.025, 0.001])
time_resolution = 100

# Compute trajectory
position, velocity, acceleration, times = trajectory_generation(start_state, final_state, time_resolution, acceleration_limits)
#trajectory_plotter(position, velocity, acceleration, times)

# Compute forces
m_frame = 100  # kg
I_frame = 101.85669  # kg*m^2
F_g = np.array([0, -9.82*m_frame, 0])  # force from gravity
frame_forces_motion = np.multiply(np.array([m_frame, m_frame, I_frame])[:, np.newaxis], acceleration).transpose()
frame_forces_all = np.subtract(frame_forces_motion, F_g)

# Shape the array for kinematics (SHOULD BE FIXED TO DO THIS AUTOMATICALLY)
com_position = np.concatenate((position[:2, :], np.zeros_like(position[0, :][np.newaxis, :])), axis=0).transpose()
angles = position[2, :]

# Determine wire directions along trajectory
wire_dir, wire_len, moment_arms = inverse_wire_kinematics(com_position, angles, ANCHORS_POS, MOTORS_POS)



# Determine wire forces
#before = time.time()
#motor_forces, _, _, iterations, _, err_array = inverse_wire_dynamics(frame_forces_all, wire_dir, moment_arms)
#after = time.time()
#print(f"Iterations:\n{iterations}")
#print(after-before)

before = time.time()
motor_forces, _, _, iterations, _, err_array= inverse_wire_dynamics(frame_forces_all[:10], wire_dir[:10], moment_arms[:10], max_iter=10000, initial_wire_force=0)
after = time.time()
print(f"Iterations:\n{iterations}")
print(after-before)
threshold = np.ones_like(err_array[0, 1:iterations, 0]) * 0.001
plt.semilogy(abs(err_array[0, 1:iterations, 0]), label='X force')
plt.semilogy(abs(err_array[0, 1:iterations, 1]), label='Y force')
plt.semilogy(abs(err_array[0, 1:iterations, 2]), label='Z moment')
plt.semilogy(threshold, label='Tolerance')
#plt.title("Pose deviation")
plt.xlabel("Iteration")
plt.ylabel("Log10 absolute deviation")
plt.legend()
plt.savefig('DeviationPerIterationLog.pdf', format='pdf', bbox_inches='tight')
plt.show()

plt.plot(err_array[0, 1:iterations, 0], label='X force')
plt.plot(err_array[0, 1:iterations, 1], label='Y force')
plt.plot(err_array[0, 1:iterations, 2], label='Z moment')
plt.plot(threshold, label='Tolerance', c='r')
plt.plot(-threshold, c='r')
plt.xlabel("Iteration")
plt.ylabel("Deviation")
plt.legend()
plt.savefig('DeviationPerIteration.pdf', format='pdf', bbox_inches='tight')
plt.show()

#print(motor_forces.shape)
#print(wire_dir.shape)
#print(moment_arms.shape)
#F, M = forward_wire_dynamics(motor_forces[:10], wire_dir[:10], moment_arms[:10])
#print(F.shape)
#print(M.shape)

#motor_force_plotter(motor_forces, times)
#plt.waitforbuttonpress()