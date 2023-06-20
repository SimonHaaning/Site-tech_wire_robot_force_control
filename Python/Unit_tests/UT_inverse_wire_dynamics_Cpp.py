import numpy as np
import ctypes
from pathlib import Path
from Python_Functions.KinematicsFunctions import inverse_wire_kinematics
from Python_Functions.DynamicsFunctions import inverse_wire_dynamics
from Python_Functions.TrajectoryGenerationFunctions import trajectory_generation
import time
from tqdm import tqdm


# Define the MatrixXf type using ctypes (use float32 NOT float64)
class MatrixXf(ctypes.Structure):
    _fields_ = [("data", ctypes.POINTER(ctypes.c_float)),
                ("rows", ctypes.c_int),
                ("cols", ctypes.c_int)]


# Define the VectorXf type using ctypes (use float32 NOT float64)
class VectorXf(ctypes.Structure):
    _fields_ = [("data", ctypes.POINTER(ctypes.c_float)),
                ("size", ctypes.c_int)]


# Define the return structure
class WireDynamicsReturnStruct(ctypes.Structure):
    _fields_ = [("wire_forces", VectorXf),
                ("converge_iterations", ctypes.c_int)]


eigen_type35 = np.ctypeslib.ndpointer(dtype=np.float32, shape=(5,3), flags='C_CONTIGUOUS')
eigen_type3 = np.ctypeslib.ndpointer(dtype=np.float32, shape=(3,), flags='C_CONTIGUOUS')

# Import C++ functions using ctypes
file_path = Path(__file__).parent.parent.parent / 'C++' / 'cfunctions.dll'
functionlib = ctypes.cdll.LoadLibrary(str(file_path))
wire_dynamics = functionlib.wire_dynamics
wire_dynamics.argtypes = (eigen_type3, eigen_type35, eigen_type35)
wire_dynamics.restype = WireDynamicsReturnStruct

# Anchor positions static frame (m)
a1 = [0.0, 3.5, 0.0]
a2 = [1.5, 3.5, 0.0]
a3 = [3.0, 3.5, 0.0]
a4 = [3.0, 0.0, 0.0]
a5 = [0.0, 0.0, 0.0]
anchors_pos = np.array([a1, a2, a3, a4, a5], np.float32)  # 5x3 matrix

# Motor positions relative to CoM (m)
m1 = [-0.4, 0.25, 0.1]
m2 = [0.01, 0.25, 0.1]
m3 = [0.4, 0.25, 0.1]
m4 = [0.4, -0.25, 0.1]
m5 = [-0.4, -0.25, 0.1]
motors_pos = np.array([m1, m2, m3, m4, m5], np.float32)  # 5x3 matrix

# Inputs
final_state = np.array([[0.9, 0, 0],
                        [0.9, 0, 0],
                        [0, 0, 0]], np.float32)
start_state = np.array([[2.1, 0, 0],
                        [2.1, 0, 0],
                        [0, 0, 0]], np.float32)
acceleration_limits = np.array([0.01, 0.01, 0.001])
time_resolution = 100

# Compute trajectory
position, velocity, acceleration, times = trajectory_generation(start_state, final_state, time_resolution,
                                                                acceleration_limits)

# Compute forces
m_frame = 100  # kg
I_frame = 101.85669  # kg*m^2
F_g = np.array([0, -9.82 * m_frame, 0])  # force from gravity
frame_forces_motion = np.multiply(np.array([m_frame, m_frame, I_frame])[:, np.newaxis], acceleration).transpose()
frame_forces_all = np.subtract(frame_forces_motion, F_g)

# Shape the array for kinematics (SHOULD BE FIXED TO DO THIS AUTOMATICALLY)
com_position = np.concatenate((position[:2, :], np.zeros_like(position[0, :][np.newaxis, :])), axis=0).transpose()
angles = position[2, :]

# Determine wire directions along trajectory
wire_dir, wire_len, moment_arms = inverse_wire_kinematics(com_position, angles, anchors_pos, motors_pos)

# Determine wire forces
before = time.time()
motor_forces, _, _, iterations, _, _ = inverse_wire_dynamics(frame_forces_all[:200], wire_dir[:200], moment_arms[:200])
after = time.time()
print(after - before)
print(f"Iterations:\n{iterations}\n")

# static case for DEBUG
#com_position = np.array([[2.0, 2.0, 0]])
#angle = np.array([0])
#wire_dir, wire_len, moment_arms = inverse_wire_kinematics(com_position, angle, anchors_pos, motors_pos)
#frame_forces_all = np.array([0, 9.82 * m_frame, 0])


# Prepare arrays for c++
#for i in tqdm(range(len(times))):
before = time.time()
for i in range(len(times)):
    frame_forces_send = np.ascontiguousarray(frame_forces_all[i].astype(np.float32))
    wire_dir_send = np.ascontiguousarray(wire_dir[i].astype(np.float32))
    moment_arms_send = np.ascontiguousarray(moment_arms[i].astype(np.float32))

    retval = wire_dynamics(frame_forces_send, wire_dir_send, moment_arms_send)

after = time.time()
print(after - before)
#print(retval.wire_forces.size)
