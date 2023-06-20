import numpy as np
import socket
import threading
import time
import Threads
from Python_Functions.TrajectoryGenerationFunctions import trajectory_generation
from Python_Functions.KinematicsFunctions import inverse_wire_kinematics
from Python_Functions.DynamicsFunctions import inverse_wire_dynamics
from Python_Functions.RobotMotionSimulator import RobotMotionSimulator
from Python_Functions.PlotterFunctions import wire_vel_plotter, motor_force_plotter
import csv

# Anchor positions static frame (m)
a1 = [0.0, 3.355, -0.20646]
a2 = [1.45, 3.355, -0.20646]
a3 = [3.08-0.240, 3.355-0.065, -0.20646]
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
ANGLE_OFFSET = -0.12  # Frame angle offset measured by IMU

# Inputs
# Boundary X 1.00 -> 2.00
# Boundary y 1.00 -> 2.00
test_nr = '11'
START_STATE = np.array([[1.44, 0.0, 0.0],
                        [1.20, 0.0, 0.0],
                        [0, 0.0, 0.0]])
FINAL_STATE = np.array([[1.15, 0.0, 0.0],
                        [1.65, 0.0, 0.0],
                        [0, 0.0, 0.0]])

# Tolerance for reaching goal
GOAL_TOLERANCE = np.array([[0.1, 0.001, 0.001],
                           [0.1, 0.001, 0.001],
                           [0.1, 0.001, 0.001]])

# Create object for robot simulation
sim = RobotMotionSimulator(START_STATE, ANCHORS_POS, MOTORS_POS)

# Parameters for trajectory generation
ACCELERATION_LIMITS = np.array([0.01, 0.01, 0.001])
TIME_RESOLUTION = 1

# Parameters for dynamics
MAX_ITERATIONS = 5000
MIN_WIRE_FORCE = 500
MAX_WIRE_FORCE = 2000

# Inertial parameters
MASS_FRAME = 59.5  # kg
INERTIA_FRAME = 12.5369  # kg*m^2
FORCE_GRAVITY = np.array([0, -9.82 * MASS_FRAME, 0])  # force from gravity

# Socket parameters
HOST_IP = socket.gethostbyname(socket.gethostname())  # IP of the device running the current code
PORT_IMU1 = 65439  # Port to receive data from the first IMU
PORT_IMU2 = 65440  # Port to receive data from the second IMU
PORT_IMU3 = 65441
PORT_MCON = 65442  # Port to send data to the motor controller
SOCKET_TIMEOUT = 1.0  # Timeout in seconds

# Timing
FORCES_SEND_INTERVAL = 1  # seconds
FORCES_SEND_BATCH_SIZE = 1
IMU_RECEIVE_INTERVAL = 0.01  # seconds
STATE_ESTIMATION_INTERVAL = 0.1  # seconds
ESTIMATION_LOWPASS = 10  # Amount of samples from IMU readings used to estimate state

# Rotate motor positions
r_frame = np.array([[np.cos(ANGLE_OFFSET), -np.sin(ANGLE_OFFSET), 0],
                    [np.sin(ANGLE_OFFSET), np.cos(ANGLE_OFFSET), 0],
                    [0, 0, 0]])
MOTORS_POS_ROTATED = np.matmul(MOTORS_POS, r_frame)

np.set_printoptions(precision=5)


def debug_state_message(system_state):
    print("\nCurrent state")
    print(f"X: {system_state[0, 0]:.4f} m\t\tdX: {system_state[0, 1]:.4f} m/s\t\tddX: {system_state[0, 2]:.4f} m/s^2")
    print(f"Y: {system_state[1, 0]:.4f} m\t\tdY: {system_state[1, 1]:.4f} m/s\t\tddY: {system_state[1, 2]:.4f} m/s^2")
    print(f"Z: {system_state[2, 0]:.4f} rad\tdZ: {system_state[2, 1]:.4f} rad/s\tddZ: {system_state[2, 2]:.4f} rad/s^2")


def setup_socket_tcp(ip, port, timeout):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind((ip, port))
    sock.settimeout(timeout)
    return sock


# Initialize socket server
sock_imu1 = setup_socket_tcp(HOST_IP, PORT_IMU1, SOCKET_TIMEOUT)  # Create socket objects
sock_imu2 = setup_socket_tcp(HOST_IP, PORT_IMU2, SOCKET_TIMEOUT)
sock_imu3 = setup_socket_tcp(HOST_IP, PORT_IMU3, SOCKET_TIMEOUT)
sock_mcon = setup_socket_tcp(HOST_IP, PORT_MCON, SOCKET_TIMEOUT)
print(f'Server started at {HOST_IP} using ports: {PORT_IMU1}, {PORT_IMU2}, {PORT_IMU3}, {PORT_MCON}')

# Initialize arrays for mutable communication with threads
imu_data_mutable = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
current_state_mutable = [START_STATE]
motor_forces = [np.zeros((FORCES_SEND_BATCH_SIZE * 5,), np.float64)]
wire_vel = [np.zeros((FORCES_SEND_BATCH_SIZE * 5,), np.float64)]
times = np.zeros(FORCES_SEND_BATCH_SIZE, np.float64)  # Array for times (only used for simulation)

# Initialize threading
wifi_lock = threading.Lock()  # Lock for access to wifi
imu_data_lock = threading.Lock()  # Data lock for imu result
current_state_lock = threading.Lock()  # Data lock for the estimate_current_state thread
motor_force_lock = threading.Lock()  # Data lock for the send_motor_forces thread
imu1_active_flag = threading.Event()  # Active flags for IMU data collectors
imu2_active_flag = threading.Event()
imu3_active_flag = threading.Event()
pos_estimator_active_flag = threading.Event()  # Active flag for the state estimator
sender_active_flag = threading.Event()  # Active flag for the force sender
motion_controller_active_flag = threading.Event()  # Active flag for the main planning loop

# Data lists for logging
log_current_state = []
log_motor_forces = []
log_wire_vel = []


# Logger function
def logger_function(filename: str, data_list: list):
    with open('test_data/'+filename+'.csv', 'w', newline='') as file:
        writer = csv.writer(file)

        # Write data to file each index becomes a row
        for row in data_list:
            writer.writerow(row)


# Create IMU threads
imu1_receiver_thread = threading.Thread(target=Threads.receive_imu_data,
                                        args=(imu_data_mutable,
                                              sock_imu1,
                                              STATE_ESTIMATION_INTERVAL,
                                              imu_data_lock,
                                              wifi_lock,
                                              imu1_active_flag,
                                              1))
imu1_receiver_thread.daemon = True  # Allow sending thread to exit when code stops
imu1_receiver_thread.start()
imu2_receiver_thread = threading.Thread(target=Threads.receive_imu_data,
                                        args=(imu_data_mutable,
                                              sock_imu2,
                                              STATE_ESTIMATION_INTERVAL,
                                              imu_data_lock,
                                              wifi_lock,
                                              imu2_active_flag,
                                              2))
imu2_receiver_thread.daemon = True  # Allow sending thread to exit when code stops
imu2_receiver_thread.start()
imu3_receiver_thread = threading.Thread(target=Threads.receive_imu_data,
                                        args=(imu_data_mutable,
                                              sock_imu3,
                                              STATE_ESTIMATION_INTERVAL,
                                              imu_data_lock,
                                              wifi_lock,
                                              imu3_active_flag,
                                              3))
imu3_receiver_thread.daemon = True  # Allow sending thread to exit when code stops
imu3_receiver_thread.start()

# Create estimate_current_state thread
estimate_current_state_thread = threading.Thread(target=Threads.estimate_current_state,
                                                 args=(imu_data_mutable,
                                                       current_state_mutable,
                                                       STATE_ESTIMATION_INTERVAL,
                                                       imu_data_lock,
                                                       current_state_lock,
                                                       pos_estimator_active_flag,
                                                       imu1_active_flag,
                                                       imu2_active_flag,
                                                       imu3_active_flag,
                                                       ESTIMATION_LOWPASS,
                                                       ANCHORS_POS,
                                                       MOTORS_POS_ROTATED,
                                                       ANGLE_OFFSET,
                                                       log_current_state))
estimate_current_state_thread.daemon = True  # Allow sending thread to exit when code stops
estimate_current_state_thread.start()

# Create send_motor_forces thread
send_motor_forces_thread = threading.Thread(target=Threads.send_motor_forces,
                                            args=(motor_forces,
                                                  wire_vel,
                                                  sock_mcon,
                                                  FORCES_SEND_INTERVAL,
                                                  motor_force_lock,
                                                  wifi_lock,
                                                  sender_active_flag,
                                                  motion_controller_active_flag,
                                                  log_motor_forces,
                                                  log_wire_vel))
send_motor_forces_thread.daemon = True  # Allow sending thread to exit when code stops
send_motor_forces_thread.start()

# Initialise iteration counter for control loop
iterations = MAX_ITERATIONS + 1

# Catch keyboard interrupt
try:
    # Main control loop
    while True:
        while not sender_active_flag.is_set() or not pos_estimator_active_flag.is_set():
            motion_controller_active_flag.clear()
            time.sleep(1)

        # Get current state from sensor reading
        current_state_lock.acquire()
        current_state = current_state_mutable[0]
        current_state_lock.release()

        # Check if goal is reached
        if np.all(abs(np.subtract(FINAL_STATE, current_state)) <= GOAL_TOLERANCE):
            print(f"\n\nGOAL REACHED")
            debug_state_message(current_state)
            break

        # Compute trajectory
        position, velocity, acceleration, times_new = trajectory_generation(current_state,
                                                                            FINAL_STATE,
                                                                            TIME_RESOLUTION,
                                                                            ACCELERATION_LIMITS)

        # Compute forces
        frame_forces_motion = np.multiply(np.array([MASS_FRAME, MASS_FRAME, INERTIA_FRAME])[:, np.newaxis],
                                          acceleration).transpose()
        frame_forces_all = np.subtract(frame_forces_motion, FORCE_GRAVITY)

        # Shape the array for kinematics
        com_positions = np.concatenate((position[:2, :], np.zeros_like(position[0, :][np.newaxis, :])),
                                       axis=0).transpose()
        angles = position[2, :]

        # Compute wire directions along trajectory
        wire_dirs, wire_len, moment_arms = inverse_wire_kinematics(com_positions, angles, ANCHORS_POS, MOTORS_POS_ROTATED)

        # Compute changes in wire lengths
        wire_vel_new = np.diff(wire_len, axis=0)

        # Compute motor forces
        motor_forces_new, f_frame, _, iterations, err, _ = inverse_wire_dynamics(frame_forces_all[:FORCES_SEND_BATCH_SIZE],
                                                                                 wire_dirs[:FORCES_SEND_BATCH_SIZE],
                                                                                 moment_arms[:FORCES_SEND_BATCH_SIZE],
                                                                                 max_iter=MAX_ITERATIONS,
                                                                                 wire_forces_min=MIN_WIRE_FORCE,
                                                                                 wire_forces_max=MAX_WIRE_FORCE,
                                                                                 initial_wire_force=MIN_WIRE_FORCE)

        needed_forces = np.where(wire_vel_new[:FORCES_SEND_BATCH_SIZE][:, :, np.newaxis] > -1e-6, 0, motor_forces_new)

        # If trajectory is feasible update the result
        if iterations <= MAX_ITERATIONS:
            # print(f"Successfully computed new motor forces after {iterations} iterations")
            times = times_new
            # print(f'Expected to complete in {times[-1]} seconds')

            # print(f'{time.time()}->Max motor force: {np.max(motor_forces_new, axis=0).transpose()[0]}')
            # print(f'{time.time()}->Wire velocities: {np.mean(wire_vel_new, axis=0)}')

        else:
            # print(f"Error: Maximum iterations reached, could not converge")
            pass

        # Update motor forces
        motor_force_lock.acquire()
        motor_forces[0] = np.reshape(motor_forces_new, (FORCES_SEND_BATCH_SIZE * 5,))
        wire_vel[0] = np.reshape(wire_vel_new[:FORCES_SEND_BATCH_SIZE], (FORCES_SEND_BATCH_SIZE * 5))
        motor_force_lock.release()
        motion_controller_active_flag.set()
        # Simulate robot motion
        # sim.simulate_motion(motor_forces[:FORCES_SEND_BATCH_SIZE], times[:FORCES_SEND_BATCH_SIZE])  # Simulate motion

        # DEBUG
        debug_state_message(current_state)
        print(f'Motor forces after {iterations} iterations:\n{motor_forces_new[0].transpose()}')
        print(f'Wire velocities:\n{wire_vel_new[0]}')
        # print(f'Maximum errors:\n{np.max(err, axis=0).transpose()}')
        # print(f'Average frame forces:\n{np.mean(f_frame, axis=0).transpose()}')
        # wire_vel_plotter(wire_vel_new[:100], times_new[:101])
        # motor_force_plotter(needed_forces, times_new[:100])

except KeyboardInterrupt as e:
    pass

finally:
    # Log data to files
    folder = 'motion_tests/' + test_nr + '/'
    logger_function(folder+'target_pos', [FINAL_STATE])
    logger_function(folder+'current_state', log_current_state)
    logger_function(folder+'motor_forces', log_motor_forces)
    logger_function(folder+'wire_vel', log_wire_vel)

    exit(0)
