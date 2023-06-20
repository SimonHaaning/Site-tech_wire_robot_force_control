import numpy as np
import socket
from Python_Functions.KinematicsFunctions import forward_wire_kinematics, inverse_wire_kinematics
import time

HOST_IP = socket.gethostbyname(socket.gethostname())  # IP of the device running the current code
PORT_IMU1 = 65439  # Port to receive data from the first IMU
PORT_IMU2 = 65440  # Port to receive data from the second IMU
SOCKET_TIMEOUT = 0.001

# Anchor positions static frame (m)
a1 = [0.0, 3.5, 0.0]
a2 = [1.5, 3.5, 0.0]
a3 = [3.0, 3.5, 0.0]
a4 = [3.0, 0.0, 0.0]
a5 = [0.0, 0.0, 0.0]
ANCHORS_POS = np.array([a1, a2, a3, a4, a5])  # 5x3 matrix

# Motor positions relative to CoM (m)
m1 = [-0.4, 0.25, 0.1]
m2 = [0.01, 0.25, 0.1]
m3 = [0.4, 0.25, 0.1]
m4 = [0.4, -0.25, 0.1]
m5 = [-0.4, -0.25, 0.1]
MOTORS_POS = np.array([m1, m2, m3, m4, m5])  # 5x3 matrix

# Initial position
com_position = np.array([[1.5, 1.5, 0]])
angle = np.array([0])


def calibration_matrix(v1, v2):
    """Returns the rotation matrix needed to transform v1 into v2."""
    if np.allclose(v1, v2):
        return np.eye(3)  # Return identity matrix if vectors are identical

    v = np.cross(v1, v2)  # Calculate the cross product of the two vectors
    s = np.linalg.norm(v)  # Calculate the sine of the angle between the vectors

    c = np.dot(v1, v2)  # Calculate the cosine of the angle between the vectors

    vx = np.array([[0, -v[2], v[1]],
                   [v[2], 0, -v[0]],
                   [-v[1], v[0], 0]])

    rotation_matrix = np.eye(3) + vx + np.dot(vx, vx) * ((1 - c) / (s ** 2))

    return rotation_matrix


def imu_calibration(wire1_zero, wire2_zero, com_position, angle, anchors_pos, motors_pos):
    # Compute wire directions
    wire_dirs, _, _ = inverse_wire_kinematics(com_position, angle, anchors_pos, motors_pos)

    # Get calibration matrices
    calib_wire1 = calibration_matrix(wire1_zero, wire_dirs[0, 0])
    calib_wire2 = calibration_matrix(wire2_zero, wire_dirs[0, 2])

    #print(f'Wire 1 calib matrix: {calib_wire1}')
    #print(f'Wire 2 calib matrix: {calib_wire2}')
    return calib_wire1, calib_wire2, wire_dirs[0, 0], wire_dirs[0, 2]


def setup_socket_tcp(ip, port, timeout):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind((ip, port))
    sock.settimeout(timeout)
    return sock


def receive_imu_data(sock: socket.socket(socket.AF_INET, socket.SOCK_STREAM)):
    try:
        sock.listen()
        conn, _ = sock.accept()
    except socket.timeout:
        # print('No connection to client')
        wire_dir = None
    else:
        with conn:
            data = conn.recv(1024)
            if data:
                # Separate into the 3 XYZ values and convert to float
                data = [float(x) for x in data.decode().split(',')[:-1]]
                wire_dir = np.array(data[:3])
    return wire_dir

"""
# Start socket objects
sock_imu1 = setup_socket_tcp(HOST_IP, PORT_IMU1, SOCKET_TIMEOUT)
print(f'Server started at {HOST_IP}:{PORT_IMU1}')
sock_imu2 = setup_socket_tcp(HOST_IP, PORT_IMU2, SOCKET_TIMEOUT)
print(f'Server started at {HOST_IP}:{PORT_IMU2}')

# Initialize data receptors
data_received1 = None
data_received2 = None

# Make first measurements
while data_received2 is None:
    data_received2 = receive_imu_data(sock_imu2)  # Receive IMU2 sensor output
print('Got wire2')

while data_received1 is None:
    data_received1 = receive_imu_data(sock_imu1)  # Receive IMU1 sensor output
print('Got wire1')

# Calibrate wires
calib1, calib2, wire1_dir, wire2_dir = imu_calibration(data_received1, data_received2, com_position, angle, ANCHORS_POS, MOTORS_POS)

while True:
    data_received1 = receive_imu_data(sock_imu1)  # Receive IMU1 sensor output
    if data_received1 is not None:
        #print(f'Wire 1 data: {data_received1}')
        wire1_dir = np.dot(calib1, data_received1)
        print(f'Wire 1 calib: {wire1_dir}')

    data_received2 = receive_imu_data(sock_imu2)  # Receive IMU2 sensor output
    if data_received2 is not None:
        #print(f'Wire 2 data: {data_received2}')
        wire2_dir = np.dot(calib2, data_received2)
        print(f'Wire 2 calib: {wire2_dir}')

    #if wire1_dir is not None and wire2_dir is not None:
        #print('Got data from both IMU')
    com_pos, angle = forward_wire_kinematics(np.array([[wire1_dir, wire2_dir]]), ANCHORS_POS, MOTORS_POS)
    print(f'Position:\nX: {com_pos[0, 0]:.3f}\nY: {com_pos[0, 1]:.3f}\nZ: {com_pos[0, 2]:.3f}\nAngle: {angle:0.3f}')
    #time.sleep(0.0)
"""