import numpy as np
import math
import socket
import struct
import threading
import time
from Python_Functions.KinematicsFunctions import estimate_position


class ExponentialLowPassFilter:
    def __init__(self, alphas):
        self.alphas = alphas
        self.filtered_position = None
        self.last_filtered_position = None
        self.filtered_velocity = None
        self.last_filtered_velocity = None
        self.filtered_acceleration = None

    def update(self, new_position, time_interval):
        if self.filtered_position is None:
            self.last_filtered_position = new_position
            self.filtered_position = new_position
        else:
            self.last_filtered_position = self.filtered_position
            self.filtered_position = self.alphas[0] * new_position + (1 - self.alphas[0]) * self.filtered_position

        new_velocity = (self.filtered_position - self.last_filtered_position)/time_interval
        if self.filtered_velocity is None:
            self.last_filtered_velocity = new_velocity
            self.filtered_velocity = new_velocity
        else:
            self.last_filtered_velocity = self.filtered_velocity
            self.filtered_velocity = self.alphas[1] * new_velocity + (1 - self.alphas[1]) * self.filtered_velocity

        new_acceleration = (self.filtered_velocity - self.last_filtered_velocity)/time_interval
        if self.filtered_acceleration is None:
            self.filtered_acceleration = new_acceleration
        else:
            self.filtered_acceleration = self.alphas[2] * new_acceleration + (1 - self.alphas[2]) * self.filtered_acceleration
        return self.filtered_position, self.filtered_velocity, self.filtered_acceleration


def receive_imu_data(result_mutable,
                     sock: socket.socket(socket.AF_INET, socket.SOCK_STREAM),
                     interval: float,
                     data_lock: threading.Lock(),
                     conn_lock: threading.Lock(),
                     active_status: threading.Event(),
                     imu_nr: int):
    conn = None

    while True:
        # Establish TCP connection
        try:
            conn_lock.acquire()
            sock.listen()
            conn, _ = sock.accept()
            conn_lock.release()
        except socket.timeout:
            conn_lock.release()  # Make sure connection lock is released
            if conn is not None:  # Make sure conn1 closes if connection times out
                conn.close()
            print(f'Could not connect to IMU{imu_nr}... Retrying!')
        else:
            print(f'Connected to IMU{imu_nr}')
            # last_receive_time = 0

            # Main loop used to estimate position
            while True:
                before_receive = time.time()  # Record code execution time
                # print(f'Receive frequency IMU{imu_nr}: {1 / (before_receive - last_receive_time):.3f} Hz')
                # last_receive_time = before_receive

                # Receive data from IMUs
                try:
                    data = conn.recv(64)
                except (ConnectionResetError, ConnectionAbortedError):
                    # Either IMU disconnected while receiving
                    print('Lost connection to an IMU while receiving')
                    active_status.clear()  # Inform the system the state estimator stopped working
                    conn.close()  # Close connections
                    break
                else:
                    # Separate into the 3 XYZ values and convert to float
                    try:
                        data_lock.acquire()  # Get data lock
                        result_mutable[imu_nr - 1] = np.array([float(x) for x in data.decode().split(',')[:3]])
                    except ValueError as e:
                        print(e)
                        print(data.decode().split(','))
                        active_status.clear()  # Inform the system the state estimator stopped working
                        conn.close()  # Close connections
                        break
                    finally:
                        data_lock.release()  # Ensure the data lock is released
                    active_status.set()  # Set active status once data is made available

                after_send = time.time()  # Record code execution time
                send_execution_time = after_send - before_receive
                until_next_send = interval - send_execution_time
                if until_next_send > 0:
                    time.sleep(until_next_send)  # wait until it is time to send again


def estimate_current_state(input_mutable,
                           result_mutable,
                           interval: float,
                           input_data_lock: threading.Lock(),
                           result_data_lock: threading.Lock(),
                           active_status: threading.Event(),
                           imu1_active_status: threading.Event(),
                           imu2_active_status: threading.Event(),
                           imu3_active_status: threading.Event(),
                           max_len_lowpass,
                           anchors_pos,
                           motor_pos_rotated,
                           angle_offset):
    lp_filter = ExponentialLowPassFilter([0.1, 0.01, 0.01])
    last_before_filter = 0
    positions = np.zeros((max_len_lowpass, 3))  # Circular buffer for positional measurements
    velocities = np.zeros((max_len_lowpass, 3))  # Circular buffer for positional measurements
    # last_receive_time = 0

    # Main loop used to estimate position
    while True:
        before_receive = time.time()  # Record code execution time
        # print(f'Estimate position frequency: {1 / (before_receive - last_receive_time):.3f} Hz')
        # last_receive_time = before_receive

        # Wait for all IMU to be connected and receiving data
        while not imu1_active_status.is_set() or not imu2_active_status.is_set() or not imu3_active_status.is_set():
            active_status.clear()
            # print('State estimator waits for IMU connections')
            time.sleep(1)

        # Acquire data from IMUs
        try:
            input_data_lock.acquire()
            wire1_dir = input_mutable[0]
            wire2_dir = input_mutable[1]
            frame_dir = input_mutable[2]
        finally:
            input_data_lock.release()  # Ensure the data lock is released

        # Calculate current position and angle
        angle = math.atan2(frame_dir[1], frame_dir[0]) - math.pi * 0.5 + angle_offset
        r_frame = np.array([[np.cos(angle), -np.sin(angle), 0],
                            [np.sin(angle), np.cos(angle), 0],
                            [0, 0, 0]])
        com_pos = estimate_position(np.array([wire1_dir, wire2_dir]),
                                    anchors_pos,
                                    np.matmul(motor_pos_rotated, r_frame))

        # add current position and angle to circular buffer
        position_combined = np.append(com_pos, angle)  # list should be (3,)

        # Apply lowpass
        before_filter = time.time()
        lowpass_result = lp_filter.update(position_combined, max(before_filter-last_before_filter, 0.05))
        last_before_filter = before_filter

        lowpass_current_state = np.array(lowpass_result).transpose()
        lowpass_current_state[:, 2] = 0  # remove acceleration components

        positions[:-1] = positions[1:]
        positions[-1] = position_combined

        # add velocity to circular buffer
        velocities[:-1] = velocities[1:]
        velocities[-1] = np.mean(np.diff(positions, axis=0) * 100, axis=0)

        # Get estimate for position, velocity, and acceleration
        position_lowpass = np.mean(positions, axis=0)
        velocity_lowpass = velocities[-1]
        acceleration_lowpass = np.mean(np.diff(velocities, axis=0), axis=0)

        current_state = np.array([position_lowpass[[0, 1, 2]],
                                  velocity_lowpass[[0, 1, 2]],
                                  acceleration_lowpass[[0, 1, 2]]
                                  ]).transpose()
        current_state_pos_only = np.array([position_lowpass[[0, 1, 2]],
                                           [0, 0, 0],
                                           [0, 0, 0],
                                           ]).transpose()

        # Alter the mutable to give access to result elsewhere in the program
        try:
            result_data_lock.acquire()
            result_mutable[0] = lowpass_current_state
        finally:
            result_data_lock.release()  # Ensure the data lock is released
        active_status.set()  # Inform the system the state estimator is running

        after_receive = time.time()  # Record code execution time
        receive_execution_time = after_receive - before_receive
        until_next_send = interval - receive_execution_time
        if until_next_send > 0:
            time.sleep(until_next_send)  # wait until it is time to send again


def send_motor_forces(motor_forces_to_send,
                      wire_vel_to_send,
                      sock: socket.socket(socket.AF_INET, socket.SOCK_STREAM),
                      interval: float,
                      data_lock: threading.Lock(),
                      conn_lock: threading.Lock(),
                      active_status: threading.Event(),
                      forces_ready: threading.Event()):
    conn = None

    while True:  # not stop_flag.is_set():
        # Establish TCP connection
        try:
            conn_lock.acquire()
            sock.listen()
            conn, _ = sock.accept()
            conn_lock.release()
        except socket.timeout:
            conn_lock.release()  # Make sure connection lock is released
            if conn is not None:  # Make sure connection closes in case of timeout
                conn.close()
            print('Could not connect to motor controller... Retrying!')
        else:
            print('Connected to motor controller')
            active_status.set()  # Inform the system the force sender is running
            # last_send_time = 0

            # Main loop used to send motor forces
            while True:
                before_send = time.time()  # Record code execution time
                # print(f'Sending frequency: {1 / (before_send - last_send_time):.3f} Hz')
                # last_send_time = before_send

                # Wait for the force calculator to finish its first batch
                while not forces_ready.is_set():
                    # print('Force sender is waiting for forces to be calculated')
                    time.sleep(1)

                data_lock.acquire()  # Get lock on the data to send
                try:
                    # Pack motor forces into a byte array
                    msg_bytes = bytearray()
                    # print(f'{time.time()}->Motor forces: {motor_forces_to_send[0][:5]}')
                    for value in motor_forces_to_send[0]:
                        msg_bytes += struct.pack('f', value.astype(np.float32))
                    for value in wire_vel_to_send[0]:
                        msg_bytes += struct.pack('f', value.astype(np.float32))

                    try:
                        conn.sendall(msg_bytes)  # Send motor forces
                    except (ConnectionResetError, ConnectionAbortedError):
                        # The motor controller disconnected while forces were being sent
                        print('Lost connection while sending to motor controller')
                        active_status.clear()  # Inform the system the force sender stopped working
                        break
                finally:
                    data_lock.release()  # Ensure the data lock is released
                after_send = time.time()  # Record code execution time
                send_execution_time = after_send - before_send
                until_next_send = interval - send_execution_time
                if until_next_send > 0:
                    time.sleep(until_next_send)  # wait until it is time to send again
