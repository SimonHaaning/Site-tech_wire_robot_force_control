import struct

import numpy as np
import socket
import threading
import time

# Socket parameters
HOST_IP = socket.gethostbyname(socket.gethostname())  # IP of the device running the current code
PORT_MCON = 65441  # Port to send data to the motor controller
SOCKET_TIMEOUT = 100

FORCES_SEND_INTERVAL = 0.5  # seconds
FORCES_SEND_BATCH_SIZE = 100

# Make dummy array to send
motor_forces = np.random.random((FORCES_SEND_BATCH_SIZE, 5, 1))
print(f'Motor forces:\n{motor_forces}')

# Reshape to array
motor_forces = np.reshape(motor_forces, (FORCES_SEND_BATCH_SIZE*5,))
print(f'reshaped:\n{motor_forces}')


def send_motor_forces(motor_forces_to_send,
                      sock: socket.socket(socket.AF_INET, socket.SOCK_STREAM),
                      interval: float,
                      lock: threading.Lock(),
                      stop_flag: threading.Event()):
    while True:
        # Establish TCP connection
        try:
            sock.listen()
            conn, _ = sock.accept()
        except socket.timeout:
            print('No connection to motor controller client')
        else:
            while True:
                lock.acquire()  # Get lock on the data to send
                try:
                    # Fill buffer with values
                    msg_bytes = bytearray()
                    for value in motor_forces_to_send:
                        msg_bytes += struct.pack('f', value.astype(np.float32))

                    try:
                        # Send motor forces
                        conn.sendall(msg_bytes)
                    except ConnectionResetError:
                        break
                finally:
                    lock.release()
                # last_forces_send_time = current_time
                time.sleep(interval)


def setup_socket_tcp(ip, port, timeout):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind((ip, port))
    sock.settimeout(timeout)
    return sock


# Start socket
sock_mcon = setup_socket_tcp(HOST_IP, PORT_MCON, SOCKET_TIMEOUT)
print(f'Server started at {HOST_IP}:{PORT_MCON}')

# Initialize threading
thread_lock = threading.Lock()  # Data lock for threads
stop_event = threading.Event()  # Event for stopping the thread
sending_thread = threading.Thread(target=send_motor_forces,
                                  args=(motor_forces, sock_mcon, FORCES_SEND_INTERVAL, thread_lock, stop_event))
sending_thread.start()

# Timings
current_time = 0
last_forces_send_time = 0
start_time = time.time()
while True:
    current_time = time.time()
    print(f'Program running for {current_time - start_time:.2f} seconds')
    time.sleep(10)
