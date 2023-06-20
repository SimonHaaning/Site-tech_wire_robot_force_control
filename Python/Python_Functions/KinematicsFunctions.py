import numpy as np


def forward_wire_kinematics(wire_directions: np.ndarray[(None, 2, 3), np.float64], anchor_positions: np.ndarray[(5, 3), np.float64], motor_positions_local: np.ndarray[(5, 3), np.float64]):
    """
    Function computes position of centre of mass and frame angle from the directions of wire 1 and 3
    :param wire_directions: `numpy` array of shape (N, 2, 3), N sets of wire directions from wire 1 and wire 3
    :param anchor_positions: `numpy` array of shape (5, 3), XYZ coordinates of 5 anchor points
    :param motor_positions_local: `numpy` array of shape (5, 3), XYZ coordinates of 5 motor points, relative to frame centre of mass
    :return: A tuple containing two `numpy` arrays with data type `np.float64`:
             The first, of shape (N, 3), contains N sets of centre of mass positions.
             The second, of shape (N,), contains N sets of frame angles about Z axis
    """

    # Compute distance from motors to wall
    dist_motor_to_wall = motor_positions_local[[0, 2], 2] - anchor_positions[[0, 2], 2]

    # Compute scaling factor from unit vector to wire vector
    wire_vector_scaling_factor = np.divide(dist_motor_to_wall, wire_directions[:, :, 2])

    # Determine wire vectors (known distance from motor to wall)
    wire_vectors = np.multiply(wire_directions, wire_vector_scaling_factor[:, :, np.newaxis])

    # Determine motor points
    pos_motors = np.subtract(anchor_positions[[0, 2], :], wire_vectors)

    # Determine vector between motor points (motor1 -> 3)
    between_motors = np.subtract(pos_motors[:, 1, :], pos_motors[:, 0, :])

    # Determine thetaz as angle of vector between motor points
    # NOTE: This assumes the motors are both placed at corners
    # If this is not true the angle needs an offset
    frame_angle = np.arctan2(between_motors[:, 1], between_motors[:, 0])

    # Rotation matrix for frame orientation
    N = frame_angle.shape[0]  # Number of rotation matrices to create
    r_frame = np.stack([np.cos(frame_angle), -np.sin(frame_angle), np.zeros(N),
                        np.sin(frame_angle), np.cos(frame_angle), np.zeros(N),
                        np.zeros(N), np.zeros(N), np.ones(N)], axis=1).reshape((N, 3, 3))

    # Apply rotation to frame
    motor_positions_local_rotated = np.matmul(np.repeat(motor_positions_local[np.newaxis, 0, :], N, axis=0)[:, np.newaxis, :], r_frame)

    # Determine the CoM pose, by using motor1 offset
    com_position = np.subtract(pos_motors[:, 0, :], motor_positions_local_rotated[:, 0, :])

    return com_position, frame_angle


def inverse_wire_kinematics(com_position: np.ndarray[(None, 3), np.float64], frame_angle: np.ndarray[(None,), np.float64], anchor_positions: np.ndarray[(5, 3), np.float64], motor_positions_local: np.ndarray[(5, 3), np.float64]):
    """
    Function computes the wire directions and lengths from the position of anchor point and the frame position
    :param com_position: `numpy` array of shape (N, 3), N sets of XYZ coordinates of frame centre of mass
    :param frame_angle: `numpy` array of shape (N,), N sets of frame angles about Z axis
    :param anchor_positions: `numpy` array of shape (5, 3), XYZ coordinates of 5 anchor points
    :param motor_positions_local: `numpy` array of shape (5, 3), XYZ coordinates of 5 motor points, relative to frame centre of mass
    :return: A tuple containing three `numpy` arrays with data type `np.float64`:
             The first, of shape (N, 5, 3), represents the N sets of 5 wire directions.
             The second, of shape (N, 5), represents the N sets of 5 wire lengths.
             The third, of shape (N, 5, 3), represents the N sets of moment arms from CoM to motors
    """

    # Amount of positions to compute inverse kinematics for
    N = com_position.shape[0]

    # Rotation matrix for frame orientation about Z
    r_frame = np.stack([np.cos(frame_angle), -np.sin(frame_angle), np.zeros(N),
                        np.sin(frame_angle),  np.cos(frame_angle), np.zeros(N),
                        np.zeros(N),         np.zeros(N),          np.ones(N)], axis=1).reshape((N, 3, 3))

    # Apply rotation to the motor positions
    motor_positions_local_rotated = np.matmul(np.repeat(motor_positions_local[np.newaxis, :, :], N, axis=0), r_frame)

    # Calculate position of motors in global frame
    motor_positions_world = np.add(com_position[:, np.newaxis, :], motor_positions_local_rotated)

    # Calculate wire vectors
    wire_vectors = np.subtract(np.repeat(anchor_positions[np.newaxis, :, :], N, axis=0), motor_positions_world)

    # Calculate wire lengths
    wire_lengths = np.sqrt(np.sum(wire_vectors**2, axis=2))

    # Wire directions
    wire_directions = np.divide(wire_vectors, wire_lengths[:, :, np.newaxis])

    return wire_directions, wire_lengths, motor_positions_local_rotated


def estimate_position(wire_directions: np.ndarray[(2, 3), np.float64],
                      anchor_positions: np.ndarray[(5, 3), np.float64],
                      motor_positions_local: np.ndarray[(5, 3), np.float64]):
    p1 = anchor_positions[0, :2] - motor_positions_local[0, :2]
    p2 = anchor_positions[2, :2] - motor_positions_local[2, :2]
    v1 = wire_directions[0, :2]
    v2 = wire_directions[1, :2]

    # Find the determinant of the system of equations
    det_A = np.linalg.det(np.column_stack((v1, -v2)))

    # Check if the lines intersect
    if det_A != 0:
        # Solve for the intersection point
        t = np.linalg.det(np.column_stack((p2 - p1, -v2))) / det_A
        com_position = p1 + t * v1
    else:
        # The lines are parallel and do not intersect (Should never happen)
        com_position = None
        pass

    return com_position
