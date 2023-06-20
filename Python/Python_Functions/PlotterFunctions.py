import numpy as np
import matplotlib.pyplot as plt


def trajectory_plotter(position: np.ndarray[(None, None), np.float64], velocity: np.ndarray[(None, None), np.float64], acceleration: np.ndarray[(None, None), np.float64], timestamps: np.ndarray[(1, None), np.float64]):
    """
    Function generates a plot of position, velocity, and acceleration trajectories
    :param position: `numpy` array of shape (N, M), position values for N trajectories at M timestamps
    :param velocity: `numpy` array of shape (N, M), velocity values for N trajectories at M timestamps
    :param acceleration: `numpy` array of shape (N, M), acceleration values for N trajectories at M timestamps
    :param timestamps: `numpy` array of shape (N,), list of timestamps at which the trajectories will be plotted
    :return: None
    """
    # Labels for plots
    plot_labels = [['X', 'Y', 'rotZ'],
                   ['Position (m)', 'Velocity (m/s)', 'Acceleration (m/s^2)'],
                   ['Position trajectories', 'Velocity trajectories', 'Acceleration trajectories']]

    # Create the plot
    fig, ax = plt.subplots(3, 1)
    for i in range(position.shape[0]):
        ax[0].plot(timestamps, position[i], label=plot_labels[0][i])
        ax[1].plot(timestamps, velocity[i], label=plot_labels[0][i])
        ax[2].plot(timestamps, acceleration[i], label=plot_labels[0][i])

    # Add axis labels and title
    ax[2].set_xlabel('Time (s)')
    for i in range(3):
        ax[i].set_xlim([min(timestamps), max(timestamps)])
        ax[i].set_ylabel(plot_labels[1][i])
        ax[i].set_title(plot_labels[2][i])
        ax[i].legend()

    # Show the plot
    plt.show(block=False)
    return fig


def motor_force_plotter(motor_forces, timestamps):
    """
    Function generates a plot of the 5 motor force trajectories
    :param motor_forces: `numpy` array of shape (N, M), force values for M forces at N timestamps
    :param timestamps: `numpy` array of shape (N,), list of timestamps at which the trajectories will be plotted
    :return: None
    """
    # Labels for plots
    legend_labels = ['Motor 1', 'Motor 2', 'Motor 3', 'Motor 4', 'Motor 5']

    fig, ax = plt.subplots()
    for i in range(motor_forces.shape[1]):
        ax.plot(timestamps, motor_forces[:, i], label=legend_labels[i])
    ax.set_ylabel('Force (N)')
    ax.set_xlim([min(timestamps), max(timestamps)])
    ax.set_title('Motor force trajectories')
    ax.legend()

    # Show the plot
    plt.show(block=False)
    return


def wire_vel_plotter(wire_vel, timestamps):
    # Labels for plots
    legend_labels = ['Motor 1', 'Motor 2', 'Motor 3', 'Motor 4', 'Motor 5']

    fig, ax = plt.subplots()
    for i in range(wire_vel.shape[1]):
        ax.plot(timestamps[:-1], wire_vel[:, i], label=legend_labels[i])
    ax.set_ylabel('m/s')
    ax.set_xlim([min(timestamps), max(timestamps)])
    ax.set_title('Wire velocity trajectories')
    ax.legend()

    # Show the plot
    plt.show()
    return


def draw_robot_2d(com_position: np.ndarray[(None, 3), np.float64],
                  frame_angle: float,
                  anchor_positions: np.ndarray[(5, 3), np.float64],
                  motor_positions_local: np.ndarray[(5, 3), np.float64]):
    # create a new figure instance
    fig, ax = plt.subplots()

    # Compute motor positions
    r_frame = np.array(
        [[np.cos(frame_angle), -np.sin(frame_angle), 0], [np.sin(frame_angle), np.cos(frame_angle), 0], [0, 0, 0]])
    rotated_motors_pos = np.matmul(motor_positions_local, r_frame) + com_position

    # Compute wires
    wires = anchor_positions - rotated_motors_pos

    # Draw frame
    plt.fill(rotated_motors_pos[:, 0], rotated_motors_pos[:, 1], color='k')
    plt.text(com_position[0], com_position[1] + 0.11, 'CoM', fontsize=10, color='g', va='center', ha='center')
    #plt.quiver(0, 0, com_position[0], com_position[1], color='g', angles='xy', scale_units='xy', scale=1)
    plt.scatter(com_position[0], com_position[1], color='g', marker='x')

    # Draw motors and wires
    plt.quiver(rotated_motors_pos[:, 0], rotated_motors_pos[:, 1], wires[:, 0], wires[:, 1], color='r', angles='xy', scale_units='xy', scale=1)
    plt.scatter(anchor_positions[:, 0], anchor_positions[:, 1], color='r', s=15)
    plt.scatter(rotated_motors_pos[:, 0], rotated_motors_pos[:, 1], color='r', s=15)

    # Add text to vectors
    for i, (x, y) in enumerate(zip(rotated_motors_pos[:, 0], rotated_motors_pos[:, 1])):
        mid_x, mid_y = (x + anchor_positions[i, 0]) / 2, (y + anchor_positions[i, 1]) / 2
        plt.scatter(mid_x, mid_y, color='w', s=150)
        plt.text(mid_x, mid_y - 0.01, str(i + 1), fontsize=10, va='center', ha='center')

    # Adjust axes
    plt.xlim(np.min(anchor_positions[:, 0]) - 0.5, np.max(anchor_positions[:, 0]) + 0.5)
    plt.ylim(np.min(anchor_positions[:, 1]) - 0.5, np.max(anchor_positions[:, 1]) + 0.5)
    plt.xlabel('X coordinate (m)')
    plt.ylabel('Y coordinate (m)')
    plt.gca().set_aspect('equal', adjustable='box')

    return fig, ax
