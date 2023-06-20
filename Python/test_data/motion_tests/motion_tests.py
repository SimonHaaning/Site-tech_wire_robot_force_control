import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from test_data.test_data_reader import reader_function
from Python_Functions.TrajectoryGenerationFunctions import trajectory_generation

a1 = [0.0, 3.355, -0.20646]
a2 = [1.45, 3.355, -0.20646]
a3 = [3.08 - 0.240, 3.355 - 0.065, -0.20646]
a4 = [2.9, 0.0, -0.20646]
a5 = [0.0, 0.0, -0.20646]
ANCHORS_POS = np.array([a1, a2, a3, a4, a5])  # 5x3 matrix

estimation_frequency = 10

target_pose = []
estimated_pose = []
target_state = []
current_state = []
# Read CSV for recorded data
for i in range(12):
    try:
        # read the target pose
        target = reader_function(str(i + 1) + '/', 'target_pos.csv')

        # read the estimated pose path
        pose = reader_function(str(i+1)+'/', 'current_state.csv')
        x = pose[:, 0, 0]
        y = pose[:, 1, 0]
        z = pose[:, 2, 0]

        # Append data
        target_pose.append(target[0, :, 0])
        estimated_pose.append(np.array([x, y, z]))
        target_state.append(target[0])
        current_state.append(pose)
    except (IndexError, FileNotFoundError) as e:
        print(e)
        target_pose.append(None)
        estimated_pose.append(None)
        pass

for test_nr in range(12):
    # Reach goal test
    fig1, ax = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

    # Plot relative to time instead of iteration
    time = np.linspace(0, len(estimated_pose[test_nr][0, :])/estimation_frequency, len(estimated_pose[test_nr][0, :]))

    print(f'test {test_nr+1}:')

    # Plot motion for X individually
    ax[0].plot(time, np.repeat(target_pose[test_nr][0], estimated_pose[test_nr].shape[1]), 'r', label='Target')
    ax[0].plot(time, estimated_pose[test_nr][0, :], 'b', label='Measured position')
    ax[0].set_xlim((0, time[-1]))
    ax[0].set_ylim((np.min(ANCHORS_POS[:, 1])-0.1, np.max(ANCHORS_POS[:, 1])+0.1))
    print(f'{estimated_pose[test_nr][0, -1] - target_pose[test_nr][0]:.4f}')

    # Plot motion for Y individually
    ax[1].plot(time, np.repeat(target_pose[test_nr][1], estimated_pose[test_nr].shape[1]), 'r', label='Target')
    ax[1].plot(time, estimated_pose[test_nr][1, :], 'b', label='Measured position')
    ax[1].set_xlim((0, time[-1]))
    ax[1].set_ylim((np.min(ANCHORS_POS[:, 1])-0.1, np.max(ANCHORS_POS[:, 1])+0.1))
    print(f'{estimated_pose[test_nr][1, -1] - target_pose[test_nr][1]:.4f}')

    # Plot motion for Z individually
    ax[2].plot(time, np.repeat(target_pose[test_nr][2], estimated_pose[test_nr].shape[1]), 'r', label='Target')
    ax[2].plot(time, estimated_pose[test_nr][2, :], 'b', label='Measured orientation')
    ax[2].set_xlim((0, time[-1]))
    y_bound = np.maximum(np.max(np.abs(estimated_pose[test_nr][2, :])), np.abs(target_pose[test_nr][2]))+0.1
    ax[2].set_ylim((-y_bound, y_bound))
    print(f'{estimated_pose[test_nr][2, -1] - target_pose[test_nr][2]:.4f}')

    # Combine x axis
    ax[0].xaxis.set_tick_params(top=False)
    ax[1].xaxis.set_tick_params(top=False)
    plt.subplots_adjust(hspace=0.05)

    # Add grids
    ax[0].grid()
    ax[1].grid()
    ax[2].grid()

    # Add labels
    ax[0].set_title(f'Reach goal result (test {test_nr+1})')
    ax[0].set_ylabel('X position (m)')
    ax[1].set_ylabel('Y position (m)')
    ax[2].set_ylabel('Z orientation (deg)')
    ax[2].set_xlabel('Time (s)')
    ax[0].legend()
    ax[1].legend()
    ax[2].legend()

    # Show figure
    fig1.show()

    ######################
    # Trajectory following test
    fig2, ax = plt.subplots(figsize=(10, 11))
    ax.set_aspect('equal')
    ax.set_title('Robot motion')

    # Plot anchor points
    ax.plot(ANCHORS_POS[:, 0], ANCHORS_POS[:, 1], 'kx', label='Anchor points')

    # generate trajectories
    init_state = current_state[test_nr][::10]
    for i in range(len(init_state)):
        positions, _, _, _ = trajectory_generation(init_state[i], target_state[test_nr], 1, np.array([0.01, 0.01, 0.001]))
        ax.plot(positions[0], positions[1], 'lightgray', linestyle=':')

    positions, _, _, _ = trajectory_generation(init_state[0], target_state[test_nr], 1, np.array([0.01, 0.01, 0.001]))
    ax.plot(positions[0], positions[1], 'orange', linestyle='-')

    # make color map for path
    colors = ["green", "green", "cyan", "blue", "magenta", "red"]
    cmap = mcolors.LinearSegmentedColormap.from_list("green_red", colors, N=len(estimated_pose[test_nr][0, :]))

    # Plot path
    for i in range(len(estimated_pose[test_nr][0, :])-2):
        ax.plot(estimated_pose[test_nr][0, i:i+2], estimated_pose[test_nr][1, i:i+2], color=cmap(i))

    # Plot start and end pos
    ax.plot(target_pose[test_nr][0], target_pose[test_nr][1], 'rx', markersize=10, label='Goal pos')
    ax.plot(estimated_pose[test_nr][0, 0], estimated_pose[test_nr][1, 0], 'gx', markersize=10, label='Start pos')
    ax.plot(10, 12, 'lightgray', linestyle=':', label='Trajectories')
    ax.plot(10, 12, 'orange', linestyle='-', label='Initial trajectory')

    # Add labels
    ax.set_title(f'Trajectory result (test {test_nr+1})')
    ax.set_xlabel('CoM X position')
    ax.set_ylabel('CoM Y position')
    ax.set_xlim((np.min(ANCHORS_POS[:, 0])-0.03, np.max(ANCHORS_POS[:, 0])+0.03))
    ax.set_ylim((np.min(ANCHORS_POS[:, 1])-0.03, np.max(ANCHORS_POS[:, 1])+0.03))
    ax.grid()
    ax.legend()

    # Show figure
    fig2.show()

    fig1.savefig(f'ReachGoal{test_nr}.pdf', format='pdf', bbox_inches='tight')
    fig2.savefig(f'TrajectoryFollow{test_nr}.pdf', format='pdf', bbox_inches='tight')

    try:
        plt.waitforbuttonpress()
        plt.close(fig1)
        plt.close(fig2)
    except KeyboardInterrupt as e:
        exit(e)