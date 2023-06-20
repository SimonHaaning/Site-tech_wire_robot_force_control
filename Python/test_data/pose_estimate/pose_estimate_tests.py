import numpy as np
import matplotlib.pyplot as plt
from test_data.test_data_reader import reader_function

a1 = [0.0, 3.355, -0.20646]
a2 = [1.45, 3.355, -0.20646]
a3 = [3.08 - 0.240, 3.355 - 0.065, -0.20646]
a4 = [2.9, 0.0, -0.20646]
a5 = [0.0, 0.0, -0.20646]
ANCHORS_POS = np.array([a1, a2, a3, a4, a5])  # 5x3 matrix

# Locations measured on wall (m) and angle measured with level (degrees)
targets = [[1.06, 1.94, 3.8],
           [1.89, 1.71, 7.1],
           [1.55, 1.70, 6.1],
           [1.48, 1.69, -1.9],
           [1.39, 1.46, 10.8],
           [1.36, 1.87, 3.6],
           [1.05, 0.92, 1.1],
           [1.62, 0.915, -2.0],
           [1.295, 1.20, -2.1],
           [1.65, 1.685, 11.3],
           [1.44, 1.25, 5.3]]
targets = np.array(targets)

estimated_pose = []
# Read CSV for recorded data
for i in range(10):
    data = reader_function(str(i+1)+'/', 'current_state.csv')
    x = data[:, 0, 0]
    y = data[:, 1, 0]
    z = np.rad2deg(data[:, 2, 0] + 0.12)  # Offset was 0.12 during test
    estimated_pose.append(np.array([x, y, z]))

# Create plot
fig, ax = plt.subplots(figsize=(10, 12))
ax.set_aspect('equal')
ax.set_title('Position estimate test results')
marker_colors = ['red', 'blue', 'green', 'orange', 'purple', 'olive', 'teal', 'navy', 'magenta', 'lime', 'brown']

# plot anchor points
ax.plot(ANCHORS_POS[:, 0], ANCHORS_POS[:, 1], 'kx', label='Anchor points')

for i in range(10):
    # Plot target
    ax.plot(targets[i, 0], targets[i, 1], '+', markersize=15, color=marker_colors[i], label=f'Pos {i + 1}')

for i in range(10):
    # Plot all measurements
    ax.plot(estimated_pose[i][0, :], estimated_pose[i][1, :], '.', markersize=3, color=marker_colors[i])  # , label=f'Measurement')

    # Compute mean measurement
    x_mean = np.mean(estimated_pose[i][0, :])
    y_mean = np.mean(estimated_pose[i][1, :])
    z_mean = np.mean(estimated_pose[i][2, :])

    # Plot line from mean to target
    ax.plot([targets[i, 0], x_mean], [targets[i, 1], y_mean], linestyle='--', color=marker_colors[i])

    print(f'Test {i + 1} mean deviations:\n'
          f'X: {x_mean - targets[i, 0]:.3f} m\n'
          f'Y: {y_mean - targets[i, 1]:.3f} m\n'
          f'Z: {z_mean - targets[i, 2]:.3f} degrees')

for i in range(10):
    # Plot target again to put on top
    ax.plot(targets[i, 0], targets[i, 1], '+', markersize=15, color=marker_colors[i])

# Add invisible things to legend
ax.plot(10, 10, '.', markersize=3, color='black', label='Estimated pos')
ax.plot([10, 20], [10, 20], linestyle='--', color='black', label='Deviation to')
ax.plot([10, 20], [10, 20], linestyle='--', color='white', label='mean estimate')

ax.set_xlabel('CoM X position')
ax.set_ylabel('CoM Y position')
ax.set_xlim((np.min(ANCHORS_POS[:, 0])-0.1, np.max(ANCHORS_POS[:, 0])+0.1))
ax.set_ylim((np.min(ANCHORS_POS[:, 1])-0.1, np.max(ANCHORS_POS[:, 1])+0.1))
ax.grid()
ax.legend(ncol=1, loc='upper left', bbox_to_anchor=(0, 0.96))

fig.savefig("PositionEstimateAccuracy.pdf", format='pdf', bbox_inches='tight')
fig.show()
try:
    plt.waitforbuttonpress()
except KeyboardInterrupt as e:
    exit(e)
