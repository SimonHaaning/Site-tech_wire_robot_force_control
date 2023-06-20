# Import necessary packages
import numpy as np
import matplotlib.pyplot as plt
from Python_Functions.TrajectoryGenerationFunctions import trajectory_generation
from Python_Functions.PlotterFunctions import trajectory_plotter

# Inputs
start_state = np.array([[-1, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0]], np.float64)
final_state = np.array([[1, 0, 0],
                        [-1, 0, 0],
                        [0, 0, 0]], np.float64)
acceleration_limits = np.array([0.05, 0.05, 0.001])
time_resolution = 1000

position, velocity, acceleration, times = trajectory_generation(start_state, final_state, time_resolution, acceleration_limits)

trajectory_plotter(position, velocity, acceleration, times)
plt.waitforbuttonpress()
