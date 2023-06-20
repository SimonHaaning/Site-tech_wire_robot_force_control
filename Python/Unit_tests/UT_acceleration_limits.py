# Import necessary packages
import numpy as np
import matplotlib.pyplot as plt
from Python_Functions.TrajectoryGenerationFunctions import quintic_polynomial, acceleration_limited_completion_time
from Python_Functions.PlotterFunctions import trajectory_plotter

# Inputs
start_state = np.array([[-1, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0]], np.float64)
final_state = np.array([[1, 0, 0],
                        [-1, 0, 0],
                        [0, 0, 0]], np.float64)
acceleration_limits = np.array([0.05, 0.05, 0.001])

completion_time = acceleration_limited_completion_time(start_state, final_state, acceleration_limits)
time_resolution = 100

# Generate timetable
time_step_size = 1/time_resolution
times = np.arange(0, completion_time + time_step_size, time_step_size)

# Evaluate polynomials
position, velocity, acceleration = quintic_polynomial(start_state, final_state, times)

trajectory_plotter(position, velocity, acceleration, times)
plt.waitforbuttonpress()
