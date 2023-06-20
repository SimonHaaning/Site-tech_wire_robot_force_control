import numpy as np
from Python_Functions.KinematicsFunctions import inverse_wire_kinematics
from Python_Functions.DynamicsFunctions import forward_wire_dynamics


class RobotMotionSimulator:
    def __init__(self, initial_state, anchor_positions, motor_positions):
        self.current_positions = initial_state[:, 0]
        self.current_velocities = initial_state[:, 1]
        self.current_accelerations = initial_state[:, 2]

        # Geometric parameters
        self._ANCHOR_POINTS = anchor_positions
        self._MOTOR_POINTS = motor_positions

        # Inertial parameters
        self._ROBOT_MASS = 100  # kg
        self._ROBOT_INERTIA = 101.85669  # kg*m^2

        self.motion_steps = 0

        # External forces
        self._FORCE_GRAVITY = np.array([0, -9.82*self._ROBOT_MASS, 0])

    def simulate_step(self, motor_forces, time_step_size):
        # Determine wire directions and moment arms
        com_position = np.array([[self.current_positions[0], self.current_positions[1], 0]])
        frame_angle = np.array([self.current_positions[2]])
        wire_directions, _, moment_arms = inverse_wire_kinematics(com_position, frame_angle,
                                                                  self._ANCHOR_POINTS, self._MOTOR_POINTS)

        # Compute sum of forces and moments
        forces_sum, moments_sum = forward_wire_dynamics(motor_forces, wire_directions, moment_arms)

        # Add external forces
        forces_sum = forces_sum + self._FORCE_GRAVITY
        forces_sum = forces_sum + self.calc_friction(forces_sum)

        # Compute acceleration
        self.current_accelerations = np.array([forces_sum[0, 0]/self._ROBOT_MASS,
                                               forces_sum[0, 1]/self._ROBOT_MASS,
                                               moments_sum[0]/self._ROBOT_INERTIA])

        # Update position and velocity
        self.current_positions += self.current_velocities * time_step_size + np.sign(self.current_accelerations) * 0.5*self.current_accelerations * time_step_size**2
        self.current_velocities += self.current_accelerations * time_step_size

        self.motion_steps += 1

        #print(f"motor_forces: {motor_forces}")
        #print(f"acceleration: {self.current_accelerations}")
        #print(f"velocity: {self.current_velocities}")
        #print(f"position: {self.current_positions}\n")

    def simulate_motion(self, motor_forces_trajectory, times):
        if len(motor_forces_trajectory) != len(times):
            print(f"Error: Cannot simulate motion as length of motor_forces_trajectory ({len(motor_forces_trajectory)}) and times ({len(times)}) do not match.")
            return

        # Compute time step size
        time_step_size = times[1] - times[0]

        # Iterate over all given positions
        for i in range(len(times)):
            self.simulate_step(motor_forces_trajectory[i, :, :], time_step_size)

    def calc_friction(self, other_forces):
        # Friction parameters TODO: Model these from real data
        STATIC_FRICTION_THRESHOLD = 0.0001
        STATIC_FRICTION = 0.0
        DYNAMIC_FRICTION = 0.0

        # Determine if moving
        if np.any(np.abs(self.current_velocities) < STATIC_FRICTION_THRESHOLD):
            friction = np.where(np.abs(STATIC_FRICTION) < np.abs(other_forces[0]), -1 * np.sign(self.current_velocities) * STATIC_FRICTION, -1 * other_forces[0])
            #print(f"Static friction: {friction}")
        else:
            friction = np.where(np.abs(self.current_velocities * DYNAMIC_FRICTION) < np.abs(other_forces[0]), -1 * self.current_velocities * DYNAMIC_FRICTION, -1 * other_forces[0])
            #print(f"Dynamic friction: {friction}")

        return friction
