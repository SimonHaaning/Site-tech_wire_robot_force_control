import numpy as np


def inverse_wire_dynamics(forces_desired, wire_directions, moment_arms, max_iter=2000, wire_forces_min=0, wire_forces_max=5000, initial_wire_force=0):
    # Internal Parameters
    F_tolerance = 1       # Accepted deviation from desired force
    M_tolerance = 1  # Accepted deviation from desired moment

    # Initialize motor_forces for first iteration
    motor_forces = np.array([[[initial_wire_force]] * 5] * forces_desired.shape[0])

    # Initialize convergence counter in case it does not converge
    converge_iter = max_iter + 1

    # Calculate the gradients with respect to each motor_forces value
    dF_df = wire_directions
    dM_df = np.cross(moment_arms, wire_directions, axis=2)

    # Arrays for plotting the errors over iterations
    err_array = np.zeros((forces_desired.shape[0], max_iter, 3))

    for i in range(max_iter):
        # Calculate the total force and moment using the current f values
        F, M = forward_wire_dynamics(motor_forces, wire_directions, moment_arms)

        # Calculate error
        F_err = np.subtract(F[:, :2], forces_desired[:, :2])
        M_err = np.subtract(M, forces_desired[:, 2])[:, np.newaxis]
        err_array[:, i, :2] = F_err
        err_array[:, i, 2] = M_err[:, 0]

        # Check for convergence
        if np.all(abs(F_err[:, :2]) < F_tolerance) and np.all(abs(M_err) < M_tolerance):
            converge_iter = i
            break

        motor_forces = update_forces(dF_df, dM_df, F_err, M_err, motor_forces, wire_forces_min, wire_forces_max)

    return motor_forces, F, M, converge_iter, np.concatenate([F_err, M_err], axis=1), err_array


def forward_wire_dynamics(motor_forces, wire_directions, moment_arms):
    """
    Function computes the combined forces and moments from motor forces
    :param motor_forces: `numpy` array of shape (N, 5)
    :param wire_directions: `numpy` array of shape (N, 5, 3)
    :param moment_arms: `numpy` array of shape (N, 5, 3)
    :return:
    """
    wire_forces = np.multiply(motor_forces, wire_directions)
    wire_moments = np.cross(moment_arms, wire_forces)
    wire_forces_sum = np.sum(wire_forces, axis=1)
    wire_moments_sum = np.sum(wire_moments, axis=1)[:, 2]
    return wire_forces_sum, wire_moments_sum


def update_forces(dF_df, dM_df, F_err, M_err, motor_forces, wire_forces_min, wire_forces_max):
    step_size_Fx = 0.1  # Step size (learning rate) for Fx
    step_size_Fy = 0.1  # Step size (learning rate) for Fy
    step_size_Mz = 0.8  # Step size (learning rate) for Mz

    # Compute the gradients
    f_grad_xy = np.multiply(F_err[:, np.newaxis, :2], dF_df[:, :, :2])
    f_grad_z = np.multiply(M_err, dM_df[:, :, 2])

    # Compute the step sizes
    f_step_x = np.multiply(step_size_Fx, f_grad_xy[:, :, 0])
    f_step_y = np.multiply(step_size_Fy, f_grad_xy[:, :, 1])
    f_step_z = np.multiply(step_size_Mz, f_grad_z)

    # Update f values using gradient descent
    motor_forces = np.subtract(motor_forces, np.sum([f_step_x, f_step_y, f_step_z], axis=0)[:, :, np.newaxis])

    # Apply constraints to f values
    motor_forces = np.maximum(wire_forces_min, np.minimum(wire_forces_max, motor_forces))

    return motor_forces
