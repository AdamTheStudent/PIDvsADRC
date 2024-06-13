import numpy as np
from utils.funkcje import generate_trajectory, PIDController, ADRCController, MassSpringDamper, plot_results,\
    calculate_quality_indices


def main():
    # Configuration parameters
    dt = 0.01
    simulation_time = 10
    t = np.arange(0, simulation_time, dt)

    # Trajectory configuration
    trajectory_type = 'const'  # Options: 'sin', 'const', 'poly', 'triangle'
    const_value = 1.0  # for 'const'
    poly_coefficients = [1, -2, 1]  # for 'poly'
    triangle_period = 2  # for 'triangle'
    triangle_amplitude = 1  # for 'triangle'

    # Controller parameters
    pid_params = {'kp': 5.0, 'ki': 1.5, 'kd': 0.5}
    adrc_params = {'beta1': 30, 'beta2': 300, 'beta3': 1000, 'k1': 50, 'k2': 2, 'dt': dt}

    # Initial system parameters
    initial_mass = 1.0
    initial_spring_constant = 1.0
    initial_damping_coefficient = 0.3

    # Changed system parameters
    change_time = 5.0
    changed_mass = 1.0
    changed_spring_constant = 1.0

    # ==================================================================================================================

    # Generate trajectory
    trajectory = generate_trajectory(t, trajectory_type, const_value, poly_coefficients, triangle_period,
                                     triangle_amplitude)

    # Initialize controllers
    pid = PIDController(**pid_params)
    adrc = ADRCController(**adrc_params)

    # Create separate systems for PID and ADRC
    system_pid = MassSpringDamper(initial_mass, initial_spring_constant, initial_damping_coefficient)
    system_adrc = MassSpringDamper(initial_mass, initial_spring_constant, initial_damping_coefficient)

    # Prepare arrays to store results
    pid_output = np.zeros_like(t)
    adrc_output = np.zeros_like(t)
    system_response_pid = np.zeros_like(t)
    system_response_adrc = np.zeros_like(t)

    # Simulation loop
    for i in range(1, len(t)):
        current_time = t[i]
        # Update system parameters if current_time >= change_time
        if current_time >= change_time:
            system_pid.mass = changed_mass
            system_pid.spring_constant = changed_spring_constant
            system_adrc.mass = changed_mass
            system_adrc.spring_constant = changed_spring_constant

        # Get control signals
        pid_control = pid.calculate(trajectory[i], system_pid.position, dt)
        adrc_control = adrc.calculate(trajectory[i], system_adrc.position, dt)

        # Update system dynamics
        system_response_pid[i] = system_pid.update(pid_control, dt, current_time)
        system_response_adrc[i] = system_adrc.update(adrc_control, dt, current_time)

        # Store control outputs
        pid_output[i] = pid_control
        adrc_output[i] = adrc_control

    # Calculate quality indices
    IAE_pid, ITAE_pid, ISE_pid, ITSE_pid = calculate_quality_indices(t, trajectory, system_response_pid)
    IAE_adrc, ITAE_adrc, ISE_adrc, ITSE_adrc = calculate_quality_indices(t, trajectory, system_response_adrc)

    # Print quality indices
    print(f'PID Controller Quality Indices:'
          f'\nIAE: {IAE_pid:.4f}, ITAE: {ITAE_pid:.4f}, ISE: {ISE_pid:.4f}, ITSE: {ITSE_pid:.4f}')
    print(f'ADRC Controller Quality Indices:'
          f'\nIAE: {IAE_adrc:.4f}, ITAE: {ITAE_adrc:.4f}, ISE: {ISE_adrc:.4f}, ITSE: {ITSE_adrc:.4f}')

    # Plot results
    plot_results(t, trajectory, pid_output, adrc_output, system_response_pid, system_response_adrc, dt, initial_mass,
                 initial_spring_constant, initial_damping_coefficient, changed_mass, changed_spring_constant)


if __name__ == '__main__':
    main()
