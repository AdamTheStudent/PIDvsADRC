import .utils.funkcje
def main():
    # Simulation parameters
    dt = 0.01
    t = np.arange(0, 10, dt)

    # Generate trajectory
    trajectory = generate_trajectory(t)

    # Initialize controllers and system
    pid = PIDController(kp=1.0, ki=0.1, kd=0.01)
    adrc = ADRCController(beta1=1.0, beta2=0.5, beta3=0.1)
    system = MassSpringDamper(mass=1.0, spring_constant=1.0, damping_coefficient=0.2)

    # Prepare arrays to store results
    pid_output = np.zeros_like(t)
    adrc_output = np.zeros_like(t)
    system_response = np.zeros_like(t)

    # Simulation loop
    for i in range(1, len(t)):
        # Get control signals
        pid_control = pid.calculate(trajectory[i], system.position, dt)
        adrc_control = adrc.calculate(trajectory[i], system.position, dt)

        # Use one of the controllers to update the system (alternatively, simulate both)
        force = pid_control  # or adrc_control

        # Update system dynamics
        system_response[i] = system.update(force, dt)

        # Store control outputs
        pid_output[i] = pid_control
        adrc_output[i] = adrc_control

    # Plot results
    plot_results(t, trajectory, pid_output, adrc_output, system_response)


if __name__ == '__main__':
    main()