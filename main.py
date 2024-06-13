# main.py
import numpy as np
from utils.funkcje import generate_trajectory, PIDController, ADRCController, MassSpringDamper, plot_results


def main():
    # Simulation parameters
    dt = 0.01
    t = np.arange(0, 10, dt)

    # choose trajectory
    # trajectory = generate_trajectory(t, 'sin')  # sinus
    # Generate trajectory
    trajectory = generate_trajectory(t, 'const')
    # trajectory = generate_trajectory(t, 'poly')  # polynomial trajectory
    # trajectory = generate_trajectory(t, 'triangle')  # triangle trajectory

    # Initialize controllers and system
    pid = PIDController(kp=5.0, ki=1.5, kd=0.5)  # Optymalizowane parametry PID
    adrc = ADRCController(beta1=30, beta2=300, beta3=1000, k1=50, k2=2, dt=dt)


    # Create separate systems for PID and ADRC
    system_pid = MassSpringDamper(mass=1.0, spring_constant=1.0, damping_coefficient=0.3)
    system_adrc = MassSpringDamper(mass=1.0, spring_constant=1.0, damping_coefficient=0.3)

    # Prepare arrays to store results
    pid_output = np.zeros_like(t)
    adrc_output = np.zeros_like(t)
    system_response_pid = np.zeros_like(t)
    system_response_adrc = np.zeros_like(t)

    # Simulation loop
    for i in range(1, len(t)):
        # Get control signals
        pid_control = pid.calculate(trajectory[i], system_pid.position, dt)
        adrc_control = adrc.calculate(trajectory[i], system_adrc.position, dt)

        # Update system dynamics
        system_response_pid[i] = system_pid.update(pid_control, dt)
        system_response_adrc[i] = system_adrc.update(adrc_control, dt)

        # Store control outputs
        pid_output[i] = pid_control
        adrc_output[i] = adrc_control

    # Plot results
    plot_results(t, trajectory, pid_output, adrc_output, system_response_pid, system_response_adrc,dt=dt)


if __name__ == '__main__':
    main()
