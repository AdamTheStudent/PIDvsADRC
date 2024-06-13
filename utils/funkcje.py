import numpy as np

def generate_trajectory(t):
    # Example: Sine wave trajectory
    trajectory = np.sin(t)
    return trajectory


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.previous_error = 0

    def calculate(self, setpoint, measured_value, dt):
        error = setpoint - measured_value
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output


# ADRC implementation is more complex and usually problem-specific
# Here is a simple placeholder implementation
class ADRCController:
    def __init__(self, beta1, beta2, beta3):
        self.beta1 = beta1
        self.beta2 = beta2
        self.beta3 = beta3

    def calculate(self, setpoint, measured_value, dt):
        # Placeholder: Simple PD control for demonstration
        error = setpoint - measured_value
        output = self.beta1 * error - self.beta2 * (measured_value / dt)
        return output


class MassSpringDamper:
    def __init__(self, mass, spring_constant, damping_coefficient):
        self.mass = mass
        self.spring_constant = spring_constant
        self.damping_coefficient = damping_coefficient
        self.position = 0
        self.velocity = 0

    def update(self, force, dt):
        acceleration = (
                                   force - self.spring_constant * self.position - self.damping_coefficient * self.velocity) / self.mass
        self.velocity += acceleration * dt
        self.position += self.velocity * dt
        return self.position


import matplotlib.pyplot as plt


def plot_results(time, trajectory, pid_output, adrc_output, system_response):
    plt.figure(figsize=(12, 8))

    plt.subplot(3, 1, 1)
    plt.plot(time, trajectory, label='Desired Trajectory')
    plt.title('Desired Trajectory')
    plt.xlabel('Time (s)')
    plt.ylabel('Position')
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(time, pid_output, label='PID Output')
    plt.plot(time, adrc_output, label='ADRC Output')
    plt.title('Control Signals')
    plt.xlabel('Time (s)')
    plt.ylabel('Control Signal')
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(time, system_response, label='System Response')
    plt.title('System Response')
    plt.xlabel('Time (s)')
    plt.ylabel('Position')
    plt.legend()

    plt.tight_layout()
    plt.show()