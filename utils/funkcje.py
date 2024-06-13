# funkcje.py

import numpy as np
import matplotlib.pyplot as plt


def generate_trajectory(t, option):
    if option == 'sin':
        return np.sin(t)
    elif option == 'const':
        return np.full_like(t, 1.0)
    elif option == 'poly':
        # TODO
        coefficients = [1, -2, 1]
        trajectory = np.zeros_like(t)
        n = len(coefficients) - 1
        for i, coef in enumerate(coefficients):
            trajectory += coef * t ** (n - i)
        return trajectory
    elif option == 'triangle':
        period = 2
        amplitude = 1
        trajectory = amplitude * (2 * np.abs(2 * (t / period - np.floor(t / period + 0.5))) - 1)
        return trajectory
    else:
        # Default to triangle trajectory
        period = 2
        amplitude = 1
        trajectory = amplitude * (2 * np.abs(2 * (t / period - np.floor(t / period + 0.5))) - 1)
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


class ESO:
    def __init__(self, beta1, beta2, beta3, dt):
        self.beta1 = beta1
        self.beta2 = beta2
        self.beta3 = beta3
        self.dt = dt
        self.x1_hat = 0.0
        self.x2_hat = 0.0
        self.x3_hat = 0.0

    def update(self, measured_value):
        error = measured_value - self.x1_hat
        self.x1_hat += self.dt * (self.x2_hat + self.beta1 * error)
        self.x2_hat += self.dt * (self.x3_hat + self.beta2 * error)
        self.x3_hat += self.dt * (self.beta3 * error)
        return self.x1_hat, self.x2_hat, self.x3_hat


class ADRCController:
    def __init__(self, beta1, beta2, beta3, k1, k2, dt):
        self.eso = ESO(beta1, beta2, beta3, dt)
        self.k1 = k1
        self.k2 = k2

    def calculate(self, setpoint, measured_value, dt):
        x1_hat, x2_hat, x3_hat = self.eso.update(measured_value)
        error = setpoint - x1_hat
        control_signal = self.k1 * error - self.k2 * x2_hat - x3_hat
        return control_signal


class MassSpringDamper:
    def __init__(self, mass, spring_constant, damping_coefficient):
        self.mass = mass
        self.spring_constant = spring_constant
        self.damping_coefficient = damping_coefficient
        self.position = 0
        self.velocity = 0

    def update(self, force, dt):
        acceleration = (force - self.spring_constant * self.position - self.damping_coefficient * self.velocity) / self.mass
        self.velocity += acceleration * dt
        self.position += self.velocity * dt
        return self.position


def plot_results(time, trajectory, pid_output, adrc_output, system_response_pid, system_response_adrc):
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
    plt.plot(time, system_response_pid, label='System Response (PID)')
    plt.plot(time, system_response_adrc, label='System Response (ADRC)')
    plt.title('System Response')
    plt.xlabel('Time (s)')
    plt.ylabel('Position')
    plt.legend()

    plt.tight_layout()
    plt.show()
