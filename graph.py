import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import TransferFunction, step
from ipywidgets import interactive
def step_response(J, b, k, Kp, Kd):
"""
Plots the step response of a robotic arm control system using adjustable parameters.
Parameters:
J - Moment of inertia
b - Damping coefficient
k - Spring constant
Kp - Proportional gain
Kd - Derivative gain
"""
num = [1]
den = [J, b + Kd, k + Kp]
try:
system = TransferFunction(num, den)
time, response = step(system)
plt.figure(figsize=(8, 6))
plt.plot(time, response, label=f"J={J}, b={b}, k={k}, Kp={Kp}, Kd={Kd}")
plt.title("Step Response of Robotic Arm with PID Control")
plt.xlabel("Time (seconds)")
plt.ylabel("Angular Position")
plt.grid()
plt.legend()
plt.show()
except Exception as e:
print(f"Error in generating transfer function: {e}")
interactive_plot = interactive(
step_response,
J=(0.1, 5.0, 0.1),
b=(0.1, 10.0, 0.1),
k=(0.1, 10.0, 0.1),
Kp=(1.0, 50.0, 1.0),
Kd=(0.1, 10.0, 0.1)
)
interactive_plot
