from vpython import sphere, cylinder, vector, label, rate, scene, slider, wtext, button, color
import time
import math
scene.title = "3D Robotic Arm Simulation with Stability Analysis and Time Recording"
scene.background = vector(0.8, 0.8, 0.8)
base = cylinder(pos=vector(0, 0, 0), axis=vector(0, 1, 0), radius=0.2, color=vector(0.7, 0.7, 0.7))
joint = sphere(pos=vector(0, 1, 0), radius=0.1, color=vector(0, 0.5, 1))
arm = cylinder(pos=joint.pos, axis=vector(1, 0, 0), radius=0.05, color=vector(1, 0, 0))
target = sphere(pos=vector(2, 1.5, 0), radius=0.1, color=vector(0, 1, 0))
# System Parameters
J = 1.0 # Moment of inertia
b = 0.5 # Damping coefficient
k = 2.0 # Spring constant
# Gains (adjustable via sliders)
K_p = 1.0
K_d = 0.5
current_pos = joint.pos + arm.axis
previous_error = vector(0, 0, 0)
dt = 0.01
tolerance = 0.05
current_label = label(pos=vector(-3, 3, 0), text="Current Position: ", height=12, color=vector(0, 0, 0))
target_label = label(pos=vector(-3, 2.5, 0), text="Target Position: ", height=12, color=vector(0, 0, 0))
stability_label = label(pos=vector(-3, 2, 0), text="Stability: ", height=12, color=vector(0, 0, 0))
def set_new_target(evt):
global target, start_time, is_reaching
target.pos = evt.pos
start_time = time.time()
is_reaching = True
scene.bind('click', set_new_target)
def update_kp(s):
global K_p
K_p = s.value
def update_kd(s):
global K_d
K_d = s.value
kp_slider = slider(min=0.1, max=5, value=K_p, length=300, bind=update_kp, right=15)
kp_text = wtext(text=f"K_p: {K_p:.2f}")
kd_slider = slider(min=0.1, max=5, value=K_d, length=300, bind=update_kd, right=15)
kd_text = wtext(text=f"K_d: {K_d:.2f}")
results = []
result_label = label(pos=vector(-3, -3, 0), text="Results Table:\n", height=12, color=vector(0, 0, 0))
def update_results_table():
table_text = "Results Table:\n"
for i, (kp, kd, time_taken) in enumerate(results):
table_text += f"{i+1}. K_p: {kp:.2f}, K_d: {kd:.2f}, Time: {time_taken:.2f}s\n"
result_label.text = table_text
def clear_results():
global results
results = []
update_results_table()
clear_button = button(text="Clear Table", pos=scene.title_anchor, bind=lambda _: clear_results())
start_time = None
is_reaching = False
while True:
rate(100)
kp_text.text = f"K_p: {K_p:.2f}"
kd_text.text = f"K_d: {K_d:.2f}"
error = target.pos - current_pos
derivative = (error - previous_error) / dt
# Laplace Transfer Function Control Signal
control_signal = (1 / (J * dt**2 + (b + K_d) * dt + (k + K_p))) * error
arm.axis += control_signal * dt
current_pos = joint.pos + arm.axis
# Stability Analysis
discriminant = (b + K_d)**2 - 4 * J * (k + K_p)
if discriminant > 0:
stability_type = "Overdamped"
pole1 = (-b - K_d + math.sqrt(discriminant)) / (2 * J)
pole2 = (-b - K_d - math.sqrt(discriminant)) / (2 * J)
elif discriminant == 0:
stability_type = "Critically Damped"
pole1 = pole2 = (-b - K_d) / (2 * J)
else:
stability_type = "Underdamped"
real_part = (-b - K_d) / (2 * J)
imaginary_part = math.sqrt(-discriminant) / (2 * J)
pole1 = f"{real_part:.2f} + {imaginary_part:.2f}i"
pole2 = f"{real_part:.2f} - {imaginary_part:.2f}i"
current_label.text = f"Current Position: {current_pos.x:.2f}, {current_pos.y:.2f}, {current_pos.z:.2f}"
target_label.text = f"Target Position: {target.pos.x:.2f}, {target.pos.y:.2f}, {target.pos.z:.2f}"
stability_label.text = f"Stability: {stability_type}, Poles: {pole1}, {pole2}"
if is_reaching and error.mag < tolerance:
time_taken = time.time() - start_time
results.append((K_p, K_d, time_taken))
update_results_table()
is_reaching = False
previous_error = error
