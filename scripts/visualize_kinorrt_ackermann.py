"""
Visualize Kino-RRT Path and Obstacles for Ackermann Robot

This script visualizes the output of the `vamp_kinorrt_example` C++ program, including:
- The simplified path produced by Kino-RRT
- Robot heading at each path point
- Obstacles from the environment
- A circular robot trace (radius = 0.2 m) along the path

🚨 Usage Instructions:
1. First, compile and run the C++ planner:
       ./vamp_kinorrt_example
   This will generate three files:
       - path.csv: List of [x, y, theta] for the robot path
       - obstacles.csv: List of [x, y, radius] for each obstacle
       - control_inputs.csv: List of [speed, steering_angle, duration] for each path segment

2. Then run this script:
       python visualize_kinorrt.py

🖼️ Requirements:
- Python with matplotlib, numpy, pandas
- GUI support (or use `%matplotlib inline` in Jupyter)
"""

import pandas as pd
import numpy as np
import matplotlib
matplotlib.use("TkAgg")  # You can use 'Qt5Agg' if preferred or available
import matplotlib.pyplot as plt

# Load path data: each row is [x, y, theta]
path = pd.read_csv("path.csv", header=None, names=["x", "y", "theta"])

# Load control inputs: each row is [speed, steering_angle, duration]
controls = pd.read_csv("controls.csv", header=None, names=["speed", "steering_angle", "duration"])

# Load obstacle data: each row is [x, y, radius]
obstacles = pd.read_csv("obstacles.csv", header=None, names=["x", "y", "radius"])

def simulate_ackermann_segment(x0, y0, theta0, delta, v, L=0.1, dt=0.001, duration=1.0):
    """
    Simulate a single Ackermann segment given control inputs.
    
    Params:
    - x0, y0, theta0: initial pose
    - delta: steering angle (radians)
    - v: linear velocity (m/s)
    - L: wheelbase (m)
    - dt: timestep for integration
    - duration: time duration of segment (s)
    
    Returns:
    - xs, ys: arrays of points along the trajectory
    """
    xs = [x0]
    ys = [y0]
    theta = theta0
    
    steps = int(duration / dt)
    x, y = x0, y0
    
    for _ in range(steps):
        x += v * np.cos(theta) * dt
        y += v * np.sin(theta) * dt
        theta += v / L * np.tan(delta) * dt
        xs.append(x)
        ys.append(y)
    
    return np.array(xs), np.array(ys)

# Plot setup
fig, ax = plt.subplots(figsize=(8, 6))

# Wheelbase length of the Ackermann robot (adjust if needed)
wheelbase = 0.1

# Plot the path by simulating each segment with controls
for i in range(len(path) - 1):
    x0, y0, theta0 = path.iloc[i]
    speed = controls.iloc[i]['speed']
    steering_angle = controls.iloc[i]['steering_angle']
    duration = controls.iloc[i]['duration']
    
    xs, ys = simulate_ackermann_segment(x0, y0, theta0, steering_angle, speed, L=wheelbase, dt=0.01, duration=duration)
    ax.plot(xs, ys, 'b-', label="Ackermann Path" if i == 0 else None)

# Plot robot heading (orientation) as red arrows at path points
ax.quiver(
    path["x"], path["y"],
    np.cos(path["theta"]), np.sin(path["theta"]),
    scale=10, width=0.003, color='r', label="Heading"
)

# Draw robot trace: circles with radius = 0.2 at each (x, y)
robot_radius = 0.2
for x, y in zip(path["x"], path["y"]):
    circle = plt.Circle((x, y), robot_radius, color='blue', alpha=0.1, edgecolor='black', linewidth=0.3)
    ax.add_patch(circle)

# Plot obstacles as gray semi-transparent circles
for _, row in obstacles.iterrows():
    obs = plt.Circle((row["x"], row["y"]), row["radius"], color='gray', alpha=0.5)
    ax.add_patch(obs)

# Add index numbers at each node position
for i, (x, y) in enumerate(zip(path["x"], path["y"])):
    ax.text(x, y, str(i), fontsize=8, color='black', verticalalignment='bottom', horizontalalignment='right')
    
# Set labels, grid, legend, etc.
ax.set_aspect('equal')  # Ensures 1:1 aspect ratio
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_title("Kino-RRT Ackermann Path with Robot Trace and Obstacles")
ax.grid(True)
ax.legend()
plt.tight_layout()

# Show the plot window
plt.show()
