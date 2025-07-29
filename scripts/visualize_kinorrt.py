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
   This will generate two files:
       - path.csv: List of [x, y, theta] for the robot path
       - obstacles.csv: List of [x, y, radius] for each obstacle

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

# Load obstacle data: each row is [x, y, radius]
obstacles = pd.read_csv("obstacles.csv", header=None, names=["x", "y", "radius"])

# Create plot
fig, ax = plt.subplots(figsize=(8, 6))

# Plot the simplified path as a blue line
ax.plot(path["x"], path["y"], 'b-', label="Simplified Path")

# Plot robot heading (orientation) as red arrows using quiver
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
