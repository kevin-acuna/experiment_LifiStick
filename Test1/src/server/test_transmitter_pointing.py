#!/usr/bin/env python
"""
Test program for the compute_pose_pointed_to_transmitter function
with specific transmitter and receiver positions.
"""

import numpy as np
import sys
import os

# Add the current directory to the path to ensure imports work correctly
#sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Import the function and config
from transformations import compute_pose_pointed_to_transmitter, rotation_matrix_to_axis_angle
import config

# Override config values with our test values
print("Transmitter position:", 
      config.TRANSMITTER_X, config.TRANSMITTER_Y, config.TRANSMITTER_Z)

# Set transmitter at position (0.5, 0.5, Z)
config.ROBOT_GLOBAL_X = 0.5
config.ROBOT_GLOBAL_Y = 0.5

# Display robot configuration
print("\nRobot configuration:")
print(f"Position: ({config.ROBOT_GLOBAL_X}, {config.ROBOT_GLOBAL_Y}, {config.ROBOT_GLOBAL_Z})")
print(f"Orientation: {config.ROBOT_YAW_DEG} degrees")
print(f"Piece height: {config.PIECE_HEIGHT} meters")

# Test the function with piece at (0.4, 0, 0.96)
piece_x = 0.8
piece_y = 0
piece_z = 0.96

print("\nTesting with piece position:", piece_x, piece_y, piece_z)

# Call the function
pos_robot, R_robot = compute_pose_pointed_to_transmitter(piece_x, piece_y, piece_z)

# Print results
print("\nResults:")
print("Robot position vector (x, y, z):")
print(pos_robot)

print("\nRobot orientation matrix (3x3):")
for row in R_robot:
    print(f"[{row[0]:8.5f}, {row[1]:8.5f}, {row[2]:8.5f}]")


print("ANALISIS ***********************************")
R = R_robot
# Check determinant
det_R = np.linalg.det(R)
if abs(det_R - 1.0) > 1e-3:
      print("[WARNING] rotation_matrix_to_axis_angle: determinant not close to 1.")

trace_val = np.trace(R)
print(trace_val)

# Clip within -1..1 for numerical safety
cos_theta = max(min((trace_val - 1.0) / 2.0, 1.0), -1.0)
theta = np.arccos(cos_theta)

print(theta*180/np.pi)
if np.isclose(theta, 0.0, atol=1e-8):
      # No significant rotation
      print("iscloseeee !! ")
      #return (0.0, 0.0, 0.0)

print((R[2,1] - R[1,2]))
print(1/(2.0*np.sin(theta)))
rx = (R[2,1] - R[1,2]) / (2.0*np.sin(theta))
ry = (R[0,2] - R[2,0]) / (2.0*np.sin(theta))
rz = (R[1,0] - R[0,1]) / (2.0*np.sin(theta))

print(rx, ry, rz)
# Multiply by theta
rx *= theta
ry *= theta
rz *= theta

print(rx, ry, rz)












# Convert rotation matrix to axis-angle for easier interpretation
axis_angle = rotation_matrix_to_axis_angle(R_robot)
print("\nRobot orientation as axis-angle (rx, ry, rz):")
print(f"({axis_angle[0]:8.5f}, {axis_angle[1]:8.5f}, {axis_angle[2]:8.5f})")


'''


# Calculate the vector from piece to transmitter (for verification)
transmitter_pos = np.array([config.TRANSMITTER_X, config.TRANSMITTER_Y, config.TRANSMITTER_Z])
piece_pos = np.array([piece_x, piece_y, piece_z])
vector_to_transmitter = transmitter_pos - piece_pos
vector_to_transmitter_normalized = vector_to_transmitter / np.linalg.norm(vector_to_transmitter)

print("\nVerification:")
print("Vector from piece to transmitter (normalized):")
print(vector_to_transmitter_normalized)

# Extract the z-axis (3rd column) from the rotation matrix
z_axis_robot = R_robot[:, 2]  # Third column represents the z-axis direction
print("\nRobot end-effector z-axis (from rotation matrix):")
print(z_axis_robot)

# Calculate dot product to measure alignment (1.0 = perfect alignment, -1.0 = opposite direction)
alignment = np.dot(z_axis_robot, vector_to_transmitter_normalized)
print("\nAlignment between robot z-axis and vector to transmitter:")
print(f"Dot product: {alignment:.6f} (1.0 = perfect alignment)")

# Calculate the angle between the two vectors in degrees
angle_rad = np.arccos(np.clip(alignment, -1.0, 1.0))  # Clip to handle numerical errors
angle_deg = np.degrees(angle_rad)
print(f"Angle between vectors: {angle_deg:.2f} degrees")

# Evaluate alignment quality
if abs(alignment - 1.0) < 1e-5:
    print("PERFECT: Robot z-axis perfectly aligned with vector to transmitter!")
elif alignment > 0.9999:
    print("EXCELLENT: Robot z-axis very closely aligned with vector to transmitter.")
elif alignment > 0.999:
    print("GOOD: Robot z-axis well aligned with vector to transmitter.")
elif alignment > 0.99:
    print("ACCEPTABLE: Robot z-axis reasonably aligned with vector to transmitter.")
else:
    print("WARNING: Robot z-axis not well aligned with vector to transmitter.")
'''




