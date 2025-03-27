"""
config.py

Global configuration and constants for the robot setup.
"""

# Robot connection settings
ROBOT_HOST = "193.51.28.59"  # Robot's IP address
ROBOT_PORT = 30002          # Robot's port

# Motion parameters
ACCELERATION = 1.0
VELOCITY = 0.1
MOVE_TIME = 5  # Estimated movement time

# Robot base position in the global frame
ROBOT_GLOBAL_X = -0.5
ROBOT_GLOBAL_Y = -0.5
ROBOT_GLOBAL_Z = 0.782

# Robot yaw orientation in degrees (around global Z)
ROBOT_YAW_DEG = 90

# Transmitter position (example: 2 meters above global origin)
TRANSMITTER_X = 0.0
TRANSMITTER_Y = 0.0
TRANSMITTER_Z = 2.0

# Height of the piece (end-effector extension)
PIECE_HEIGHT = 0.035

# Workspace 
WS_R_INNER = 0.40 # Inner radius of the workspace
WS_R_OUTER = 0.90 # Outer radius of the workspace
WS_THETA_INIT = 86 # Initial angle of the workspace
WS_THETA_FIN = 184 # Final angle of the workspace
