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
MOVE_TIME = 8  # Estimated movement time

# Robot base position in the global frame
ROBOT_GLOBAL_X = 0
ROBOT_GLOBAL_Y = 0
ROBOT_GLOBAL_Z = 0.646  #0.782 (with wheels)

# Robot yaw orientation in degrees (around global Z)
ROBOT_YAW_DEG = 90

# Transmitter position (example: 2 meters above global origin)
TRANSMITTER_X = 0.0
TRANSMITTER_Y = 0.0
TRANSMITTER_Z = 2.0

# Height of the piece (end-effector extension in meters)
PIECE_HEIGHT = 0.037 # previous piece

# Maximum inclination angle for random orientation (in degrees)
THETA_MAX = 15  # 0 = vertical, max inclination from vertical

# Workspace 
# Cuando se usa beamstearing en el UE y el robot tiene que moverse tmb
#WS_R_INNER = 0.55 # Inner radius of the workspace
#WS_R_OUTER = 0.77 # Outer radius of the workspace
#WS_THETA_INIT = 80 # Initial angle of the workspace
#WS_THETA_FIN = 190 # Final angle of the workspace

WS_R_INNER = 0.65 # Default inner radius of the workspace
WS_R_OUTER = 0.85 # Default outer radius of the workspace
WS_THETA_INIT = 90 # Initial angle of the workspace:190
WS_THETA_FIN = 315 # Final angle of the workspace

# Height-dependent workspace radii
WS_HEIGHT_RADII = {
    0.4: {'inner': 0.64, 'outer': 0.74},
    0.6: {'inner': 0.62, 'outer': 0.82},
    0.8: {'inner': 0.50, 'outer': 0.86},
    1.0: {'inner': 0.44, 'outer': 0.84},
    1.2: {'inner': 0.40, 'outer': 0.80}
}