# workspace_utils.py

import math
import config  # so we can access workspace constants

def cartesian_to_cylindrical(x, y, z):
    """
    Converts Cartesian (x, y, z) to cylindrical coordinates (r, theta_degrees, z).
    r = sqrt(x^2 + y^2)
    theta in degrees, range [0..360).
    """
    r = math.sqrt(x**2 + y**2)
    theta = math.degrees(math.atan2(y, x))
    if theta < 0:
        theta += 360
    return (r, theta, z)

def check_workspace(coordinates):
    """
    Checks whether the given (x, y, z) coordinates lie within the
    robot's configured workspace limits.
    Uses config.WS_R_INNER, config.WS_R_OUTER, config.WS_THETA_INIT, config.WS_THETA_FIN
    """
    x, y, z = coordinates
    r, theta, _ = cartesian_to_cylindrical(x, y, z)
    print("radios!!",r)
    # Check radial range
    if config.WS_R_INNER <= r <= config.WS_R_OUTER:
        # Check angular range
        if config.WS_THETA_INIT <= theta <= config.WS_THETA_FIN:
            return True
    return False
