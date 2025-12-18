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

def check_workspace(coordinates, piece_height=None):
    """
    Checks whether the given (x, y, z) coordinates lie within the
    robot's configured workspace limits.
    Uses height-dependent radii if piece_height is provided and matches a configured height.
    """
    x, y, z = coordinates
    r, theta, _ = cartesian_to_cylindrical(x, y, z)
    
    r_inner = config.WS_R_INNER
    r_outer = config.WS_R_OUTER
    
    if piece_height is not None and piece_height in config.WS_HEIGHT_RADII:
        r_inner = config.WS_HEIGHT_RADII[piece_height]['inner']
        r_outer = config.WS_HEIGHT_RADII[piece_height]['outer']
    
    print(f"Height: {piece_height}, Inner: {r_inner}, Outer: {r_outer}, Radial: {r}")
    
    if r_inner <= r <= r_outer:
        if config.WS_THETA_INIT <= theta <= config.WS_THETA_FIN:
            return True
    return False
