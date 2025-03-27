"""
transformations.py

Auxiliary functions for building and manipulating transformation 
matrices (SE(3)) and for converting rotation matrices to axis-angle, etc.
"""

import numpy as np

def get_robot_pose_in_global(x_robot, y_robot, z_robot, yaw_deg):
    """
    Builds T^G_R in SE(3).
    The robot is located at (x_robot, y_robot, z_robot) in {G}
    and rotated 'yaw_deg' degrees around the global Z axis.
    """
    T = np.eye(4)
    yaw_rad = np.deg2rad(yaw_deg)
    Rz = np.array([
        [ np.cos(yaw_rad), -np.sin(yaw_rad), 0],
        [ np.sin(yaw_rad),  np.cos(yaw_rad), 0],
        [               0,                0, 1]
    ])
    T[0:3, 0:3] = Rz
    T[0:3, 3]   = [x_robot, y_robot, z_robot]
    return T

def build_global_pose_from_vector(position, orient_vec):
    """
    Given the desired position of the piece (3x1) and a vector orient_vec (3x1),
    this builds T^G_P in SE(3). We assume orient_vec defines the local z-axis of the piece.

    position:    array-like [px, py, pz]
    orient_vec:  array-like [n1, n2, n3], not necessarily unit
    """
    p = np.array(position, dtype=float).reshape(3)
    n = np.array(orient_vec, dtype=float).reshape(3)

    norm_n = np.linalg.norm(n)
    if norm_n < 1e-9:
        raise ValueError("Orientation vector is nearly zero length.")

    z_axis = n / norm_n  # local z-axis

    # Use an 'up' vector to find a suitable x_axis
    up_global = np.array([0, 0, 1], dtype=float)
    dot_val = abs(np.dot(z_axis, up_global))
    if dot_val > 0.99:
        # If too parallel, pick a different up vector
        up_global = np.array([0, 1, 0], dtype=float)
    x_axis = np.cross(up_global, z_axis)
    x_axis /= np.linalg.norm(x_axis)

    # y_axis = z_axis x x_axis
    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)

    R = np.column_stack((x_axis, y_axis, z_axis))

    T = np.eye(4)
    T[0:3, 0:3] = R
    T[0:3, 3]   = p
    return T

def inv_se3(T):
    """
    Inverts a 4x4 homogeneous transform T in SE(3).
    T = [ R   t ]
        [ 0   1 ]
    => T^-1 = [ R^T  -R^T t ]
              [ 0      1    ]
    """
    R = T[0:3, 0:3]
    t = T[0:3, 3]
    T_inv = np.eye(4)
    T_inv[0:3, 0:3] = R.T
    T_inv[0:3, 3]   = -R.T @ t
    return T_inv

def rotation_matrix_to_axis_angle(R):
    """
    Converts a 3x3 rotation matrix into axis-angle (rx, ry, rz).
    The output (rx, ry, rz) = theta * (unit_axis).
    """
    # Check determinant
    det_R = np.linalg.det(R)
    if abs(det_R - 1.0) > 1e-3:
        print("[WARNING] rotation_matrix_to_axis_angle: determinant not close to 1.")

    trace_val = np.trace(R)
    # Clip within -1..1 for numerical safety
    cos_theta = max(min((trace_val - 1.0) / 2.0, 1.0), -1.0)
    theta = np.arccos(cos_theta)

    if np.isclose(theta, 0.0, atol=1e-8):
        # No significant rotation
        return (0.0, 0.0, 0.0)

    rx = (R[2,1] - R[1,2]) / (2.0*np.sin(theta))
    ry = (R[0,2] - R[2,0]) / (2.0*np.sin(theta))
    rz = (R[1,0] - R[0,1]) / (2.0*np.sin(theta))

    # Multiply by theta
    rx *= theta
    ry *= theta
    rz *= theta

    return (rx, ry, rz)

def get_piece_pose_from_transmitter(xp, yp, zp, xt, yt, zt):
    """
    Example function: given the piece's desired position (xp,yp,zp) 
    and the known transmitter position (xt, yt, zt) in the same global frame,
    returns (pos, orient_vec) to aim the local z-axis of the piece 
    toward the transmitter.
    """
    pos = np.array([xp, yp, zp], dtype=float)
    orient_vec = np.array([xt - xp, yt - yp, zt - zp], dtype=float)
    return pos, orient_vec
