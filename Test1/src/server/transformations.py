"""
transformations.py

Auxiliary functions for building and manipulating transformation 
matrices (SE(3)) and for converting rotation matrices to axis-angle, etc.
"""

import numpy as np
import config  

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


def compute_pose_pointed_to_transmitter(piece_x, piece_y, piece_z):
    """
    1) Build T^G_P by aiming local z-axis toward the transmitter.
    2) Compute T^R_E for the robot.
    3) Return (pos_robot, R_robot).
    """
    # 1) Build T^G_P
    pos_piece, orient_piece = get_piece_pose_from_transmitter(
        piece_x, piece_y, piece_z,
        config.TRANSMITTER_X,
        config.TRANSMITTER_Y,
        config.TRANSMITTER_Z
    )
    T_G_P = build_global_pose_from_vector(pos_piece, orient_piece)

    # 2) Robot pose in global
    T_G_R = get_robot_pose_in_global(
        config.ROBOT_GLOBAL_X,
        config.ROBOT_GLOBAL_Y,
        config.ROBOT_GLOBAL_Z,
        config.ROBOT_YAW_DEG
    )

    # 3) T^E_P: piece offset from end-effector
    T_E_P = np.eye(4)
    T_E_P[2, 3] = config.PIECE_HEIGHT

    # 4) T^R_E = (T^G_R)^-1 * T^G_P * (T^E_P)^-1
    T_R_E = inv_se3(T_G_R) @ T_G_P @ inv_se3(T_E_P)

    # Extract position/orientation
    pos_robot = T_R_E[0:3, 3]
    R_robot   = T_R_E[0:3, 0:3]
    return pos_robot, R_robot


def compute_pose_vertical(piece_x, piece_y, piece_z):
    """
    1) Build T^G_P by giving it a fixed upward z-axis = [0, 0, 1].
    2) Compute T^R_E for the robot.
    3) Return (pos_robot, R_robot).
    """
    # 1) Build T^G_P
    pos_piece = np.array([piece_x, piece_y, piece_z])
    orient_vec = np.array([0.0, 0.0, 1.0])  # vertical up
    T_G_P = build_global_pose_from_vector(pos_piece, orient_vec)

    # 2) Robot pose in global
    T_G_R = get_robot_pose_in_global(
        config.ROBOT_GLOBAL_X,
        config.ROBOT_GLOBAL_Y,
        config.ROBOT_GLOBAL_Z,
        config.ROBOT_YAW_DEG
    )

    # 3) T^E_P: piece offset from end-effector
    T_E_P = np.eye(4)
    T_E_P[2, 3] = config.PIECE_HEIGHT

    # 4) T^R_E = (T^G_R)^-1 * T^G_P * (T^E_P)^-1
    T_R_E = inv_se3(T_G_R) @ T_G_P @ inv_se3(T_E_P)

    # Extract position/orientation
    pos_robot = T_R_E[0:3, 3]
    R_robot   = T_R_E[0:3, 0:3]
    return pos_robot, R_robot
