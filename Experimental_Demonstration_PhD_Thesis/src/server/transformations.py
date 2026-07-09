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

def rotation_matrix_to_axis_angle_previus(R):
    """
    Versión anterior de la función rotation_matrix_to_axis_angle.
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



# fixed
def rotation_matrix_to_axis_angle(R):
    """
    Se considera los puntos cercanos a 180 que causaban problemas en la version anterior 
    Convierte una matriz de rotación 3x3 en una rotación en formato
    eje-ángulo (rx, ry, rz) = theta * (axis unitario).
    """
    # Verificar determinante
    det_R = np.linalg.det(R)
    if abs(det_R - 1.0) > 1e-3:
        print("[WARNING] rotation_matrix_to_axis_angle: determinant not close to 1.")

    trace_val = np.trace(R)
    # Clampeo para evitar errores numéricos
    cos_theta = max(min((trace_val - 1.0) / 2.0, 1.0), -1.0)
    theta = np.arccos(cos_theta)

    # Caso theta ~ 0 => no hay rotación
    if np.isclose(theta, 0.0, atol=1e-8):
        return (0.0, 0.0, 0.0)

    # Caso theta ~ pi => usar fórmula especial
    if np.isclose(theta, np.pi, atol=1e-8):
        # Ejes candidatos a partir de la diagonal
        rx = R[0,0] + 1.0
        ry = R[1,1] + 1.0
        rz = R[2,2] + 1.0

        # Podría ocurrir que el mayor sea muy pequeño (matriz degenerada numéricamente)
        # así que lo forzamos a que no sea negativo por razones numéricas
        # y evitamos sqrt de un número negativo
        rx = max(rx, 0.0)
        ry = max(ry, 0.0)
        rz = max(rz, 0.0)

        rx = np.sqrt(rx / 2.0)
        ry = np.sqrt(ry / 2.0)
        rz = np.sqrt(rz / 2.0)

        # Ajuste de signos basado en elementos fuera de la diagonal
        # (Mirar la relación R[2,1]-R[1,2], etc.)
        if (R[2,1] - R[1,2]) < 0.0:
            rx = -rx
        if (R[0,2] - R[2,0]) < 0.0:
            ry = -ry
        if (R[1,0] - R[0,1]) < 0.0:
            rz = -rz

        # Multiplicamos por pi (theta)
        rx *= np.pi
        ry *= np.pi
        rz *= np.pi

        return (rx, ry, rz)

    # Caso general: sin(theta) != 0
    sin_theta = np.sin(theta)
    rx = (R[2,1] - R[1,2]) / (2.0 * sin_theta)
    ry = (R[0,2] - R[2,0]) / (2.0 * sin_theta)
    rz = (R[1,0] - R[0,1]) / (2.0 * sin_theta)

    # Multiplicamos por theta
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


def compute_pose_random_orientation(piece_x, piece_y, piece_z):
    """
    1) Build T^G_P with a random orientation within specified limits:
       - Inclination (theta): 0 to THETA_MAX degrees from vertical
       - Azimuth: 0 to 359 degrees
    2) Compute T^R_E for the robot.
    3) Return (pos_robot, R_robot, theta, azimuth) where theta and azimuth are the generated values.
    """
    # Generate random integers for theta and azimuth
    theta_deg = np.random.randint(0, config.THETA_MAX + 1)  # 0 to THETA_MAX inclusive
    azimuth_deg = np.random.randint(0, 360)  # 0 to 359 inclusive
    
    # Convert to radians for computation
    theta_rad = np.deg2rad(theta_deg)
    azimuth_rad = np.deg2rad(azimuth_deg)
    
    # Convert spherical coordinates to Cartesian orientation vector
    # theta = 0 means vertical (0,0,1), theta = THETA_MAX means maximum inclination
    n_x = np.sin(theta_rad) * np.cos(azimuth_rad)
    n_y = np.sin(theta_rad) * np.sin(azimuth_rad)
    n_z = np.cos(theta_rad)
    
    # 1) Build T^G_P
    pos_piece = np.array([piece_x, piece_y, piece_z])
    orient_vec = np.array([n_x, n_y, n_z])
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
    
    return pos_robot, R_robot, theta_deg, azimuth_deg


def axis_angle_to_quaternion(rx, ry, rz):
    """
    Convierte una rotacion en eje-angulo (rx, ry, rz) = theta * axis
    al cuaternion (qx, qy, qz, qw). Es la orientacion que realmente se
    comanda al UR5, por lo que el cuaternion reportado coincide con la pose.
    """
    theta = np.sqrt(rx * rx + ry * ry + rz * rz)
    if theta < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    ax, ay, az = rx / theta, ry / theta, rz / theta
    s = np.sin(theta / 2.0)
    return (ax * s, ay * s, az * s, np.cos(theta / 2.0))


def orient_vec_to_incl_az(orient_vec):
    """
    Devuelve (inclinacion, azimut) en grados del vector de orientacion del PD (n_r)
    en el marco global {G}:
      - inclinacion: angulo desde +Z (0 = vertical apuntando arriba).
      - azimut: atan2(n_y, n_x), desde +X hacia +Y, en [0, 360).
    """
    v = np.array(orient_vec, dtype=float).reshape(3)
    n = np.linalg.norm(v)
    if n < 1e-9:
        return (0.0, 0.0)
    v = v / n
    incl = np.degrees(np.arccos(max(min(v[2], 1.0), -1.0)))
    az = np.degrees(np.arctan2(v[1], v[0]))
    if az < 0:
        az += 360.0
    return (incl, az)


def compute_pose_tilt(piece_x, piece_y, piece_z, theta_deg, az_deg):
    """
    Version DETERMINISTA de compute_pose_random_orientation: el orquestador C++
    genera el tilt (theta, az) y el servidor solo lo ejecuta.
      - theta_deg: inclinacion desde vertical.
      - az_deg: azimut desde +X hacia +Y.
    Devuelve (pos_robot, R_robot, orient_vec) donde orient_vec es la direccion
    global del PD (n_r).
    """
    theta_rad = np.deg2rad(theta_deg)
    az_rad = np.deg2rad(az_deg)

    n_x = np.sin(theta_rad) * np.cos(az_rad)
    n_y = np.sin(theta_rad) * np.sin(az_rad)
    n_z = np.cos(theta_rad)

    pos_piece = np.array([piece_x, piece_y, piece_z])
    orient_vec = np.array([n_x, n_y, n_z])
    T_G_P = build_global_pose_from_vector(pos_piece, orient_vec)

    T_G_R = get_robot_pose_in_global(
        config.ROBOT_GLOBAL_X,
        config.ROBOT_GLOBAL_Y,
        config.ROBOT_GLOBAL_Z,
        config.ROBOT_YAW_DEG
    )

    T_E_P = np.eye(4)
    T_E_P[2, 3] = config.PIECE_HEIGHT

    T_R_E = inv_se3(T_G_R) @ T_G_P @ inv_se3(T_E_P)

    pos_robot = T_R_E[0:3, 3]
    R_robot   = T_R_E[0:3, 0:3]
    return pos_robot, R_robot, orient_vec


