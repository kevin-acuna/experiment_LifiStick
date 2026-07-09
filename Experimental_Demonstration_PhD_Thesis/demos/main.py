"""
main.py

This script demonstrates how to:
1) Build a desired piece pose in the global frame based on the transmitter position.
2) Compute the required end-effector pose in the robot's local frame (T^R_E).
3) Convert the rotation to axis-angle (rx, ry, rz).
4) Open a socket to the robot and send the move command.

Adjust the code to match your real scenario.
"""

import time
import numpy as np

import config
from transformations import (
    get_robot_pose_in_global,
    get_piece_pose_from_transmitter,
    build_global_pose_from_vector,
    inv_se3,
    rotation_matrix_to_axis_angle
)
from network import (
    create_socket,
    send_move_command_time
)

def main():
    print("Starting program...")

    # ----------------------------------------------------------
    # 1) Desired piece pose in the global frame
    # ----------------------------------------------------------
    # Example: we want the piece (receptor) at (0.4, 0.5, 1.3),
    # oriented toward the transmitter at (0,0,2) [defined in config].
    piece_x, piece_y, piece_z = (-1.1, -1.1, 0.92)
    pos_piece, orient_piece = get_piece_pose_from_transmitter(
        piece_x, piece_y, piece_z,
        config.TRANSMITTER_X,
        config.TRANSMITTER_Y,
        config.TRANSMITTER_Z
    )
    T_G_P = build_global_pose_from_vector(pos_piece, orient_piece)

    # ----------------------------------------------------------
    # 2) Robot pose in the global frame: T^G_R
    # ----------------------------------------------------------
    T_G_R = get_robot_pose_in_global(
        config.ROBOT_GLOBAL_X,
        config.ROBOT_GLOBAL_Y,
        config.ROBOT_GLOBAL_Z,
        config.ROBOT_YAW_DEG
    )

    # ----------------------------------------------------------
    # 3) Piece offset in the end-effector: T^E_P (0.1 m up in local z)
    # ----------------------------------------------------------
    T_E_P = np.eye(4)
    T_E_P[2, 3] = config.PIECE_HEIGHT

    # ----------------------------------------------------------
    # 4) Compute T^R_E so that the piece ends up at T^G_P:
    #    T^G_P = T^G_R * T^R_E * T^E_P
    #    => T^R_E = (T^G_R)^-1 * T^G_P * (T^E_P)^-1
    # ----------------------------------------------------------
    T_R_E = inv_se3(T_G_R) @ T_G_P @ inv_se3(T_E_P)

    # Extract position and rotation from T^R_E
    pos_robot = T_R_E[0:3, 3]
    R_robot   = T_R_E[0:3, 0:3]

    rx, ry, rz = rotation_matrix_to_axis_angle(R_robot)

    # Prepare final pose in [x, y, z, rx, ry, rz] format
    pose_robot = [pos_robot[0], pos_robot[1], pos_robot[2], rx, ry, rz]

    print("===== Computed Robot Pose (Local) =====")
    print(f"Position: {pos_robot}")
    print(f"Axis-angle rotation (rx, ry, rz): {rx, ry, rz}")
    print("=======================================")

    # ----------------------------------------------------------
    # 5) Establish socket connection and send move command
    # ----------------------------------------------------------
    sock = create_socket(config.ROBOT_HOST, config.ROBOT_PORT)
    print("Connected to the robot.")

    # Now send the movement command
    send_move_command_time(sock,
                           pose_robot,
                           config.ACCELERATION,
                           config.VELOCITY,
                           config.MOVE_TIME)

    # Wait for the motion to complete
    time.sleep(config.MOVE_TIME + 2)

    # Close socket
    sock.close()
    print("Movement completed. Socket closed.")

if __name__ == "__main__":
    main()
