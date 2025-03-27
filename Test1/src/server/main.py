"""
main.py
"""

import time
import numpy as np
import socket

import config
from network import create_socket, send_move_command_time
from cpp_client_comm import (
    start_server,
    process_coordinates,
    send_text_message,
    receive_text_message
)
from workspace_utils import check_workspace
from transformations import (
    compute_pose_vertical,
    compute_pose_pointed_to_transmitter,
    rotation_matrix_to_axis_angle
)

def main():
    print("Starting main...")

    # 1) Connect to the robot
    print(f"Connecting to robot at {config.ROBOT_HOST}:{config.ROBOT_PORT}")
    robot_socket = create_socket(config.ROBOT_HOST, config.ROBOT_PORT)
    print("Connected to the robot.")

    # 2) Start local server for the C++ client
    server_socket = start_server('localhost', 12345)
    client_socket, addr = server_socket.accept()
    print(f"Connection from {addr} established.")

    try:
        while True:
            # 3) Wait for piece coordinates from the client
            piece_coords = process_coordinates(client_socket)
            if not piece_coords:
                print("No data received. Client may have disconnected.")
                break  # or continue, or handle differently

            piece_x, piece_y, piece_z = piece_coords

            # 4) Directly compute the vertical orientation 
            #    (since we removed the old step 4 with transform_to_baseReference)
            pos_robot_vert, R_robot_vert = compute_pose_vertical(piece_x, piece_y, piece_z)

            # 5) Check if that vertical pose is in workspace
            x_r, y_r, z_r = pos_robot_vert
            if check_workspace((x_r, y_r, z_r)):
                send_text_message(client_socket, "reachable")

                # Wait for next message: "vertical" or "pointed"
                command_str = receive_text_message(client_socket, 24)
                if not command_str:
                    print("No command received. Client might have closed.")
                    break

                if command_str == "vertical":
                    rx, ry, rz = rotation_matrix_to_axis_angle(R_robot_vert)
                    pose = [x_r, y_r, z_r, rx, ry, rz]
                    send_move_command_time(robot_socket, pose, config.ACCELERATION, config.VELOCITY, config.MOVE_TIME)

                    time.sleep(config.MOVE_TIME + 2)
                    send_text_message(client_socket, "reached")

                elif command_str == "pointed":
                    pos_robot_ptr, R_robot_ptr = compute_pose_pointed_to_transmitter(piece_x, piece_y, piece_z)
                    x_p, y_p, z_p = pos_robot_ptr
                    rx2, ry2, rz2 = rotation_matrix_to_axis_angle(R_robot_ptr)
                    pose2 = [x_p, y_p, z_p, rx2, ry2, rz2]
                    send_move_command_time(robot_socket, pose2, config.ACCELERATION, config.VELOCITY, config.MOVE_TIME)

                    time.sleep(config.MOVE_TIME + 2)
                    send_text_message(client_socket, "reached")

                else:
                    print(f"Unknown command: {command_str}")
                    # Possibly ignore or break from loop

            else:
                send_text_message(client_socket, "unobtainable")
                # Then continue to wait for next coords

    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        print("Shutting down. Closing sockets.")
        robot_socket.close()
        client_socket.close()
        server_socket.close()
        print("Done.")

if __name__ == "__main__":
    main()
