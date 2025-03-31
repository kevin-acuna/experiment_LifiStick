"""
main.py

Now we:
 1) Connect to the robot.
 2) Start the local server for the C++ client.
 3) **Wait once** for the robot's (X, Y, Z) position from the client and override config.
 4) Then, in the main loop, process piece coordinates, orientation commands, etc.
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

        print("Waiting to receive the robot's current (X, Y, Z) position from the client...")
        robot_coords = process_coordinates(client_socket)   # Reuse same 24-byte struct logic if you want
        if not robot_coords:
            print("No data received for the robot's position. Closing.")
            return
        # Unpack
        robot_x, robot_y, robot_z = robot_coords
        print(f"Received robot position: X={robot_x}, Y={robot_y}, Z={robot_z}")

        # Overwrite config so the rest of the code sees updated values
        config.ROBOT_GLOBAL_X = robot_x
        config.ROBOT_GLOBAL_Y = robot_y

        while True:
            # 3) Wait for piece coordinates from the client
            piece_coords = process_coordinates(client_socket)
            if not piece_coords:
                print("No data received for the piece coordinates. Client may have disconnected.")
                break
            piece_x, piece_y, piece_z = piece_coords
            # 4) Compute the 'vertical' orientation
            pos_robot_vert, R_robot_vert = compute_pose_vertical(piece_x, piece_y, piece_z)

            # 5) Check workspace
            x_r, y_r, z_r = pos_robot_vert
            if check_workspace((x_r, y_r, z_r)):
                send_text_message(client_socket, "reachable")

                while True:
                    # Wait for next command
                    command_str = receive_text_message(client_socket, 24)
                    if not command_str:
                        print("No command received. Client might have closed.")
                        break

                    if command_str == "vertical":
                        rx, ry, rz = rotation_matrix_to_axis_angle(R_robot_vert)
                        pose = [x_r, y_r, z_r, rx, ry, 0.7854] #pi/4
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

                    elif command_str == "finished":
                        print("finish this Run")
                        break

                    else:
                        print(f"Unknown command: {command_str}")
                        # Possibly ignore or break from loop

            else:
                send_text_message(client_socket, "unobtainable")
                # Then continue waiting for next piece coords

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
