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
    compute_pose_random_orientation,
    compute_pose_tilt,
    rotation_matrix_to_axis_angle,
    axis_angle_to_quaternion,
    orient_vec_to_incl_az
)


def format_reached(pos_robot, rx, ry, rz, orient_vec):
    """
    Construye la respuesta enviada al cliente C++:
        "reached px py pz qx qy qz qw nr_incl nr_az"
      - (px,py,pz)      posicion del end-effector del UR5 (marco robot).
      - (qx,qy,qz,qw)   orientacion del end-effector del UR5 (cuaternion),
                        derivada del eje-angulo que realmente se comanda.
      - nr_incl, nr_az  orientacion del PD (n_r) en el marco global [grados].
    """
    qx, qy, qz, qw = axis_angle_to_quaternion(rx, ry, rz)
    incl, az = orient_vec_to_incl_az(orient_vec)
    px, py, pz = pos_robot
    return (f"reached {px} {py} {pz} "
            f"{qx} {qy} {qz} {qw} {incl} {az}")

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
        # Coordenadas de la base del robot, es decir, su (0,0,0).
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
            print(pos_robot_vert)
            # 5) Check workspace
            x_r, y_r, z_r = pos_robot_vert
            if check_workspace((x_r, y_r, z_r), piece_z):
                send_text_message(client_socket, "reachable")

                while True:
                    # Wait for next command (buffer amplio para "tilt <theta> <az>")
                    command_str = receive_text_message(client_socket, 64)
                    if not command_str:
                        print("No command received. Client might have closed.")
                        break

                    # El comando puede llevar argumentos (p.ej. "tilt 20 340").
                    tokens = command_str.split()
                    cmd = tokens[0] if tokens else ""

                    if cmd == "vertical":
                        rx, ry, rz = rotation_matrix_to_axis_angle(R_robot_vert)
                        rz = -0.7854  # ajuste de muneca (pi/4) para el caso vertical
                        pose = [x_r, y_r, z_r, rx, ry, rz]
                        send_move_command_time(robot_socket, pose, config.ACCELERATION, config.VELOCITY, config.MOVE_TIME)

                        time.sleep(config.MOVE_TIME + 2)
                        # n_r vertical = [0, 0, 1] (PD apuntando al cenit)
                        send_text_message(client_socket,
                                          format_reached(pos_robot_vert, rx, ry, rz, [0.0, 0.0, 1.0]))

                    elif cmd == "pointed":
                        pos_robot_ptr, R_robot_ptr = compute_pose_pointed_to_transmitter(piece_x, piece_y, piece_z)
                        x_p, y_p, z_p = pos_robot_ptr
                        rx2, ry2, rz2 = rotation_matrix_to_axis_angle(R_robot_ptr)
                        pose2 = [x_p, y_p, z_p, rx2, ry2, rz2]
                        send_move_command_time(robot_socket, pose2, config.ACCELERATION, config.VELOCITY, config.MOVE_TIME)

                        time.sleep(config.MOVE_TIME + 2)
                        # n_r apuntando al transmisor (LED)
                        orient_ptr = [config.TRANSMITTER_X - piece_x,
                                      config.TRANSMITTER_Y - piece_y,
                                      config.TRANSMITTER_Z - piece_z]
                        send_text_message(client_socket,
                                          format_reached(pos_robot_ptr, rx2, ry2, rz2, orient_ptr))

                    elif cmd == "tilt":
                        # tilt determinista: "tilt <theta> <az>" (grados, marco global)
                        try:
                            theta_deg = float(tokens[1])
                            az_deg = float(tokens[2])
                        except (IndexError, ValueError):
                            print(f"Invalid tilt command: {command_str}")
                            send_text_message(client_socket, "error_tilt_args")
                            continue

                        pos_robot_t, R_robot_t, orient_vec_t = compute_pose_tilt(
                            piece_x, piece_y, piece_z, theta_deg, az_deg)
                        x_t, y_t, z_t = pos_robot_t
                        rx_t, ry_t, rz_t = rotation_matrix_to_axis_angle(R_robot_t)
                        pose_t = [x_t, y_t, z_t, rx_t, ry_t, rz_t]
                        print(f"Tilt orientation: theta={theta_deg} deg, az={az_deg} deg")
                        send_move_command_time(robot_socket, pose_t, config.ACCELERATION, config.VELOCITY, config.MOVE_TIME)

                        time.sleep(config.MOVE_TIME + 2)
                        send_text_message(client_socket,
                                          format_reached(pos_robot_t, rx_t, ry_t, rz_t, orient_vec_t))

                    elif cmd == "random_n_r":
                        pos_robot_rnd, R_robot_rnd, theta, azimuth = compute_pose_random_orientation(piece_x, piece_y, piece_z)
                        x_rnd, y_rnd, z_rnd = pos_robot_rnd
                        rx_rnd, ry_rnd, rz_rnd = rotation_matrix_to_axis_angle(R_robot_rnd)
                        pose_rnd = [x_rnd, y_rnd, z_rnd, rx_rnd, ry_rnd, rz_rnd]
                        print(f"Random orientation: theta={theta} deg, azimuth={azimuth} deg")
                        send_move_command_time(robot_socket, pose_rnd, config.ACCELERATION, config.VELOCITY, config.MOVE_TIME)

                        time.sleep(config.MOVE_TIME + 2)
                        theta_rad = np.deg2rad(theta)
                        az_rad = np.deg2rad(azimuth)
                        orient_rnd = [np.sin(theta_rad) * np.cos(az_rad),
                                      np.sin(theta_rad) * np.sin(az_rad),
                                      np.cos(theta_rad)]
                        send_text_message(client_socket,
                                          format_reached(pos_robot_rnd, rx_rnd, ry_rnd, rz_rnd, orient_rnd))

                    elif cmd == "finished":
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
