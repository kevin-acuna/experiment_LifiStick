"""
network.py

Functions to handle the socket connection with the robot 
and to send commands in the robot's expected text format.
"""

import socket

def create_socket(host, port):
    """
    Creates and returns a socket connected to the robot.
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((host, port))
    return s

def send_move_command_time(sock, pose, acceleration, velocity, time_pos):
    """
    Sends a 'movel' command to the robot using a pose [x, y, z, rx, ry, rz]
    and motion parameters: acceleration, velocity, and a specified time.
    The message format follows the robot's text-based command interface.
    """
    # Example: "movel(p[x,y,z,rx,ry,rz], a=..., v=..., t=...)\n"
    message = f"movel(p{pose}, a={acceleration}, v={velocity}, t={time_pos})\n"
    print("Sending command:", message.strip())
    sock.send(message.encode())
    print("Command sent.")
