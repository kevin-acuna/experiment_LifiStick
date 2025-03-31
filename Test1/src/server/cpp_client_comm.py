# cpp_client_comm.py

import socket
import struct

def start_server(host='localhost', port=12345):
    """
    Starts a TCP server socket for incoming connections from a C++ client.
    By default, it listens on localhost:12345.
    """
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)
    print(f"Listening on {host}:{port}")
    return server_socket

def process_coordinates(client_socket):
    """
    Receives a fixed-size binary packet (24 bytes) from the client
    and unpacks three double-precision floats (x, y, z).
    Returns (x, y, z) as a tuple or False if no data is received.
    """
    data = client_socket.recv(24)
    print(data)
    if not data:
        return False
    x, y, z = struct.unpack('ddd', data)
    return (x, y, z)

def send_text_message(client_socket, message: str):
    """
    Sends a text message to the client via the socket.
    message: ASCII string to send (e.g. 'reachable', 'unobtainable', 'reached').
    """
    client_socket.sendall(message.encode())
    print("sended:", message.encode())

def receive_text_message(client_socket, buffer_size=24):
    """
    Receives a text message from the client socket, up to buffer_size bytes.
    Returns the string (decoded).
    """
    data = client_socket.recv(buffer_size)
    if not data:
        return None
    print(data)
    return data.decode().strip()
