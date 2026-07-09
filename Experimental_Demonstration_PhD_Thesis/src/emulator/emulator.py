"""
mock_cpp_client.py

A simple Python script to emulate the behavior of a C++ client.
It can:
1) Connect to the server started by your main.py on localhost:12345
2) Send 3 double-precision floats (x, y, z) in 24 bytes
3) Send text commands like "vertical" or "pointed"
4) Display any replies from the server

Usage Example:
    1) Run main.py (which starts the server).
    2) Then run:
        python mock_cpp_client.py
    3) Type commands like:
        sendxyz 0.5 0.6 1.2   # to send x=0.5, y=0.6, z=1.2
        text vertical         # to send the text "vertical"
        text pointed          # ...
        quit                  # to exit
"""

import socket
import struct

HOST = 'localhost'
PORT = 12345

def main():
    # 1) Create a client socket and connect
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((HOST, PORT))
    print(f"Connected to server at {HOST}:{PORT}")
    print("Type 'help' for usage instructions.\n")

    try:
        while True:
            user_input = input("> ").strip()
            if not user_input:
                continue

            # Let's define a simple command format:
            #   sendxyz x y z   -> sends 3 doubles
            #   text some_string -> sends ASCII text
            #   quit -> closes

            tokens = user_input.split()
            cmd = tokens[0].lower()

            if cmd == "help":
                print("Commands:")
                print("  sendxyz x y z   -> send three floats in 24 bytes to the server")
                print("  text message    -> send a text message to the server")
                print("  quit            -> close the connection and exit")
                print("  read            -> send manually a text message to the server")
                continue

            if cmd == "quit":
                print("Closing connection.")
                break

            if cmd == "read":
                try:
                    client_socket.settimeout(0.5)  # short timeout to not block forever
                    resp = client_socket.recv(1024)
                    if resp:
                        print("Server response:", resp.decode().strip())
                except socket.timeout:
                    pass
                finally:
                    client_socket.settimeout(None)  # reset timeout
                continue

            if cmd == "sendxyz":
                # Expecting: sendxyz x y z
                if len(tokens) != 4:
                    print("Usage: sendxyz <x> <y> <z>")
                    continue
                try:
                    x = float(tokens[1])
                    y = float(tokens[2])
                    z = float(tokens[3])
                except ValueError:
                    print("Error: x, y, z must be numeric.")
                    continue

                # Pack into 24 bytes: 3 doubles
                data = struct.pack('ddd', x, y, z)
                client_socket.sendall(data)
                print(f"Sent (x={x}, y={y}, z={z}) as 3 doubles [24 bytes].")

                # Let's try receiving any immediate response
                # We might or might not get a response (depending on server flow).
                try:
                    client_socket.settimeout(0.5)  # short timeout to not block forever
                    resp = client_socket.recv(1024)
                    if resp:
                        print("Server response:", resp.decode().strip())
                except socket.timeout:
                    pass
                finally:
                    client_socket.settimeout(None)  # reset timeout
                continue

            if cmd == "text":
                # Send the rest of the line as text
                msg = " ".join(tokens[1:])  # everything after 'text'
                if not msg:
                    print("Usage: text <message>")
                    continue
                client_socket.sendall(msg.encode())
                print(f"Sent text message: '{msg}'")

                # Attempt to read immediate server response
                try:
                    client_socket.settimeout(0.5)
                    resp = client_socket.recv(1024)
                    if resp:
                        print("Server response:", resp.decode().strip())
                except socket.timeout:
                    pass
                finally:
                    client_socket.settimeout(None)
                continue

            # If command is unrecognized
            print("Unknown command. Type 'help' for a list of valid commands.")

    except KeyboardInterrupt:
        print("Interrupted by user.")

    finally:
        client_socket.close()
        print("Socket closed. Exiting.")

if __name__ == "__main__":
    main()
