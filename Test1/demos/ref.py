# POSITION OF THE ROBOT
# ********************************************************************
ROBOT_COORDINATE_X = 0.9
ROBOT_COORDINATE_Y = 0.9

# ********************************************************************

# ************************************************************************************************
# CONFIG DEPENDING OF THE POSITION OF THE ROBOT
# ************************************************************************************************
TIEMPO_POS = 4

import socket
import struct
from time import sleep
import math3d as m3d
import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # Usa TkAgg, que es generalmente bien soportado y estable

import matplotlib.pyplot as plt

# to email
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
from dotenv import load_dotenv
import smtplib
import os
load_dotenv()



# ************************************************************************************************
# IMPORTANTE
# ************************************************************************************************
# 1) ROBOT TIENE QUE ESTAR CONECTADO
# 2) CONFIGURAR LA UBICACION DEL ROBOT



# ************************************************************************************************
# FIXING PARAMETERS (WORKING SPACE)
# ************************************************************************************************
WS_R_INNER = 0.27
WS_R_OUTER = 0.90
# 2do cuadrante
WS_THETA_INIT = 86
WS_THETA_FIN = 184
# 3er cuadrante
# WS_THETA_INIT = 180
# WS_THETA_FIN = 270

# ************************************************************************************************
# Constantes de configuración
HOST = "193.51.28.59"
PORT = 30002
ROBOT_COORDINATE_Z = 0.87 #Constant

ACCELERATION = 1.0
VELOCITY = 0.1
INITIAL_POSE = [-0.4, -0.4, 0.2, 0, 0, 0]

# ************************************************************************************************

# ************************************************************************************************
working_space = { "r_inner": WS_R_INNER,
                  "r_outer": WS_R_OUTER,
                  "theta_init": WS_THETA_INIT,
                  "theta_fin":WS_THETA_FIN}

origin_baseRobot = {"x": ROBOT_COORDINATE_X,
                    "y": ROBOT_COORDINATE_Y, 
                    "z": ROBOT_COORDINATE_Z} # z es fijo prueba exp: 90cm >> 107cm (real) 75cm >> 92cm


# ***********************************
# CONEXION CON EL ROBOT
# ***********************************
def create_socket(host, port):
    """ Crea y retorna un socket conectado al servidor especificado. """
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Corregido AF_INET y los paréntesis
    s.connect((host, port))
    return s

def send_move_command(sock, pose, acceleration, velocity):
    """ Envía un comando de movimiento al robot a través del socket. """
    message = "movel(p{}, a={}, v={})\n".format(pose, acceleration, velocity)
    print("Enviando comando:", message.strip())
    sock.send(message.encode())
    print("Ejecutado")

def send_move_command_time(sock, pose, acceleration, velocity, time_pos):
    """ Envía un comando de movimiento al robot a través del socket. """
    message = "movel(p{}, a={}, v={}, t={})\n".format(pose, acceleration, velocity, time_pos)
    print("Enviando comando:", message.strip())
    sock.send(message.encode())
    print("Ejecutado")

def receive_data(sock):
    """ Recibe datos del socket y los imprime. """
    data = sock.recv(1024)
    print("Received:", repr(data))
# *********************************************


def cartesian_to_cilindrical(x, y, z):
    # Calcula el radio
    r = np.sqrt(x**2 + y**2)

    # Calcula el ángulo en radianes usando arctan2
    theta = np.arctan2(y, x)

    # Convierte el ángulo a grados
    theta_degrees = np.degrees(theta)

    # Ajusta el rango de 0 a 360 grados
    if theta_degrees < 0:
        theta_degrees += 360

    return r, theta_degrees, z

def verificar_espacio_trabajo(coordenada):
  x,y,z = coordenada
  r, theta, z = cartesian_to_cilindrical(x, y, z)
  #print(r,theta,z)
  if (working_space["r_inner"] <= r <= working_space["r_outer"]):
    if (working_space["theta_init"] <= theta <= working_space["theta_fin"]):
      is_reachable = True
    else:
      is_reachable = False  
  else:
    is_reachable = False
  #print(is_reachable)
  return is_reachable

def procesar_coordenada(client_socket):
    data = client_socket.recv(24)
    if not data:
        return False
    x, y, z = struct.unpack('ddd', data)
    return (x, y, z)

def iniciar_servidor():
    host = 'localhost'
    port = 12345
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)
    print(f"Listening on {host}:{port}")
    return server_socket





def main():

    sock = create_socket(HOST, PORT) #ROBOT
    print("Conectado al robot adecuadamente ...")
    server_socket = iniciar_servidor()

    try:
        client_socket, addr = server_socket.accept()
        print(f"Connection from {addr} has been established.")

        while True:
            coordenada = procesar_coordenada(client_socket)
            coordenada_robot = transform_to_baseReference(coordenada)

            if coordenada_robot:
                if verificar_espacio_trabajo(coordenada_robot):
                    client_socket.sendall(b"reachable")
                    data = client_socket.recv(24) # Confirmacion de inicio
                    if (data == b"go"):
                        
                        pose = [coordenada_robot[0],coordenada_robot[1],coordenada_robot[2],0,0,0]
                        sleep(0.2)
                        
                        send_move_command_time(sock, pose, ACCELERATION, VELOCITY, TIEMPO_POS)

                        SLEEP_DURATION = TIEMPO_POS + 2
                        sleep(SLEEP_DURATION) 
                        
                        client_socket.sendall(b"reached")
                        sleep(0.2)
                else:
                    client_socket.sendall(b"unobtainable")
            

            
    except KeyboardInterrupt:
        print("Shutting down server.")
    finally:

        print("Closing socket")
        sock.close() #ROBOT
        client_socket.close()
        server_socket.close()
        input()

if __name__ == "__main__":
    main()
