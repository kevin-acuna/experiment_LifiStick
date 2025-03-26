import socket
from time import sleep

# Configuración del robot
HOST = "193.51.28.59"  # Dirección IP del robot
PORT = 30002           # Puerto de conexión

# Posición objetivo (x, y) y valor constante de z
TARGET_X = 1.0
TARGET_Y = 1.0
TARGET_Z = 0.87

# Parámetros de movimiento
ACCELERATION = 1.0
VELOCITY = 0.1
TIME_POS = 4  # Tiempo estimado de movimiento

def create_socket(host, port):
    """Crea y retorna un socket conectado al robot."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((host, port))
    return s

def send_move_command_time(sock, pose, acceleration, velocity, time_pos):
    """
    Envía el comando de movimiento al robot con la pose, aceleración, velocidad y tiempo especificados.
    El formato del mensaje se adapta al sistema de comandos del robot.
    """
    message = "movel(p{}, a={}, v={}, t={})\n".format(pose, acceleration, velocity, time_pos)
    print("Enviando comando:", message.strip())
    sock.send(message.encode())
    print("Comando enviado.")

def main():
    # Conexión al robot
    sock = create_socket(HOST, PORT)
    print("Conectado al robot.")

    # Definición de la pose: [x, y, z, rx, ry, rz]
    pose = [TARGET_X, TARGET_Y, TARGET_Z, 0, 0, 0]
    send_move_command_time(sock, pose, ACCELERATION, VELOCITY, TIME_POS)

    # Esperar el tiempo estimado para el movimiento
    sleep(TIME_POS + 2)
    
    sock.close()
    print("Movimiento completado. Conexión cerrada.")

if __name__ == "__main__":
    main()
