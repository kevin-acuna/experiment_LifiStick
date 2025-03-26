import socket
from time import sleep
from numpy import pi
import numpy as np

def vector_to_axis_angle(v):
    """
    Dado un vector (no necesariamente unitario), calcula la rotación en notación eje-ángulo 
    (rx, ry, rz) necesaria para alinear el eje Z (por defecto, el TCP apunta hacia arriba) 
    con el vector 'v'.
    
    Args:
        v (array-like): Vector objetivo (puede no estar normalizado).
        
    Returns:
        tuple: (rx, ry, rz) en radianes.
        
    Raises:
        ValueError: Si el vector ingresado es el vector cero.
    """
    v = np.array(v, dtype=float)
    norm = np.linalg.norm(v)
    if norm < 1e-6:
        raise ValueError("El vector ingresado es cero.")
    
    # Normalizar el vector objetivo
    v_unit = v / norm
    # Eje Z de referencia (TCP en configuración cero apunta hacia arriba)
    z_axis = np.array([0, 0, 1], dtype=float)
    
    # Calcular el ángulo entre z_axis y v_unit usando el producto punto
    dot = np.dot(z_axis, v_unit)
    dot = np.clip(dot, -1.0, 1.0)  # evitar errores numéricos
    angle = np.arccos(dot)
    
    # Calcular el eje de rotación como el producto vectorial entre z_axis y v_unit
    axis = np.cross(z_axis, v_unit)
    axis_norm = np.linalg.norm(axis)
    
    # Si el vector resultante es casi cero, significa que v_unit es paralelo a z_axis
    if axis_norm < 1e-6:
        # Si el vector es el mismo (ángulo 0), no se necesita rotación.
        # Si es opuesto (ángulo π), elegimos un eje arbitrario perpendicular, por ejemplo [1,0,0].
        if dot > 0:
            return 0.0, 0.0, 0.0
        else:
            return np.pi, 0.0, 0.0
    
    # Normalizar el eje de rotación
    axis_unit = axis / axis_norm
    
    # La representación eje-ángulo es: (rx, ry, rz) = eje_normalizado * ángulo
    rx, ry, rz = axis_unit * angle
    return rx, ry, rz





# Configuración del robot
HOST = "193.51.28.59"  # Dirección IP del robot
PORT = 30002           # Puerto de conexión

# Posición objetivo (x, y) y valor constante de z
TARGET_X = 0.5
TARGET_Y = 0.5
TARGET_Z = 0.13

# Parámetros de movimiento
ACCELERATION = 1.0
VELOCITY = 0.1
TIME_POS = 10  # Tiempo estimado de movimiento

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
    print("Starting ...")
    sock = create_socket(HOST, PORT)
    print("Conectado al robot.")

    # Definición de la pose: [x, y, z, rx, ry, rz]
    pose = [TARGET_X, TARGET_Y, TARGET_Z, -0.785, 0, 0]
    send_move_command_time(sock, pose, ACCELERATION, VELOCITY, TIME_POS)

    # Esperar el tiempo estimado para el movimiento
    sleep(TIME_POS + 2)
    
    sock.close()
    print("Movimiento completado. Conexión cerrada.")

    # # Ejemplos de uso:

    # # 1. Si el vector ya apunta hacia arriba (0, 0, 1) no se necesita rotación.
    # print("Para v = [0, 0, 1]:", vector_to_axis_angle([0, 0, 1]))  # Salida: (0.0, 0.0, 0.0)

    # # 2. Si el vector es (0, 1, 1) (diagonal entre Y y Z)
    # print("Para v = [0, 1, 1]:", vector_to_axis_angle([0, 1, 1]))

    # # 3. Si el vector es (1, 0, 0) (apuntar horizontalmente en X)
    # print("Para v = [1, 0, 0]:", vector_to_axis_angle([1, 0, 0]))
if __name__ == "__main__":
    main()


