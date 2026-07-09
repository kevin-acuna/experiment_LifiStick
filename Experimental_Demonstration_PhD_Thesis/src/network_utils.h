#ifndef NETWORK_UTILS_H
#define NETWORK_UTILS_H

#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")

#include <string>

// Pose devuelta por el servidor tras alcanzar una orientacion del PD.
// Formato de respuesta del servidor:
//   "reached px py pz qx qy qz qw nr_incl nr_az"
//  - (px,py,pz)      posicion del end-effector del UR5
//  - (qx,qy,qz,qw)   orientacion del end-effector del UR5 (cuaternion)
//  - nr_incl, nr_az  orientacion del PD (n_r) en el marco global [grados]:
//                    inclinacion desde +Z y azimut desde +X hacia +Y.
struct RobotPose {
    double px = 0.0, py = 0.0, pz = 0.0;
    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
    double nr_incl = 0.0, nr_az = 0.0;
    bool   valid = false;
};

// Inicializa la librería Winsock
void initializeWinsock();

// Crea un socket y se conecta al servidor
SOCKET connectToServer(const char* ip, int port);

// Envía coordenadas (x, y, z) al servidor
void sendCoordinates(SOCKET sock, double x, double y, double z);

// Recibe respuesta del servidor con un timeout en segundos
std::string receiveResponse(SOCKET sock, int timeout_sec);

// Envía un mensaje (string) al servidor
void sendMessage(SOCKET sock, const std::string& message);

void receiverPointingToCeil(SOCKET sock);
void receiverRandomOrientation(SOCKET sock);

void receiverPointingToTransmitter(SOCKET sock);

void receiverFinished(SOCKET sock);

// Comanda un tilt determinista del PD: inclinacion theta y azimut az (grados),
// en el marco global. El servidor mueve el UR5 a esa orientacion y responde con
// la pose alcanzada. El azimut se mide desde +X hacia +Y.
void receiverTilt(SOCKET sock, double theta_deg, double az_deg);

// Espera la respuesta "reached px py pz qx qy qz qw nr_incl nr_az" y la parsea.
// Devuelve RobotPose con valid=false si no llega o el formato no es valido.
RobotPose receivePose(SOCKET sock, int timeout_sec);

#endif // NETWORK_UTILS_H
