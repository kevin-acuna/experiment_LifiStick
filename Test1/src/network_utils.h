#ifndef NETWORK_UTILS_H
#define NETWORK_UTILS_H

#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")

#include <string>

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

void receiverPointingToTransmitter(SOCKET sock);

#endif // NETWORK_UTILS_H
