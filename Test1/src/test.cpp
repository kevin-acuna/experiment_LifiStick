#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")
#include <iostream>
#include <limits>      // For numeric_limits
#include <cctype>      // For toupper
#define NOMINMAX
#include <Windows.h>
#include "stdafx.h"    // Only if you're using precompiled headers
#include "instrument.h"
#include <vector>
#include <fstream>
#include "positions.h"     // Cabecera para las funciones de posiciones
#include "network_utils.h" // Cabecera para las funciones de red (renombradas)
using namespace std;


const char* PATH_POSITIONS_FILE = "src/positionsToSample/test.txt";

int main()
{
    system("chcp 65001 > nul"); // Optional: enable UTF-8 output in console
    initializeWinsock(); // Inicializar Winsock
    SOCKET sock = connectToServer("127.0.0.1", 12345); // Conectar al servidor
    vector<Position> positions; // Vector de posiciones
    loadPositions(PATH_POSITIONS_FILE, positions); // Cargar posiciones

    for (auto& pos : positions) {
        if (!pos.done) {

            cout << "Position: " << pos.x << ", " << pos.y << endl;

            sendCoordinates(sock, pos.x, pos.y, 1.05); // Envía coordenadas
            std::string response = receiveResponse(sock, 1); // Recibe respuesta

            if (response == "obtainable") {
                cout << "Position is obtainable" << endl;

                sendMessage(sock, "go"); // Confirmar que vaya
                std::string confirmation = receiveResponse(sock, 30);

                if (confirmation == "reached") {
                    cout << "Position reached" << endl;
                } else {
                    cout << "Position not reached" << endl;
                }
            } else {
                cout << "Position is not obtainable" << endl;
            }
        }
    }

    // Cerrar socket, si es necesario
    closesocket(sock);
    WSACleanup();

    return 0;
}
