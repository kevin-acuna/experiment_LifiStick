#define NOMINMAX
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")


#include <Windows.h>

#include <iostream>
#include <limits>      // For numeric_limits
#include <istream>     // For cin.ignore
#include <cctype>      // For toupper

#include "stdafx.h"    // Only if you're using precompiled headers
#include "instrument.h"
#include <vector>
#include <fstream>
#include "positions.h"     // Cabecera para las funciones de posiciones
#include "network_utils.h" // Cabecera para las funciones de red (renombradas)
using namespace std;


// *****************************************************************************
// Adjust according to your experiment
// *****************************************************************************
const char* PATH_POSITIONS_FILE = "src/positionsToSample/test.txt";

const double TRANSMITTER_H = 1.98; // altitude in meters
const double RECEIVER_H = 0.96;    // altitude in meters

int MOTOR_AXIS_X = 27006796;  // External axis
int MOTOR_AXIS_Y = 27007072;  // Internal axis
// *****************************************************************************

int main()
{
    system("chcp 65001 > nul"); // Optional: enable UTF-8 output in console
    initializeWinsock(); // Inicializar Winsock
    SOCKET sock = connectToServer("127.0.0.1", 12345); // Conectar al servidor
    vector<Position> positions; // Vector de posiciones
    loadPositions(PATH_POSITIONS_FILE, positions); // Cargar posiciones

    instrument gimbal; // Gimbal Mechanism
    gimbal.setSerialNo_MotorX(MOTOR_AXIS_X);
    gimbal.setSerialNo_MotorY(MOTOR_AXIS_Y);
    gimbal.setTransmitterPosition(0, 0, TRANSMITTER_H);

    for (auto& pos : positions) {
        if (!pos.done) {

            //cout << "Position: " << pos.x << ", " << pos.y << endl;
            sendCoordinates(sock, pos.x, pos.y, RECEIVER_H); // Envía coordenadas
            std::string response = receiveResponse(sock, 1); // Recibe respuesta

            if (response == "obtainable") {

                // ****************************************************************
                // Receiver Position Input
                // ****************************************************************
                cout << "************************************\n";
                cout << "SAMPLING POINT: X, Y, Z\n";
                cout << "Receiver position set to: (" << pos.x << ", " << pos.y << ", " << RECEIVER_H << ")\n\n";
                cout << "************************************\n\n";

                // Scenario 1 - Pointing transmitter to floor and receiver to ceiling
                cout << "Scenario 1: Waiting for alignment...\n";
                gimbal.transmitterPointingToFloor();
                // receiverPointingToCeilToPosition(receiverX, receiverY);
                // Crear funcion de esto ************
                sendMessage(sock, "go"); // Confirmar que vaya
                std::string confirmation = receiveResponse(sock, 30);
                if (confirmation == "reached") {
                    cout << "Position reached" << endl;
                } else {
                    cout << "Position not reached" << endl;
                }
                // **********************************
                cout << "Scenario 1: Ready!\n";

                // Wait for a valid option (C to continue or Q to quit)
                char option;
                while (true) {
                    cout << "Press C to continue or Q to quit: ";
                    cin >> option;
                    option = toupper(option);
                    if (option == 'C' || option == 'Q')
                        break;
                    cout << "Invalid option. Please try again." << endl;
                }
                if (option == 'Q') {
                    cout << "Program terminated by user." << endl;
                    break;
                }
                cin.ignore(numeric_limits<streamsize>::max(), '\n'); // Clean the input buffer

                // Scenario 2 - Pointing transmitter to receiver
                cout << "\nScenario 2: Waiting for alignment...\n";
                gimbal.transmitterPointingToReceiver(pos.x, pos.y, RECEIVER_H);
                cout << "Scenario 2: Ready!\n";
                while (true) {
                    cout << "Press C to continue or Q to quit: ";
                    cin >> option;
                    option = toupper(option);
                    if (option == 'C' || option == 'Q')
                        break;
                    cout << "Invalid option. Please try again." << endl;
                }
                if (option == 'Q') {
                    cout << "Program terminated by user." << endl;
                    break;
                }
                cin.ignore(numeric_limits<streamsize>::max(), '\n');

                // Scenario 3 - Pointing transmitter to floor and receiver to transmitter
                cout << "\nScenario 3: Waiting for alignment...\n";
                gimbal.transmitterPointingToFloor();
                // receiverPointingToTransmitter(receiverX, receiverY);
                cout << "Scenario 3: Ready!\n";
                while (true) {
                    cout << "Press C to continue or Q to quit: ";
                    cin >> option;
                    option = toupper(option);
                    if (option == 'C' || option == 'Q')
                        break;
                    cout << "Invalid option. Please try again." << endl;
                }
                if (option == 'Q') {
                    cout << "Program terminated by user." << endl;
                    break;
                }
                cin.ignore(numeric_limits<streamsize>::max(), '\n');

                cout << endl;



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
