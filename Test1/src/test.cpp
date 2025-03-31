#define NOMINMAX
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")


#include <Windows.h>
#include <iostream>
#include <limits>      // For numeric_limits
#include <istream>     // For cin.ignore
#include <cctype>      // For toupper
#include <string>

#include "stdafx.h"    // If using precompiled headers
#include "instrument.h"
#include <vector>
#include <fstream>

#include "positions.h"     // For position structs/functions (if needed)
#include "network_utils.h" // For connectToServer, sendCoordinates, sendMessage, receiveResponse, etc.

using namespace std;


// *****************************************************************************
// Adjust according to your experiment
// *****************************************************************************
const char* PATH_POSITIONS_FILE = "src/positionsToSample/test.txt";

const double TRANSMITTER_H = 2.00; // altitude in meters
const double RECEIVER_H = 0.96;    // altitude in meters

int MOTOR_AXIS_X = 27006796;  // External axis
int MOTOR_AXIS_Y = 27007072;  // Internal axis
// *****************************************************************************

// Predefined positions for the robot base (X, Y). Indices 1..16 map to rows 0..15.
static const double PREDEFINED_POSITIONS[16][2] = {
    {-0.6, -0.6},
    {-0.6,  0.0},
    {-0.6,  0.6},
    {-0.6,  1.2},
    { 0.0,  1.2},
    { 0.0,  0.6},
    { 0.0,  0.0},
    { 0.0, -0.6},
    { 0.6, -0.6},
    { 0.6,  0.0},
    { 0.6,  0.6},
    { 0.6,  1.2},
    { 1.2,  1.2},
    { 1.2,  0.6},
    { 1.2,  0.0},
    { 1.2, -0.6}
};

int promptUserForRobotPositionIndex()
{
    while (true) {
        cout << "\n"
                  << "====================================================\n"
                  << " Select Robot Position [1..16] or 'q' to quit\n"
                  << "----------------------------------------------------\n"
                  << " 1)  (-0.6, -0.6)\n"
                  << " 2)  (-0.6,  0.0)\n"
                  << " 3)  (-0.6,  0.6)\n"
                  << " 4)  (-0.6,  1.2)\n"
                  << " 5)  ( 0.0,  1.2)\n"
                  << " 6)  ( 0.0,  0.6)\n"
                  << " 7)  ( 0.0,  0.0)\n"
                  << " 8)  ( 0.0, -0.6)\n"
                  << " 9)  ( 0.6, -0.6)\n"
                  << "10)  ( 0.6,  0.0)\n"
                  << "11)  ( 0.6,  0.6)\n"
                  << "12)  ( 0.6,  1.2)\n"
                  << "13)  ( 1.2,  1.2)\n"
                  << "14)  ( 1.2,  0.6)\n"
                  << "15)  ( 1.2,  0.0)\n"
                  << "16)  ( 1.2, -0.6)\n"
                  << "----------------------------------------------------\n"
                  << "Choose an option: ";
        std::string input;
        cin >> input;

        if (input == "q" || input == "Q") {
            // Return a sentinel value (e.g., -1) indicating we should quit
            return -1;
        }
        try {
            int index = std::stoi(input);
            if (index >= 1 && index <= 16) {
                return index;
            }
        } catch (...) {
            // Conversion failed
        }
        cout << "[Error] Invalid selection. Please try again.\n";
        // Clear any extra input
        cin.clear();
        cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
}

int main()
{
    system("chcp 65001 > nul"); // Optional: enable UTF-8 output in console
    initializeWinsock(); // Inicializar Winsock
    SOCKET sock = connectToServer("127.0.0.1", 12345); // Conectar al servidor
    vector<Position> positions; // Vector de posiciones
    loadPositions(PATH_POSITIONS_FILE, positions); // Cargar posiciones

    int index = promptUserForRobotPositionIndex();
    if (index == -1) {
        cout << "User selected 'q' to quit. Exiting.\n";
        closesocket(sock);
        WSACleanup();
        return 0;
    }

    double robotX = PREDEFINED_POSITIONS[index - 1][0];
    double robotY = PREDEFINED_POSITIONS[index - 1][1];
    double robotZ = 0.782; 
    sendCoordinates(sock, robotX, robotY, robotZ); // Envía posicion

    // Clear the screen to proceed
    #ifdef _WIN32
        system("cls");
    #else
        system("clear");
    #endif


    instrument gimbal; // Gimbal Mechanism
    gimbal.setSerialNo_MotorX(MOTOR_AXIS_X);
    gimbal.setSerialNo_MotorY(MOTOR_AXIS_Y);
    gimbal.setTransmitterPosition(0, 0, TRANSMITTER_H);

    for (auto& pos : positions) {
        if (!pos.done) {

            //cout << "Position: " << pos.x << ", " << pos.y << endl;
            sendCoordinates(sock, pos.x, pos.y, RECEIVER_H); // Envía coordenadas
            std::string response = receiveResponse(sock, 1); // Recibe respuesta

            if (response == "reachable") {

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
                cout << "[Info] Robot starting to move\n";
                receiverPointingToCeil(sock); // Send command to receiver to point to ceiling

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
                gimbal.transmitterPointingToReceiver(-pos.x, -pos.y, RECEIVER_H); // ajuste de coordenadas
                cout << "[Info] Robot starting to move\n";
                sendCoordinates(sock, pos.x, pos.y, RECEIVER_H); // Envía coordenadas
                std::string response = receiveResponse(sock, 1); // Recibe respuesta
                receiverPointingToTransmitter(sock); 

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
