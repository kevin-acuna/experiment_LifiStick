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
// 302,x Y 44,y

// *****************************************************************************
// Adjust according to your experiment
// *****************************************************************************
const char* PATH_POSITIONS_FILE = "src/positionsToSample/positions3D.txt";

const double TRANSMITTER_H = 2.00; // altitude in meters

int MOTOR_AXIS_X = 27006796;  // External axis
int MOTOR_AXIS_Y = 27007072;  // Internal axis
// *****************************************************************************


// Orientaciones predefinidas para el transmisor {inclinacion, azimuth}
// inclinacion: angulo con respecto a la vertical (0-180 grados)
// azimuth: angulo en el plano XY desde el eje X (0-360 grados)
static const double PREDEFINED_ORIENTATIONS[][2] = {
    {0,0},
    {57.6,87.8},
    {57.7,358.6},
    {57.2,177.7},
    {55.7,268.1}
};

// Número de orientaciones predefinidas
#define K_ORIENTATIONS (sizeof(PREDEFINED_ORIENTATIONS) / sizeof(PREDEFINED_ORIENTATIONS[0]))


// Predefined positions for the robot base (X, Y).
// El tamaño del array se determina automáticamente por el número de elementos inicializados
static const double PREDEFINED_POSITIONS[][2] = {
    {-0.5, -0.5},
    {-0.5,  0.0},
    {-0.5,  0.5},
    {-0.5,  1.0},
    {-0.5,  1.5},
    { 0.0, -0.5},
    { 0.0,  0.0},
    { 0.0,  0.5},
    { 0.0,  1.0},
    { 0.0,  1.5},
    { 0.5, -0.5},
    { 0.5,  0.0},
    { 0.5,  0.5},
    { 0.5,  1.0},
    { 0.5,  1.5},
    { 1.0, -0.5},
    { 1.0,  0.0},
    { 1.0,  0.5},
    { 1.0,  1.0},
    { 1.0,  1.5},
    { 1.5, -0.5},
    { 1.5,  0.0},
    { 1.5,  0.5},
    { 1.5,  1.0},
    { 1.5,  1.5}
};

int promptUserForRobotPositionIndex()
{
    // Calculate the actual number of positions in the array
    const size_t numPositions = sizeof(PREDEFINED_POSITIONS) / sizeof(PREDEFINED_POSITIONS[0]);
    
    while (true) {
        cout << "\n"
        << "====================================================\n"
        << " Select Robot Position [1.." << numPositions << "] or 'q' to quit\n"
        << "----------------------------------------------------\n";
        
        // Imprimir las posiciones de forma dinámica
        for (size_t i = 0; i < numPositions; i++) {
            cout << " " << (i + 1) << ")  ("
                    << PREDEFINED_POSITIONS[i][0] << ", " 
                    << PREDEFINED_POSITIONS[i][1] << ")\n";
        }
        
        cout << "----------------------------------------------------\n"
            << "Choose an option: ";


        std::string input;
        cin >> input;

        if (input == "q" || input == "Q") {
            // Return a sentinel value (e.g., -1) indicating we should quit
            return -1;
        }
        try {
            int index = std::stoi(input);
            // Use the actual number of positions for validation
            const size_t numPositions = sizeof(PREDEFINED_POSITIONS) / sizeof(PREDEFINED_POSITIONS[0]);
            if (index >= 1 && index <= static_cast<int>(numPositions)) {
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
    double robotZ = 0.782; // Altura de la base
    sendCoordinates(sock, robotX, robotY, robotZ); // Envía posicion

    // Clear the screen to proceed
    system("cls");



    instrument gimbal; // Gimbal Mechanism
    gimbal.setSerialNo_MotorX(MOTOR_AXIS_X);
    gimbal.setSerialNo_MotorY(MOTOR_AXIS_Y);
    gimbal.setTransmitterPosition(0, 0, TRANSMITTER_H);

    for (auto& pos : positions) {
        if (!pos.done) {

            sendCoordinates(sock, pos.x, pos.y, pos.z); // Envía coordenadas incluyendo altura
            std::string response = receiveResponse(sock, 2); // Recibe respuesta
            
            if (response == "reachable") {

                // ****************************************************************
                // Receiver Position Input
                // ****************************************************************
                cout << "*********************************************************\n";
                cout << "SAMPLING POINT: X, Y, Z\n";
                cout << "Receiver position set to: (" << pos.x << ", " << pos.y << ", " << pos.z << ")\n";
                cout << "*********************************************************\n\n";

                // ****************************************************************
                // Scenario 1 - Transmitter orientations test with receiver pointing to ceiling
                // ****************************************************************
                
                // Wait for a valid option (C to continue or Q to quit)
                char option;
                while (true) {
                    cout << "Scenario 1 - Transmitter orientations test with receiver pointing to ceiling\n";
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

                cout << "[Info] Robot starting to move\n";
                receiverPointingToCeil(sock); // Send command to receiver to point to ceiling

                std::string confirmation = receiveResponse(sock, 30);
                if (confirmation == "reached") {
                    cout << "[Info] Position reached" << endl;
                } else {
                    cout << "[Info] Position not reached" << endl;
                    continue; // Si no se alcanzó la posición, pasar a la siguiente iteración
                }

                cout << "\nIniciando prueba con " << K_ORIENTATIONS << " orientaciones diferentes\n";
                cout << "------------------------------------------------\n";
                
                // Bucle para recorrer todas las orientaciones predefinidas
                for (size_t i = 0; i < K_ORIENTATIONS; i++) {
                    double inclination = PREDEFINED_ORIENTATIONS[i][0];
                    double azimuth = PREDEFINED_ORIENTATIONS[i][1];
                    
                    cout << "Orientación " << (i + 1) << "/" << K_ORIENTATIONS 
                         << ": Inclinación = " << inclination 
                         << ", Azimuth = " << azimuth << "\n";
                    
                    // Aplicar la orientación al transmisor
                    gimbal.setTransmitterOrientation(inclination, azimuth);
                    
                    // Esperar 10 segundos
                    cout << "Esperando 10 segundos...\n";
                    Sleep(10000); // 10 segundos en milisegundos
                }
                
                cout << "Prueba de orientaciones completada\n";
                cout << "Scenario 1: Ready!\n\n\n";
                // ****************************************************************
                

                
                // ****************************************************************
                // Scenario 2 - Pointing transmitter to receiver and receiver to transmitter
                // ****************************************************************

                cout << "\nScenario 2: Waiting for alignment...\n";
                gimbal.transmitterPointingToReceiver_New(-pos.x, -pos.y, pos.z); // ajuste de coordenadas con altura dinámica
                cout << "Esperando 10 segundos...\n";
                Sleep(10000); // 10 segundos en milisegundos
                cout << "Scenario 2: Ready!\n\n\n";

                // ****************************************************************
                // NEXT POSITION
                // ****************************************************************


                // Actualizar la posición actual indicando que ya se completó
                pos.done = 1;

                // Abrir el archivo de posiciones para escribir las actualizaciones
                std::ofstream posFile(PATH_POSITIONS_FILE);
                if (posFile.is_open()) {
                    // Reescribir todas las posiciones actualizadas (suponiendo que 'positions' es el vector actualizado)
                    for (const auto& p : positions) {
                        posFile << p.x << " " << p.y << " " << p.z << " " << p.done << "\n";
                    }
                    posFile.close();
                } else {
                    std::cerr << "[Error] No se pudo abrir el archivo de posiciones.\n";
                }
                // ------------------------------------------------------------------

                while (true) {
                    cout << "Next position? \n";
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
                system("cls");

            } else {
                //cout << "Position is not obtainable" << endl;
            }
        }
    }

    // Cerrar socket, si es necesario
    closesocket(sock);
    WSACleanup();

    return 0;
}
