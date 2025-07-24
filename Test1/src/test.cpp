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
#include <NIDAQmx.h>   // Para la adquisición de datos DAQ
#include <chrono>      // Para mediciones de tiempo
#include <thread>      // Para sleep_for

#include "stdafx.h"    // If using precompiled headers
#include "instrument.h"
#include <vector>
#include <fstream>

#include "positions.h"     // For position structs/functions (if needed)
#include "network_utils.h" // For connectToServer, sendCoordinates, sendMessage, receiveResponse, etc.

using namespace std;
// 302,x Y 44,y

// Macro para verificar errores de DAQ
#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else

// Configuración para la adquisición de DAQ
int32 fSample = 1000;  // Frecuencia de muestreo: 1000 Hz
int32 nSamples = 10 * fSample;  // 10 segundos a 1000 Hz = 10,000 muestras

// Función para adquirir datos de la DAQ
// nSamples: número de muestras a adquirir
// fSample: frecuencia de muestreo en Hz
// data: array para almacenar los datos adquiridos (debe ser preasignado)
// Devuelve: número de muestras adquiridas o 0 si hay error
int AcquireDataFromDAQ(int32 nSamples, int32 fSample, float64* data) {
    int32       error = 0;
    TaskHandle  taskHandle = 0;
    int32       read = 0;
    char        errBuff[2048] = { '\0' };

    try {
        // Crear y configurar la tarea
        DAQmxErrChk(DAQmxCreateTask("", &taskHandle));
        DAQmxErrChk(DAQmxCreateAIVoltageChan(
            taskHandle,
            "Dev1/ai1",       // Canal de entrada (ajustar según la configuración de hardware)
            "",
            DAQmx_Val_RSE,    // Modo de conexión (RSE - Single-Ended)
            -10.0,           // Rango de voltaje mínimo
            10.0,            // Rango de voltaje máximo
            DAQmx_Val_Volts, // Unidades de medida (Voltios)
            NULL
        ));
        
        // Configurar reloj de muestreo
        DAQmxErrChk(DAQmxCfgSampClkTiming(
            taskHandle,
            "",                // Usar reloj interno
            fSample,          // Frecuencia de muestreo
            DAQmx_Val_Rising,
            DAQmx_Val_FiniteSamps,
            nSamples          // Número de muestras a adquirir
        ));

        // Iniciar la tarea
        DAQmxErrChk(DAQmxStartTask(taskHandle));

        // Leer los datos
        DAQmxErrChk(DAQmxReadAnalogF64(
            taskHandle,
            nSamples,         // Leer exactamente nSamples
            200.0,            // Timeout en segundos
            DAQmx_Val_GroupByChannel,
            data,             // Array para almacenar los datos
            nSamples,         // Tamaño del buffer
            &read,            // Número de muestras leídas
            NULL
        ));

    Error:  // Manejo de errores
        if (DAQmxFailed(error))
            DAQmxGetExtendedErrorInfo(errBuff, 2048);

        if (taskHandle != 0) {
            DAQmxStopTask(taskHandle);
            DAQmxClearTask(taskHandle);
        }

        if (DAQmxFailed(error)) {
            std::cerr << "Error DAQmx: " << errBuff << std::endl;
            return 0;
        }
    }
    catch (const std::exception& ex) {
        std::cerr << "Excepción estándar: " << ex.what() << std::endl;
        return 0;
    }
    
    return read;
}

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

                // Crear un archivo CSV para guardar todas las mediciones
                std::string csv_filename = "data_" + 
                                           std::to_string(pos.x) + "_" + 
                                           std::to_string(pos.y) + "_" + 
                                           std::to_string(pos.z) + ".csv";
                                           
                std::ofstream csv_file(csv_filename);
                if (!csv_file.is_open()) {
                    std::cerr << "Error al crear el archivo CSV: " << csv_filename << std::endl;
                    continue;
                }
                
                // Escribir encabezado del CSV
                csv_file << "x,y,z,inclinacion,azimuth,stage,medida_daq" << std::endl;
                
                cout << "\nIniciando prueba con " << K_ORIENTATIONS << " orientaciones diferentes\n";
                cout << "------------------------------------------------\n";
                
                // Configuración para la adquisición de DAQ
                int32 fSample = 1000;  // Frecuencia de muestreo: 1000 Hz
                int32 nSamples = 10 * fSample;  // 10 segundos a 1000 Hz = 10,000 muestras
                
                // Bucle para recorrer todas las orientaciones predefinidas
                for (size_t i = 0; i < K_ORIENTATIONS; i++) {
                    double inclination = PREDEFINED_ORIENTATIONS[i][0];
                    double azimuth = PREDEFINED_ORIENTATIONS[i][1];
                    
                    cout << "Orientación " << (i + 1) << "/" << K_ORIENTATIONS 
                         << ": Inclinación = " << inclination 
                         << ", Azimuth = " << azimuth << "\n";
                    
                    // Aplicar la orientación al transmisor
                    gimbal.setTransmitterOrientation(inclination, azimuth);
                    
                    // Esperar un breve momento para que el motor se estabilice
                    Sleep(1000); // 1 segundo
                    
                    // Reservar memoria para los datos de la DAQ
                    float64* daq_data = new float64[nSamples];
                    
                    cout << "Adquiriendo datos durante 10 segundos...\n";
                    
                    // Adquirir datos de la DAQ
                    int read_samples = AcquireDataFromDAQ(nSamples, fSample, daq_data);
                    
                    if (read_samples > 0) {
                        cout << "Se adquirieron " << read_samples << " muestras correctamente.\n";
                        
                        // Guardar cada muestra en el archivo CSV
                        for (int j = 0; j < read_samples; j++) {
                            csv_file << pos.x << "," 
                                     << pos.y << "," 
                                     << pos.z << "," 
                                     << inclination << "," 
                                     << azimuth << "," 
                                     << "direction" << ","
                                     << daq_data[j] << std::endl;
                        }
                    } else {
                        cout << "Error en la adquisición de datos de la DAQ.\n";
                        // Escribir una línea con valor nulo para esta orientación
                        csv_file << pos.x << "," 
                                 << pos.y << "," 
                                 << pos.z << "," 
                                 << inclination << "," 
                                 << azimuth << ","
                                 << "direction" << ",NA" << std::endl;
                    }
                    
                    // Liberar memoria
                    delete[] daq_data;
                }
                
                // Cerrar el archivo CSV
                csv_file.close();
                cout << "Prueba de orientaciones completada. Datos guardados en: " << csv_filename << "\n";
                cout << "Scenario 1: Ready!\n\n\n";
                // ****************************************************************
                

                
                // ****************************************************************
                // Scenario 2 - Pointing transmitter to receiver and receiver to transmitter
                // ****************************************************************

                cout << "\nScenario 2: Waiting for alignment...\n";
                gimbal.transmitterPointingToReceiver_New(-pos.x, -pos.y, pos.z); // ajuste de coordenadas con altura dinámica
                
                // Esperar un breve momento para que el transmisor se posicione
                Sleep(1000); // 1 segundo para estabilizar
                
                // Reservar memoria para los datos de la DAQ
                float64* daq_data = new float64[nSamples];
                
                cout << "Adquiriendo datos durante 10 segundos para escenario 'distance'...\n";
                
                // Adquirir datos de la DAQ
                int read_samples = AcquireDataFromDAQ(nSamples, fSample, daq_data);
                
                if (read_samples > 0) {
                    cout << "Se adquirieron " << read_samples << " muestras correctamente.\n";
                    
                    // Guardar cada muestra en el archivo CSV
                    for (int j = 0; j < read_samples; j++) {
                        csv_file << pos.x << "," 
                                 << pos.y << "," 
                                 << pos.z << "," 
                                 << 0.0 << ","  // No hay inclinación definida para este escenario
                                 << 0.0 << ","  // No hay azimuth definido para este escenario
                                 << "distance" << ","
                                 << daq_data[j] << std::endl;
                    }
                } else {
                    cout << "Error en la adquisición de datos de la DAQ.\n";
                    // Escribir una línea con valor nulo para este escenario
                    csv_file << pos.x << "," 
                             << pos.y << "," 
                             << pos.z << "," 
                             << 0.0 << ","  // No hay inclinación definida para este escenario
                             << 0.0 << ","  // No hay azimuth definido para este escenario
                             << "distance" << ",NA" << std::endl;
                }
                
                // Liberar memoria
                delete[] daq_data;
                
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
