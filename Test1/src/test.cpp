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

// *****************************************************************************
// Hiperparametros configurables
// *****************************************************************************
const LPCWSTR COM_PORT = L"COM4";       // Puerto serial para el control del LED
const int STABILIZATION_TIME_MS = 1000;  // Tiempo de estabilización en milisegundos
const int BACKGROUND_TIME_SEC = 10;      // Tiempo de adquisición de background en segundos
const int ORIENTATION_TIME_SEC = 10;    // Tiempo de adquisición por cada orientación en segundos

// Configuración para la adquisición de DAQ
int32 fSample = 1000;  // Frecuencia de muestreo: 1000 Hz
int32 nSamples = ORIENTATION_TIME_SEC * fSample;  // Muestras según tiempo configurado

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
    initializeWinsock(); // Initialize Winsock
    SOCKET sock = connectToServer("127.0.0.1", 12345); // Connect to server
    vector<Position> positions; // Vector of positions
    loadPositions(PATH_POSITIONS_FILE, positions); // Load positions

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
                    continue; // If position was not reached, skip this iteration
                }
                
                // Create CSV file for all measurements
                std::string csv_filename = "data_" + 
                                         std::to_string(pos.x) + "_" + 
                                         std::to_string(pos.y) + "_" + 
                                         std::to_string(pos.z) + ".csv";
                                         
                std::ofstream csv_file(csv_filename);
                if (!csv_file.is_open()) {
                    std::cerr << "Error creating CSV file: " << csv_filename << std::endl;
                    continue;
                }
                
                // Write CSV header
                csv_file << "x,y,z,inclinacion,azimuth,stage,medida_daq" << std::endl;
                
                // Open serial port for LED control
                cout << "\nOpening serial port " << wstring(COM_PORT).c_str() << " for LED control...\n";
                HANDLE serialPort = instrument::openSerialPort(COM_PORT);
                if (serialPort == INVALID_HANDLE_VALUE) {
                    cout << "Failed to open serial port. Continuing without LED control.\n";
                } else {
                    cout << "Serial port opened successfully.\n";
                    
                    // Turn off the LED for background acquisition
                    cout << "Turning LED off for background measurement...\n";
                    gimbal.turnOff(serialPort);
                    Sleep(STABILIZATION_TIME_MS); // Wait for LED to stabilize
                    
                    // Acquire background DAQ data
                    cout << "Acquiring background data for " << BACKGROUND_TIME_SEC << " seconds...\n";
                    int32 backgroundSamples = BACKGROUND_TIME_SEC * fSample; // Seconds at 1000 Hz
                    float64* background_data = new float64[backgroundSamples];
                    
                    int background_read = AcquireDataFromDAQ(backgroundSamples, fSample, background_data);
                    
                    // Save background data to CSV
                    if (background_read > 0) {
                        cout << "Successfully acquired " << background_read << " background samples.\n";
                        
                        // Save each background sample to CSV file
                        for (int j = 0; j < background_read; j++) {
                            csv_file << pos.x << "," 
                                     << pos.y << "," 
                                     << pos.z << "," 
                                     << 0.0 << ","  // No inclination for background
                                     << 0.0 << ","  // No azimuth for background
                                     << "background" << ","
                                     << background_data[j] << std::endl;
                        }
                    } else {
                        cout << "Error acquiring background data from DAQ.\n";
                        // Write a line with null value for background
                        csv_file << pos.x << "," 
                                 << pos.y << "," 
                                 << pos.z << "," 
                                 << 0.0 << ","
                                 << 0.0 << ","
                                 << "background" << ",NA" << std::endl;
                    }
                    
                    // Free memory for background data
                    delete[] background_data;
                    
                    // Turn LED back on for scenarios 1 and 2
                    cout << "Turning LED on for main measurements...\n";
                    gimbal.turnOn(serialPort);
                    Sleep(STABILIZATION_TIME_MS); // Wait for LED to stabilize
                }
                
                cout << "\nStarting test with " << K_ORIENTATIONS << " different orientations\n";
                cout << "------------------------------------------------\n";
                
                // Loop through all predefined orientations
                for (size_t i = 0; i < K_ORIENTATIONS; i++) {
                    double inclination = PREDEFINED_ORIENTATIONS[i][0];
                    double azimuth = PREDEFINED_ORIENTATIONS[i][1];
                    
                    cout << "Orientation " << (i + 1) << "/" << K_ORIENTATIONS 
                         << ": Inclination = " << inclination 
                         << ", Azimuth = " << azimuth << "\n";
                    
                    // Apply orientation to transmitter
                    gimbal.setTransmitterOrientation(inclination, azimuth);
                    
                    // Wait briefly for motor stabilization
                    Sleep(STABILIZATION_TIME_MS * 2); // Double stabilization time for motors
                    
                    // Allocate memory for DAQ data
                    float64* daq_data = new float64[nSamples];
                    
                    cout << "Acquiring data for " << ORIENTATION_TIME_SEC << " seconds...\n";
                    
                    // Acquire data from DAQ
                    int read_samples = AcquireDataFromDAQ(nSamples, fSample, daq_data);
                    
                    if (read_samples > 0) {
                        cout << "Successfully acquired " << read_samples << " samples.\n";
                        
                        // Save each sample to CSV file
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
                        cout << "Error acquiring data from DAQ.\n";
                        // Write a line with null value for this orientation
                        csv_file << pos.x << "," 
                                 << pos.y << "," 
                                 << pos.z << "," 
                                 << inclination << "," 
                                 << azimuth << ","
                                 << "direction" << ",NA" << std::endl;
                    }
                    
                    // Free memory
                    delete[] daq_data;
                }
                
                cout << "Orientation test completed.\n";
                cout << "Scenario 1: Ready!\n\n\n";
                // ****************************************************************
                

                
                // ****************************************************************
                // Scenario 2 - Pointing transmitter to receiver and receiver to transmitter
                // ****************************************************************

                cout << "\nScenario 2: Waiting for alignment...\n";
                gimbal.transmitterPointingToReceiver_New(-pos.x, -pos.y, pos.z); // coordinate adjustment with dynamic height
                
                // Wait briefly for transmitter positioning
                Sleep(1000); // 1 second for stabilization
                
                // Allocate memory for DAQ data
                float64* daq_data = new float64[nSamples];
                
                cout << "Acquiring data for " << ORIENTATION_TIME_SEC << " seconds for 'distance' scenario...\n";
                
                // Acquire data from DAQ
                int read_samples = AcquireDataFromDAQ(nSamples, fSample, daq_data);
                
                if (read_samples > 0) {
                    cout << "Successfully acquired " << read_samples << " samples.\n";
                    
                    // Save each sample to CSV file (same file as scenario 1)
                    for (int j = 0; j < read_samples; j++) {
                        csv_file << pos.x << "," 
                                 << pos.y << "," 
                                 << pos.z << "," 
                                 << 0.0 << ","  // No inclination defined for this scenario
                                 << 0.0 << ","  // No azimuth defined for this scenario
                                 << "distance" << ","
                                 << daq_data[j] << std::endl;
                    }
                } else {
                    cout << "Error acquiring data from DAQ.\n";
                    // Write a line with null value for this scenario
                    csv_file << pos.x << "," 
                             << pos.y << "," 
                             << pos.z << "," 
                             << 0.0 << ","  // No inclination defined for this scenario
                             << 0.0 << ","  // No azimuth defined for this scenario
                             << "distance" << ",NA" << std::endl;
                }
                
                // Free memory
                delete[] daq_data;
                
                // Close the CSV file after both scenarios are complete
                csv_file.close();
                cout << "Data acquisition complete. Data saved to: " << csv_filename << "\n";
                cout << "Scenario 2: Ready!\n\n\n";
                
                // Close serial port if it was opened
                if (serialPort != INVALID_HANDLE_VALUE) {
                    cout << "Closing serial port...\n";
                    gimbal.closeSerialPort(serialPort);
                    cout << "Serial port closed.\n";
                }

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
