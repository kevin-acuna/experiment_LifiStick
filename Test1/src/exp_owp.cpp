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
#include <iomanip>     // Para formatear timestamp
#include <sstream>     // Para construir strings

#include "stdafx.h"    // If using precompiled headers
#include "instrument.h"
#include <vector>
#include <fstream>

#include "positions.h"     // For position structs/functions (if needed)
#include "network_utils.h" // For connectToServer, sendCoordinates, sendMessage, receiveResponse, etc.

// Sistema Gimbal 1 (antiguo)
#define MOTOR_AXIS_X_CENTER 27006796  // External axis
#define MOTOR_AXIS_Y_CENTER 27007072  // Internal axis
// Sistema Gimbal 2 (nuevo)
#define MOTOR_AXIS_X_OWP 27267164  // External axis
#define MOTOR_AXIS_Y_OWP 27602122  // Internal axis

using namespace std;

// Macro para verificar errores de DAQ
#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else

// *****************************************************************************
// Hiperparametros configurables
// *****************************************************************************
const bool USE_COM_PORT = false;            // Flag para activar/desactivar control del LED via COM
                                           // true: controla encendido/apagado del LED via COM
                                           // false: asume que el LED está encendido, no usa COM
const bool AUTO_CONTINUE = false;           // Flag para modo automático
                                           // true: requiere presionar C/Q entre posiciones (modo manual)
                                           // false: continúa automáticamente sin intervención del usuario
#define COM_PORT_NAME "COM4"           // Puerto serial para el control del LED (sin L prefix)
const int STABILIZATION_TIME_MS = 2000;  // Tiempo de estabilización en milisegundos
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

// Función para generar timestamp como string
std::string getTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    
    std::tm timeinfo;
    localtime_s(&timeinfo, &in_time_t);
    
    std::ostringstream oss;
    oss << std::put_time(&timeinfo, "%Y%m%d_%H%M%S");
    return oss.str();
}

// *****************************************************************************
// Adjust according to your experiment
// *****************************************************************************
const char* PATH_POSITIONS_FILE = "src/positionsToSample/owp_pos.txt";

const double TRANSMITTER_H = 2.00; // altitude in meters


int MOTOR_AXIS_X = MOTOR_AXIS_X_OWP;  // External axis
int MOTOR_AXIS_Y = MOTOR_AXIS_Y_OWP;  // Internal axis
// *****************************************************************************


// Orientaciones predefinidas para el transmisor {inclinacion, azimuth}
// inclinacion: angulo con respecto a la vertical (0-180 grados)
// azimuth: angulo en el plano XY desde el eje X (0-360 grados)
/*
// 45° -- 2.4x2.4m
// 35° -- 2.0x2.0m
static const double PREDEFINED_ORIENTATIONS[][2] = {
    {30,30},
    {0,0},
    {45,0}, 
    {45,90},
    {45,180},
    {45,270},
    {35,0},
    {35,90},
    {35,180},
    {35,270},
};
*/
static const double PREDEFINED_ORIENTATIONS[][2] = {
    {0,0},
    {20,180},
    {20,270}, 
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

// Estructura para almacenar coordenadas personalizadas
struct CustomPosition {
    double x;
    double y;
    bool isCustom;
};

CustomPosition customPos = {0.0, 0.0, false};

int promptUserForRobotPositionIndex()
{
    // Calculate the actual number of positions in the array
    const size_t numPositions = sizeof(PREDEFINED_POSITIONS) / sizeof(PREDEFINED_POSITIONS[0]);
    
    while (true) {
        cout << "\n"
        << "====================================================\n"
        << " Select Robot Position [1.." << numPositions << "], 'custom', or 'q' to quit\n"
        << "----------------------------------------------------\n";
        
        // Imprimir las posiciones de forma dinámica
        for (size_t i = 0; i < numPositions; i++) {
            cout << " " << (i + 1) << ")  ("
                    << PREDEFINED_POSITIONS[i][0] << ", " 
                    << PREDEFINED_POSITIONS[i][1] << ")\n";
        }
        
        cout << " custom) Ingresar posición personalizada\n";
        cout << "----------------------------------------------------\n"
            << "Choose an option: ";


        std::string input;
        cin >> input;

        if (input == "q" || input == "Q") {
            // Return a sentinel value (e.g., -1) indicating we should quit
            return -1;
        }
        
        // Check for custom position option
        if (input == "custom" || input == "Custom" || input == "CUSTOM" || input == "c" || input == "C") {
            cout << "\n[Posición Personalizada]\n";
            cout << "Ingrese coordenada X (metros): ";
            
            while (!(cin >> customPos.x)) {
                cout << "[Error] Valor inválido. Ingrese coordenada X (metros): ";
                cin.clear();
                cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            }
            
            cout << "Ingrese coordenada Y (metros): ";
            while (!(cin >> customPos.y)) {
                cout << "[Error] Valor inválido. Ingrese coordenada Y (metros): ";
                cin.clear();
                cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            }
            
            customPos.isCustom = true;
            cout << "Posición personalizada establecida: (" << customPos.x << ", " << customPos.y << ")\n";
            
            // Return a special value to indicate custom position (e.g., -2)
            return -2;
        }
        
        try {
            int index = std::stoi(input);
            // Use the actual number of positions for validation
            const size_t numPositions = sizeof(PREDEFINED_POSITIONS) / sizeof(PREDEFINED_POSITIONS[0]);
            if (index >= 1 && index <= static_cast<int>(numPositions)) {
                customPos.isCustom = false;
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

    double robotX, robotY;
    double robotZ = 0.782; // Altura de la base
    
    // Check if custom position was selected
    if (index == -2) {
        cout << "\n[Info] Usando posición personalizada: (" << customPos.x << ", " << customPos.y << ")\n";
        robotX = customPos.x;
        robotY = customPos.y;
    } else {
        robotX = PREDEFINED_POSITIONS[index - 1][0];
        robotY = PREDEFINED_POSITIONS[index - 1][1];
    }
    
    sendCoordinates(sock, robotX, robotY, robotZ); // Envía posicion

    // Clear the screen to proceed
    system("cls");

    // Open serial port for LED control (only if USE_COM_PORT is enabled)
    HANDLE serialPort = INVALID_HANDLE_VALUE;
    if (USE_COM_PORT) {
        cout << "\nOpening serial port " << COM_PORT_NAME << " for LED control...\n";
        serialPort = instrument::openSerialPort(L"COM4"); // Hardcoded to avoid conversion issues
        if (serialPort == INVALID_HANDLE_VALUE) {
            cout << "Failed to open serial port. Continuing without LED control.\n";
        } else {
            cout << "Serial port opened successfully.\n";
        }
    } else {
        cout << "\nCOM port control disabled. Assuming LED is already on.\n";
    }

    instrument gimbal; // Gimbal Mechanism
    gimbal.setSerialNo_MotorX(MOTOR_AXIS_X);
    gimbal.setSerialNo_MotorY(MOTOR_AXIS_Y);
    gimbal.setTransmitterPosition(0, 0, TRANSMITTER_H);

    // Mover ambos motores a la posición inicial (0°, 0°)
    cout << "\nMoviendo motores a posición inicial (0°, 0°)...\n";
    cout << "NOTA: Los offsets de calibración se aplican automáticamente.\n";
    gimbal.rotateMotorX(0.0);
    Sleep(2000); // Esperar estabilización
    gimbal.rotateMotorY(0.0);
    cout << "Motores en posición inicial.\n";
    Sleep(2000); // Esperar estabilización

    // Crear archivo CSV único con timestamp para todas las mediciones
    std::string csv_filename = "data_" + getTimestamp() + ".csv";
    std::ofstream csv_file(csv_filename);
    if (!csv_file.is_open()) {
        std::cerr << "Error creating CSV file: " << csv_filename << std::endl;
        // Close resources and exit
        if (USE_COM_PORT && serialPort != INVALID_HANDLE_VALUE) {
            gimbal.closeSerialPort(serialPort);
        }
        closesocket(sock);
        WSACleanup();
        return 1;
    }
    
    // Write CSV header once
    csv_file << "x,y,z,inclinacion,azimuth,mode,medida_daq" << std::endl;
    cout << "CSV file created: " << csv_filename << "\n\n";

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
                
                // Wait for a valid option (C to continue or Q to quit) - only if AUTO_CONTINUE is true
                char option = 'C'; // Default to continue
                if (AUTO_CONTINUE) {
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
                } else {
                    cout << "[AUTO MODE] Starting Scenario 1 - Transmitter orientations test with receiver pointing to ceiling\n";
                }

                // Turn off the LED for background acquisition
                if (USE_COM_PORT) {
                    cout << "Turning LED off for background measurement...\n";
                    gimbal.turnOff(serialPort);
                } else {
                    cout << "COM port control disabled. Skipping LED off command.\n";
                }
                    
                cout << "[Info] Robot starting to move\n";
                receiverPointingToCeil(sock); // Send command to receiver to point to ceiling

                std::string confirmation = receiveResponse(sock, 30);
                if (confirmation == "reached") {
                    cout << "[Info] Position reached" << endl;
                } else {
                    cout << "[Info] Position not reached" << endl;
                    continue; // If position was not reached, skip this iteration
                }
                
                // Acquire background DAQ data only if USE_COM_PORT is enabled
                if (USE_COM_PORT) {
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
                    
                    // Turn LED back on for main measurements
                    cout << "Turning LED on for main measurements...\n";
                    gimbal.turnOn(serialPort);
                    Sleep(10000); // Wait for LED to stabilize - first time turn on.
                } else {
                    cout << "COM port control disabled. Skipping background measurement.\n";
                }
                
                
                cout << "\nStarting test with " << K_ORIENTATIONS << " different orientations\n";
                cout << "------------------------------------------------\n";
                
                // Loop through all predefined orientations
                for (int i = 0; i < (int)K_ORIENTATIONS; i++) {
                    double inclination = PREDEFINED_ORIENTATIONS[i][0];
                    double azimuth = PREDEFINED_ORIENTATIONS[i][1];
                    
                    cout << "Orientation " << (i + 1) << "/" << K_ORIENTATIONS 
                         << ": Inclination = " << inclination 
                         << ", Azimuth = " << azimuth << "\n";
                    
                    // Apply orientation to transmitter
                    gimbal.setTransmitterOrientation(inclination, fmod(azimuth + 180.0, 360.0));
                    
                    // Wait briefly for motor stabilization
                    Sleep(STABILIZATION_TIME_MS ); // Double stabilization time for motors
                    
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
                                     << "r_vertical" << ","
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
                                 << "r_vertical" << ",NA" << std::endl;
                    }
                    
                    // Free memory
                    delete[] daq_data;
                }
                
                cout << "Orientation test completed.\n";
                cout << "Scenario 1: Ready!\n\n\n";
                
                // ****************************************************************
                // NEXT POSITION
                // ****************************************************************
                receiverFinished(sock);

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
                    std::cerr << "[Error] Could not open positions file.\n";
                }
                // ------------------------------------------------------------------

                if (AUTO_CONTINUE) {
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
                } else {
                    cout << "[AUTO MODE] Moving to next position automatically...\n";
                    Sleep(2000); // Brief pause before moving to next position
                }
                cout << endl;
                system("cls");

            } else {
                //cout << "Position is not obtainable" << endl;
            }
        }
    }

    // Close CSV file at the end of all acquisitions
    csv_file.close();
    cout << "\nAll data acquisition complete. Data saved to: " << csv_filename << "\n";

    // Cerrar socket, si es necesario
    // Close serial port if it was opened
    if (USE_COM_PORT && serialPort != INVALID_HANDLE_VALUE) {
        cout << "Closing serial port...\n";
        gimbal.closeSerialPort(serialPort);
        cout << "Serial port closed.\n";
    }
    closesocket(sock);
    WSACleanup();

    return 0;
}
