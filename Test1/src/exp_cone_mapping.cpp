#define NOMINMAX
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")

#include <Windows.h>
#include <iostream>
#include <limits>
#include <istream>
#include <cctype>
#include <string>
#include <NIDAQmx.h>
#include <chrono>
#include <thread>
#include <vector>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <sstream>

#include "stdafx.h"
#include "instrument.h"
#include "positions.h"
#include "network_utils.h"

using namespace std;

// Macro para verificar errores de DAQ
#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else

// *****************************************************************************
// Hiperparámetros configurables
// *****************************************************************************

// Sistema Gimbal (IDs de motores)
#define MOTOR_AXIS_X_OWP 27267164  // External axis
#define MOTOR_AXIS_Y_OWP 27602122  // Internal axis
int MOTOR_AXIS_X = MOTOR_AXIS_X_OWP;  // External axis
int MOTOR_AXIS_Y = MOTOR_AXIS_Y_OWP;  // Internal axis

// Altura del transmisor
const double TRANSMITTER_H = 2.00; // metros
const double ROBOT_OFFSET_X = -0.01; // offset en X del robot en metros
const double ROBOT_OFFSET_Y = 0.012; // offset en Y del robot en metros

// Altura de la base del robot
const double ROBOT_BASE_Z = 0.646;

// Posición del receptor (end-effector del brazo robótico)
const double RECEIVER_X = 0.0;
const double RECEIVER_Y = 0.0;
const double RECEIVER_Z = 1.0;  // end-effector en [0, 0, 1]

// ---------------------------------------------------------------------------
// Resolución del mapeo del cono
// ---------------------------------------------------------------------------
// INCLINATION_STEP: resolución de inclinación en grados.
//   Ejemplo: 2.0 significa que se muestrea cada 2° de inclinación.
//   El rango va de INCLINATION_START a INCLINATION_END inclusive.
const double INCLINATION_START = 7.0;   // grados (0 = apuntando hacia abajo / vertical)
const double INCLINATION_END   = 45.0;  // grados
const double INCLINATION_STEP  = 1.0;   // resolución de inclinación en grados

// AZIMUTH_STEP: resolución de azimuth en grados.
//   Ejemplo: 5.0 significa que se muestrea cada 5° de azimuth.
//   El rango de azimuth es siempre de 0° a 360° (exclusive de 360° ya que 360°=0°).
//   En inclinación 0° el azimuth es irrelevante, solo se toma una medición.
const double AZIMUTH_STEP = 1.0;        // resolución de azimuth en grados

// ---------------------------------------------------------------------------
// Tiempos
// ---------------------------------------------------------------------------
const int STABILIZATION_TIME_MS = 500;  // estabilización después de mover motores (ms)
const int ACQUISITION_TIME_SEC  = 1;     // adquisición por orientación: 1s -> 1K muestras a 1kHz

// ---------------------------------------------------------------------------
// Configuración de la DAQ
// ---------------------------------------------------------------------------
int32 fSample  = 1000;                              // Frecuencia de muestreo: 1000 Hz
int32 nSamples = ACQUISITION_TIME_SEC * fSample;     // Muestras por adquisición (1000)


// *****************************************************************************
// Posiciones predefinidas del robot base (X, Y)
// *****************************************************************************
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

// *****************************************************************************
// Función para adquirir datos de la DAQ
// *****************************************************************************
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
            "Dev1/ai1",       // Canal de entrada (ajustar según hardware)
            "",
            DAQmx_Val_RSE,    // Modo de conexión (RSE - Single-Ended)
            -10.0,            // Rango de voltaje mínimo
            10.0,             // Rango de voltaje máximo
            DAQmx_Val_Volts,  // Unidades de medida (Voltios)
            NULL
        ));
        
        // Configurar reloj de muestreo
        DAQmxErrChk(DAQmxCfgSampClkTiming(
            taskHandle,
            "",                // Usar reloj interno
            fSample,           // Frecuencia de muestreo
            DAQmx_Val_Rising,
            DAQmx_Val_FiniteSamps,
            nSamples           // Número de muestras a adquirir
        ));

        // Iniciar la tarea
        DAQmxErrChk(DAQmxStartTask(taskHandle));

        // Leer los datos
        DAQmxErrChk(DAQmxReadAnalogF64(
            taskHandle,
            nSamples,          // Leer exactamente nSamples
            200.0,             // Timeout en segundos
            DAQmx_Val_GroupByChannel,
            data,              // Array para almacenar los datos
            nSamples,          // Tamaño del buffer
            &read,             // Número de muestras leídas
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
// Función para calcular la mediana de un array de muestras
// *****************************************************************************
double computeMedian(float64* data, int n) {
    std::vector<double> sorted(data, data + n);
    std::sort(sorted.begin(), sorted.end());
    if (n % 2 == 0)
        return (sorted[n / 2 - 1] + sorted[n / 2]) / 2.0;
    else
        return sorted[n / 2];
}

// *****************************************************************************
// Función para generar timestamp como string
// *****************************************************************************
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
// MAIN
// *****************************************************************************
int main()
{
    system("chcp 65001 > nul"); // Habilitar UTF-8 en consola

    // =========================================================================
    // Pre-cálculo: construir la lista completa de orientaciones (inc, az)
    // =========================================================================
    struct Orientation { double inclination; double azimuth; };
    std::vector<Orientation> orientations;

    for (double inc = INCLINATION_START; inc <= INCLINATION_END + 1e-9; inc += INCLINATION_STEP) {
        if (inc < 1e-9) {
            // Inclinación ~0°: el azimuth es irrelevante, solo una medición
            orientations.push_back({ 0.0, 0.0 });
        } else {
            for (double az = 0.0; az < 360.0 - 1e-9; az += AZIMUTH_STEP) {
                orientations.push_back({ inc, az });
            }
        }
    }

    int totalOrientations = static_cast<int>(orientations.size());
    double estimatedTimeSec = totalOrientations * (STABILIZATION_TIME_MS / 1000.0 + ACQUISITION_TIME_SEC);
    double estimatedTimeMin = estimatedTimeSec / 60.0;

    // =========================================================================
    // Mostrar información del experimento
    // =========================================================================
    cout << "=========================================================\n";
    cout << " EXPERIMENTO: Mapeo del Cono de Irradiación del LED\n";
    cout << "=========================================================\n";
    cout << "Configuración:\n";
    cout << "  Inclinación: " << INCLINATION_START << "° a " << INCLINATION_END
         << "° (paso: " << INCLINATION_STEP << "°)\n";
    cout << "  Azimuth: 0° a 360° (paso: " << AZIMUTH_STEP << "°)\n";
    cout << "  Estabilización: " << STABILIZATION_TIME_MS << " ms\n";
    cout << "  Adquisición: " << ACQUISITION_TIME_SEC << " s (" 
         << nSamples << " muestras a " << fSample << " Hz)\n";
    cout << "  Posición receptor (end-effector): (" 
         << RECEIVER_X << ", " << RECEIVER_Y << ", " << RECEIVER_Z << ")\n";
    cout << "  Total de orientaciones a medir: " << totalOrientations << "\n";
    cout << "  Tiempo estimado: " << fixed << setprecision(1) 
         << estimatedTimeMin << " minutos\n";
    cout << "=========================================================\n\n";

    // =========================================================================
    // Conexión al servidor del brazo robótico
    // =========================================================================
    initializeWinsock();
    SOCKET sock = connectToServer("127.0.0.1", 12345);

    // =========================================================================
    // Selección de posición base del robot (igual que exp_owp.cpp)
    // =========================================================================
    int index = promptUserForRobotPositionIndex();
    if (index == -1) {
        cout << "User selected 'q' to quit. Exiting.\n";
        closesocket(sock);
        WSACleanup();
        return 0;
    }

    double robotX, robotY;

    // Check if custom position was selected
    if (index == -2) {
        cout << "\n[Info] Usando posición personalizada: (" << customPos.x << ", " << customPos.y << ")\n";
        robotX = customPos.x;
        robotY = customPos.y;
    } else {
        robotX = PREDEFINED_POSITIONS[index - 1][0];
        robotY = PREDEFINED_POSITIONS[index - 1][1];
    }
    
    robotX += ROBOT_OFFSET_X;
    robotY += ROBOT_OFFSET_Y;
    
    // Enviar posición base del robot
    sendCoordinates(sock, robotX, robotY, ROBOT_BASE_Z);

    // Clear the screen to proceed
    system("cls");

    // =========================================================================
    // Enviar posición del end-effector y orientar receptor
    // =========================================================================
    cout << "Enviando end-effector a posición (" 
         << RECEIVER_X << ", " << RECEIVER_Y << ", " << RECEIVER_Z << ")...\n";
    sendCoordinates(sock, RECEIVER_X, RECEIVER_Y, RECEIVER_Z);
    std::string response = receiveResponse(sock, 2);

    if (response != "reachable") {
        cerr << "Error: Posición del receptor no alcanzable. Respuesta: " << response << endl;
        closesocket(sock);
        WSACleanup();
        return 1;
    }
    cout << "Posición alcanzable. Moviendo receptor...\n";

    // Comandar al receptor que apunte al techo (PD mirando hacia arriba)
    receiverPointingToCeil(sock);
    std::string confirmation = receiveResponse(sock, 30);
    if (confirmation != "reached") {
        cerr << "Error: Receptor no alcanzó la posición. Respuesta: " << confirmation << endl;
        closesocket(sock);
        WSACleanup();
        return 1;
    }
    cout << "Receptor en posición y apuntando al techo.\n\n";

    // =========================================================================
    // Configurar gimbal (motores del transmisor)
    // =========================================================================
    instrument gimbal;
    gimbal.setSerialNo_MotorX(MOTOR_AXIS_X);
    gimbal.setSerialNo_MotorY(MOTOR_AXIS_Y);
    gimbal.setTransmitterPosition(0, 0, TRANSMITTER_H);

    // Mover ambos motores a la posición inicial (0°, 0°)
    cout << "Moviendo motores a posición inicial (0°, 0°)...\n";
    cout << "NOTA: Los offsets de calibración se aplican automáticamente.\n";
    gimbal.rotateMotorX(0.0);
    Sleep(2000);
    gimbal.rotateMotorY(0.0);
    Sleep(2000);
    cout << "Motores en posición inicial.\n\n";

    // Confirmación del usuario antes de iniciar
    cout << "¿Desea iniciar el mapeo del cono? (C para continuar, Q para salir): ";
    char option;
    cin >> option;
    option = toupper(option);
    if (option == 'Q') {
        cout << "Experimento cancelado por el usuario.\n";
        receiverFinished(sock);
        closesocket(sock);
        WSACleanup();
        return 0;
    }
    cin.ignore(numeric_limits<streamsize>::max(), '\n');

    // =========================================================================
    // Crear archivo CSV
    // =========================================================================
    std::string timestamp = getTimestamp();
    std::string csv_filename = "cone_mapping_" + timestamp + ".csv";
    std::ofstream csv_file(csv_filename);
    if (!csv_file.is_open()) {
        cerr << "Error: No se pudo crear el archivo CSV: " << csv_filename << endl;
        receiverFinished(sock);
        closesocket(sock);
        WSACleanup();
        return 1;
    }

    // Encabezado del CSV (mismo formato que exp_3D)
    csv_file << "sample_id,x,y,z,inclinacion,azimuth,mode,medida_daq" << endl;
    cout << "Archivo CSV creado: " << csv_filename << "\n\n";

    // Contador para identificador único de muestra
    int sample_counter = 0;

    // =========================================================================
    // Bucle principal: mapeo del cono
    // =========================================================================
    cout << "=========================================================\n";
    cout << " INICIANDO MAPEO DEL CONO\n";
    cout << "=========================================================\n\n";

    int orientationCount = 0;

    for (const auto& ori : orientations) {
        orientationCount++;
        sample_counter++;
        std::string sample_id = timestamp + "_" + std::to_string(sample_counter);

        cout << "---------------------------------------------------\n";
        cout << "Orientación " << orientationCount << "/" << totalOrientations
             << "  |  Inclinación: " << ori.inclination 
             << "°  |  Azimuth: " << ori.azimuth << "°\n";
        cout << "---------------------------------------------------\n";

        // Aplicar orientación al transmisor (con offset de 180° en azimuth, igual que exp_3D)
        gimbal.setTransmitterOrientation(ori.inclination, fmod(ori.azimuth + 180.0, 360.0));

        // Esperar estabilización del motor
        cout << "Estabilización (" << STABILIZATION_TIME_MS << " ms)...\n";
        Sleep(STABILIZATION_TIME_MS);

        // Adquirir datos de la DAQ
        float64* daq_data = new float64[nSamples];
        cout << "Adquiriendo " << nSamples << " muestras (" << ACQUISITION_TIME_SEC << " s)...\n";
        int read_samples = AcquireDataFromDAQ(nSamples, fSample, daq_data);

        if (read_samples > 0) {
            // Guardar cada muestra en el CSV (formato exp_3D)
            for (int j = 0; j < read_samples; j++) {
                csv_file << sample_id << ","
                         << RECEIVER_X << "," 
                         << RECEIVER_Y << "," 
                         << RECEIVER_Z << "," 
                         << ori.inclination << "," 
                         << ori.azimuth << ","
                         << "r_vertical" << ","
                         << daq_data[j] << "\n";
            }
            csv_file.flush(); // Flush para no perder datos si se interrumpe

            // Calcular y mostrar la mediana como feedback
            double median = computeMedian(daq_data, read_samples);
            cout << "OK (" << read_samples << " muestras)  ->  Mediana: " 
                 << fixed << setprecision(6) << median << " V\n\n";
        } else {
            cout << "ERROR en la adquisición de datos.\n\n";
            csv_file << sample_id << ","
                     << RECEIVER_X << "," 
                     << RECEIVER_Y << "," 
                     << RECEIVER_Z << "," 
                     << ori.inclination << "," 
                     << ori.azimuth << ","
                     << "r_vertical" << ",NA\n";
            csv_file.flush();
        }

        // Liberar memoria
        delete[] daq_data;
    }

    // =========================================================================
    // Finalización
    // =========================================================================
    csv_file.close();

    cout << "=========================================================\n";
    cout << " MAPEO DEL CONO COMPLETADO\n";
    cout << "=========================================================\n";
    cout << "Orientaciones registradas: " << orientationCount << "/" << totalOrientations << "\n";
    cout << "Datos guardados en: " << csv_filename << "\n\n";

    // Regresar motores a posición inicial
    cout << "Regresando motores a posición inicial (0°, 0°)...\n";
    gimbal.rotateMotorX(0.0);
    Sleep(1000);
    gimbal.rotateMotorY(0.0);
    Sleep(1000);
    cout << "Motores en posición inicial.\n";

    // Señalar al robot que terminó
    receiverFinished(sock);

    // Cerrar recursos
    closesocket(sock);
    WSACleanup();

    cout << "\nExperimento finalizado exitosamente.\n";
    cout << "Presione Enter para salir...";
    cin.get();

    return 0;
}
