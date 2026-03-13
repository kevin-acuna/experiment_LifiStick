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
#include <direct.h>

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
const double INCLINATION_START = 0.0;   // grados (0 = apuntando hacia abajo / vertical)
const double INCLINATION_END   = 90.0;  // grados
const double INCLINATION_STEP  = 1.0;   // resolución de inclinación en grados

// AZIMUTH_STEP: resolución de azimuth en grados.
//   Ejemplo: 5.0 significa que se muestrea cada 5° de azimuth.
//   El rango de azimuth es siempre de 0° a 360° (exclusive de 360° ya que 360°=0°).
//   En inclinación 0° el azimuth es irrelevante, solo se toma una medición.
const double AZIMUTH_STEP = 5.0;        // resolución de azimuth en grados

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
// Función para calcular la media de un array de muestras
// *****************************************************************************
double computeMean(float64* data, int n) {
    double sum = 0.0;
    for (int i = 0; i < n; i++) sum += data[i];
    return sum / n;
}

// *****************************************************************************
// Función para generar timestamp como string (para nombres de archivo)
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
// Funciones para obtener fecha y hora actuales por separado
// *****************************************************************************
std::string getCurrentDate() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm timeinfo;
    localtime_s(&timeinfo, &in_time_t);
    std::ostringstream oss;
    oss << std::put_time(&timeinfo, "%Y-%m-%d");
    return oss.str();
}

std::string getCurrentTime() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::tm timeinfo;
    localtime_s(&timeinfo, &in_time_t);
    std::ostringstream oss;
    oss << std::put_time(&timeinfo, "%H:%M:%S");
    return oss.str();
}


// *****************************************************************************
// Estado de progreso: guardar/cargar/eliminar
// *****************************************************************************
const std::string STATE_FILE = "output/cone_mapping_state.txt";

struct ResumeState {
    int nextOrientationIndex;  // 0-based: próxima orientación a procesar
    std::string csv_filename;
    std::string timestamp;
    int sample_counter;
    bool valid;
};

void saveState(int nextOrientationIndex, const std::string& csv_filename, 
               const std::string& ts, int sample_counter) {
    std::ofstream f(STATE_FILE);
    if (f.is_open()) {
        f << nextOrientationIndex << "\n"
          << csv_filename << "\n"
          << ts << "\n"
          << sample_counter << "\n";
        f.close();
    }
}

ResumeState loadState() {
    ResumeState state = {0, "", "", 0, false};
    std::ifstream f(STATE_FILE);
    if (f.is_open()) {
        std::string line;
        if (std::getline(f, line)) state.nextOrientationIndex = std::stoi(line);
        if (std::getline(f, line)) state.csv_filename = line;
        if (std::getline(f, line)) state.timestamp = line;
        if (std::getline(f, line)) state.sample_counter = std::stoi(line);
        state.valid = true;
        f.close();
    }
    return state;
}

void deleteState() {
    std::remove(STATE_FILE.c_str());
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

    // =========================================================================
    // Verificar si hay un estado previo para reanudar
    // =========================================================================
    std::string output_dir = "output";
    _mkdir(output_dir.c_str());

    ResumeState resumeState = loadState();
    bool resuming = false;
    int startIndex = 0;
    int sample_counter = 0;
    std::string timestamp;
    std::string csv_filename;
    std::ofstream csv_file;

    if (resumeState.valid) {
        cout << "=========================================================\n";
        cout << " ESTADO PREVIO ENCONTRADO\n";
        cout << "=========================================================\n";
        cout << "  Archivo CSV: " << resumeState.csv_filename << "\n";
        cout << "  Orientaciones completadas: " << resumeState.nextOrientationIndex 
             << "/" << totalOrientations << "\n";
        cout << "  Restantes: " << (totalOrientations - resumeState.nextOrientationIndex) << "\n";
        cout << "=========================================================\n\n";
        cout << "Opciones:\n";
        cout << "  R = Resetear gimbal a (0,0) e iniciar experimento nuevo\n";
        cout << "  C = Continuar desde donde se quedo\n";
        cout << "  Q = Salir\n";
        cout << "Elija una opcion: ";
    } else {
        cout << "\nOpciones:\n";
        cout << "  R = Resetear gimbal a (0,0) antes de iniciar\n";
        cout << "  C = Iniciar sin mover el gimbal (usar posicion actual)\n";
        cout << "  Q = Salir\n";
        cout << "Elija una opcion: ";
    }

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

    if (option == 'R') {
        // Resetear gimbal a (0°, 0°)
        cout << "\nMoviendo motores a posición inicial (0°, 0°)...\n";
        cout << "NOTA: Los offsets de calibración se aplican automáticamente.\n";
        gimbal.rotateMotorX(0.0);
        Sleep(2000);
        gimbal.rotateMotorY(0.0);
        Sleep(2000);
        cout << "Motores en posición inicial.\n\n";

        // Inicio fresco: nuevo CSV
        deleteState();
        timestamp = getTimestamp();
        csv_filename = output_dir + "/cone_mapping_" + timestamp + ".csv";
        csv_file.open(csv_filename);
        if (!csv_file.is_open()) {
            cerr << "Error: No se pudo crear el archivo CSV: " << csv_filename << endl;
            receiverFinished(sock);
            closesocket(sock);
            WSACleanup();
            return 1;
        }
        csv_file << "sample_id,date,time,x,y,z,inclinacion,azimuth,mode,median,mean" << endl;
        cout << "Archivo CSV creado: " << csv_filename << "\n";

        // Confirmación antes de iniciar
        cout << "¿Iniciar mapeo? (C para continuar, Q para salir): ";
        char confirm;
        cin >> confirm;
        confirm = toupper(confirm);
        if (confirm == 'Q') {
            csv_file.close();
            receiverFinished(sock);
            closesocket(sock);
            WSACleanup();
            return 0;
        }
        cin.ignore(numeric_limits<streamsize>::max(), '\n');

        startIndex = 0;
        sample_counter = 0;
        resuming = false;

    } else {
        // Continuar: no mover gimbal
        if (resumeState.valid) {
            // Reanudar sesión previa
            resuming = true;
            startIndex = resumeState.nextOrientationIndex;
            sample_counter = resumeState.sample_counter;
            timestamp = resumeState.timestamp;
            csv_filename = resumeState.csv_filename;

            // Abrir CSV en modo append
            csv_file.open(csv_filename, std::ios::app);
            if (!csv_file.is_open()) {
                cerr << "Error: No se pudo abrir el archivo CSV para append: " << csv_filename << endl;
                receiverFinished(sock);
                closesocket(sock);
                WSACleanup();
                return 1;
            }
            cout << "\nReanudando desde orientación " << (startIndex + 1) << "/" << totalOrientations << "\n";
            cout << "Archivo CSV (append): " << csv_filename << "\n";
        } else {
            // Sin estado previo: inicio fresco sin mover gimbal
            deleteState();
            timestamp = getTimestamp();
            csv_filename = output_dir + "/cone_mapping_" + timestamp + ".csv";
            csv_file.open(csv_filename);
            if (!csv_file.is_open()) {
                cerr << "Error: No se pudo crear el archivo CSV: " << csv_filename << endl;
                receiverFinished(sock);
                closesocket(sock);
                WSACleanup();
                return 1;
            }
            csv_file << "sample_id,date,time,x,y,z,inclinacion,azimuth,mode,median,mean" << endl;
            cout << "\nArchivo CSV creado: " << csv_filename << "\n";
            startIndex = 0;
            sample_counter = 0;
        }
    }

    // =========================================================================
    // Bucle principal: mapeo del cono
    // =========================================================================
    cout << "\n=========================================================\n";
    cout << " INICIANDO MAPEO DEL CONO\n";
    cout << "=========================================================\n\n";

    int orientationCount = startIndex;

    for (int i = startIndex; i < totalOrientations; i++) {
        const auto& ori = orientations[i];
        orientationCount++;
        sample_counter++;
        std::string sample_id = timestamp + "_" + std::to_string(sample_counter);

        cout << "---------------------------------------------------\n";
        cout << "[" << getTimestamp() << "]  Orientación " << orientationCount << "/" << totalOrientations
             << "  |  Inclinación: " << ori.inclination 
             << "°  |  Azimuth: " << ori.azimuth << "°\n";
        cout << "---------------------------------------------------\n";

        // Aplicar orientación al transmisor (con offset de 180° en azimuth, igual que exp_3D)
        int motorResult = gimbal.setTransmitterOrientation(ori.inclination, fmod(ori.azimuth + 180.0, 360.0));
        if (motorResult != 0) {
            cerr << "\n[ABORT] Error critico en motores (código: " << motorResult 
                 << "). Abortando experimento.\n";
            cerr << "Orientaciones completadas: " << (orientationCount - 1) << "/" << totalOrientations << "\n";
            csv_file.close();
            // Guardar estado para poder reanudar desde esta orientación
            saveState(i, csv_filename, timestamp, sample_counter - 1);
            cout << "Datos parciales guardados en: " << csv_filename << "\n";
            cout << "Estado guardado. Puede reanudar en la proxima ejecucion.\n";

            receiverFinished(sock);
            closesocket(sock);
            WSACleanup();
            cout << "Experimento abortado. Presione Enter para salir...";
            cin.get();
            return 1;
        }

        // Esperar estabilización del motor
        cout << "Estabilización (" << STABILIZATION_TIME_MS << " ms)...\n";
        Sleep(STABILIZATION_TIME_MS);

        // Adquirir datos de la DAQ
        float64* daq_data = new float64[nSamples];
        cout << "Adquiriendo " << nSamples << " muestras (" << ACQUISITION_TIME_SEC << " s)...\n";
        int read_samples = AcquireDataFromDAQ(nSamples, fSample, daq_data);

        if (read_samples > 0) {
            // Calcular mediana y media
            double median = computeMedian(daq_data, read_samples);
            double mean = computeMean(daq_data, read_samples);

            // Guardar una sola fila con mediana y media
            std::string date_str = getCurrentDate();
            std::string time_str = getCurrentTime();
            csv_file << sample_id << ","
                     << date_str << ","
                     << time_str << ","
                     << RECEIVER_X << "," 
                     << RECEIVER_Y << "," 
                     << RECEIVER_Z << "," 
                     << ori.inclination << "," 
                     << ori.azimuth << ","
                     << "r_vertical" << ","
                     << fixed << setprecision(6) << median << ","
                     << fixed << setprecision(6) << mean << "\n";
            csv_file.flush();

            cout << "OK (" << read_samples << " muestras)  ->  Mediana: " 
                 << fixed << setprecision(6) << median 
                 << "  Media: " << mean << " V\n\n";
        } else {
            cout << "ERROR en la adquisición de datos.\n\n";
            std::string date_str = getCurrentDate();
            std::string time_str = getCurrentTime();
            csv_file << sample_id << ","
                     << date_str << ","
                     << time_str << ","
                     << RECEIVER_X << "," 
                     << RECEIVER_Y << "," 
                     << RECEIVER_Z << "," 
                     << ori.inclination << "," 
                     << ori.azimuth << ","
                     << "r_vertical" << ",NA,NA\n";
            csv_file.flush();
        }

        // Liberar memoria
        delete[] daq_data;

        // Guardar estado de progreso (próxima orientación a procesar)
        saveState(i + 1, csv_filename, timestamp, sample_counter);
    }

    // =========================================================================
    // Finalización
    // =========================================================================
    csv_file.close();
    deleteState(); // Experimento completado: eliminar estado de progreso

    cout << "=========================================================\n";
    cout << " MAPEO DEL CONO COMPLETADO\n";
    cout << "=========================================================\n";
    cout << "Orientaciones registradas: " << orientationCount << "/" << totalOrientations << "\n";
    cout << "Datos guardados en: " << csv_filename << "\n\n";

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
