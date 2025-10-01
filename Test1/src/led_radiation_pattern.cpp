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
#define COM_PORT_NAME "COM4"

// Configuración del experimento de patrón de irradiación
const int STABILIZATION_TIME_MS = 2000;       // Tiempo de estabilización después de mover el motor
const int ACQUISITION_TIME_SEC = 10;          // Tiempo de adquisición por cada step (10 segundos)

// Rango de escaneo del transmisor (en grados)
const double ANGLE_START = -60.0;             // Ángulo inicial
const double ANGLE_END = 60.0;                // Ángulo final
const double ANGLE_STEP = 10.0;                // Step de escaneo en grados

// Eje a escanear: 'X' o 'Y'
const char SCAN_AXIS = 'X';                   // Cambiar a 'Y' para escanear el otro eje

// Configuración para la adquisición de DAQ
int32 fSample = 1000;                         // Frecuencia de muestreo: 1000 Hz
int32 nSamples = ACQUISITION_TIME_SEC * fSample;  // Número de muestras por adquisición

// Identificadores de motores
int MOTOR_AXIS_X = 27267164;  // External axis
int MOTOR_AXIS_Y = 27602122;  // Internal axis

// Altura del transmisor
const double TRANSMITTER_H = 2.00; // altitude in meters

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
            "Dev1/ai1",       // Canal de entrada
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
// Función principal del experimento
// *****************************************************************************
int main()
{
    system("chcp 65001 > nul"); // Habilitar UTF-8 en consola
    
    cout << "=========================================================\n";
    cout << " EXPERIMENTO: Patrón de Irradiación del LED\n";
    cout << "=========================================================\n";
    cout << "Configuración del experimento:\n";
    cout << "  - Eje a escanear: " << SCAN_AXIS << "\n";
    cout << "  - Rango: " << ANGLE_START << "° a " << ANGLE_END << "°\n";
    cout << "  - Step: " << ANGLE_STEP << "°\n";
    cout << "  - Tiempo de adquisición por step: " << ACQUISITION_TIME_SEC << " segundos\n";
    cout << "=========================================================\n\n";

    // Calcular número de pasos
    int numSteps = static_cast<int>((ANGLE_END - ANGLE_START) / ANGLE_STEP) + 1;
    cout << "Total de pasos a realizar: " << numSteps << "\n\n";


    // Configurar gimbal (control de motores)
    instrument gimbal;
    gimbal.setSerialNo_MotorX(MOTOR_AXIS_X);
    gimbal.setSerialNo_MotorY(MOTOR_AXIS_Y);
    gimbal.setTransmitterPosition(0, 0, TRANSMITTER_H);

    // Mover ambos motores a la posición inicial (0°, 0°)
    cout << "\nMoviendo motores a posición inicial (0°, 0°)...\n";
    cout << "NOTA: Los offsets de calibración se aplican automáticamente.\n";
    gimbal.rotateMotorsSimultaneously(MOTOR_AXIS_X, 0.0, MOTOR_AXIS_Y, 0.0);
    cout << "Motores en posición inicial.\n";
    Sleep(2000); // Esperar estabilización

    // Confirmación del usuario
    cout << "¿Desea continuar? (C para continuar, Q para salir): ";
    char option;
    cin >> option;
    option = toupper(option);
    if (option == 'Q') {
        cout << "Experimento cancelado por el usuario.\n";
        return 0;
    }
    cin.ignore(numeric_limits<streamsize>::max(), '\n');

    // -- Voy a asumir que el LED esta encendido (previa verificacion) --
    /*
    // Abrir puerto serial para control del LED
    cout << "\nAbriendo puerto serial " << COM_PORT_NAME << " para control del LED...\n";
    HANDLE serialPort = instrument::openSerialPort(L"COM4");
    if (serialPort == INVALID_HANDLE_VALUE) {
        cout << "Error: No se pudo abrir el puerto serial. Abortando.\n";
        return -1;
    }
    cout << "Puerto serial abierto exitosamente.\n";

    // Encender LED
    cout << "Encendiendo LED...\n";
    gimbal.turnOn(serialPort);
    cout << "Esperando estabilización del LED (10 segundos)...\n";
    Sleep(10000); // Esperar 10 segundos para estabilización inicial del LED
    */

    // Crear archivo CSV para guardar los datos
    string csv_filename = "radiation_pattern_axis_" + string(1, SCAN_AXIS) + ".csv";
    ofstream csv_file(csv_filename);
    if (!csv_file.is_open()) {
        cerr << "Error: No se pudo crear el archivo CSV: " << csv_filename << endl;
        gimbal.closeSerialPort(serialPort);
        return -1;
    }

    // Escribir encabezado del CSV
    csv_file << "eje,angulo_grados,voltaje" << endl;
    cout << "Archivo CSV creado: " << csv_filename << "\n\n";

    // Iniciar el escaneo
    cout << "=========================================================\n";
    cout << " INICIANDO ESCANEO\n";
    cout << "=========================================================\n\n";

    double currentAngle = ANGLE_START;
    int stepCount = 0;

    while (currentAngle <= ANGLE_END) {
        stepCount++;
        cout << "---------------------------------------------------\n";
        cout << "Step " << stepCount << "/" << numSteps 
             << " - Eje " << SCAN_AXIS 
             << " - Ángulo: " << currentAngle << "°\n";
        cout << "---------------------------------------------------\n";

        // Mover el motor del eje correspondiente
        if (SCAN_AXIS == 'X') {
            cout << "Moviendo motor X a " << currentAngle << "°...\n";
            gimbal.rotateMotorX(currentAngle);
        } else if (SCAN_AXIS == 'Y') {
            cout << "Moviendo motor Y a " << currentAngle << "°...\n";
            gimbal.rotateMotorY(currentAngle);
        } else {
            cerr << "Error: Eje inválido. Use 'X' o 'Y'.\n";
            break;
        }

        // Esperar estabilización del motor
        cout << "Esperando estabilización (" << STABILIZATION_TIME_MS << " ms)...\n";
        Sleep(STABILIZATION_TIME_MS);

        // Adquirir datos de la DAQ
        cout << "Adquiriendo datos de la DAQ (" << ACQUISITION_TIME_SEC << " segundos)...\n";
        float64* daq_data = new float64[nSamples];
        int read_samples = AcquireDataFromDAQ(nSamples, fSample, daq_data);

        if (read_samples > 0) {
            cout << "Adquisición exitosa: " << read_samples << " muestras adquiridas.\n";
            
            // Guardar cada muestra en el CSV
            for (int i = 0; i < read_samples; i++) {
                csv_file << SCAN_AXIS << "," 
                         << currentAngle << "," 
                         << daq_data[i] << endl;
            }
        } else {
            cout << "Error en la adquisición de datos.\n";
            // Guardar línea con NA
            csv_file << SCAN_AXIS << "," 
                     << currentAngle << ",NA" << endl;
        }

        // Liberar memoria
        delete[] daq_data;

        cout << "Step completado.\n\n";

        // Avanzar al siguiente ángulo
        currentAngle += ANGLE_STEP;
        
        // Pequeña pausa entre steps
        Sleep(500);
    }

    // Finalizar experimento
    cout << "=========================================================\n";
    cout << " ESCANEO COMPLETADO\n";
    cout << "=========================================================\n";
    cout << "Total de pasos realizados: " << stepCount << "\n";
    cout << "Datos guardados en: " << csv_filename << "\n\n";

    // Apagar LED
    cout << "Apagando LED...\n";
    gimbal.turnOff(serialPort);

    // Cerrar archivo CSV
    csv_file.close();

    // Cerrar puerto serial
    cout << "Cerrando puerto serial...\n";
    gimbal.closeSerialPort(serialPort);

    cout << "\nExperimento finalizado exitosamente.\n";
    cout << "Presione Enter para salir...";
    cin.get();

    return 0;
}
