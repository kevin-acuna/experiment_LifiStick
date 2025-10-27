#include "instrument.h"
//#include "Ke2100.h"
//#include "IviDmm.h"
#include <string>
#include <iostream>
#include <Windows.h>
#include "stdafx.h"
#include <stdlib.h>
#include <conio.h>
#include "visa.h"
#include <math.h>       
#include <windows.h>
#include <thread> // Asegúrate de incluir esto para usar std::thread

#define PI 3.14159265

#if defined TestCode
#include "C:\\Program Files\\Thorlabs\\Kinesis\\Thorlabs.MotionControl.KCube.DCServo.h"
#else
#include "Thorlabs.MotionControl.KCube.DCServo.h"
#endif

#include <fstream> 
#include <cmath>
#include <iomanip>

using namespace std;

// ----------------------------------------------------------
// CONSTRUCTOR / DESTRUCTOR
// ----------------------------------------------------------
instrument::instrument()
{
    transmitterPosX = 0;
    transmitterPosY = 0;
    transmitterPosZ = 0;
}

instrument::~instrument()
{
    cout << "***************************" << endl;
}


// ----------------------------------------------------------
// Rotate motor to a specific angle (in degrees)
// ----------------------------------------------------------
int instrument::rotateMotor(int serialNo, double deg)
{
    const double COUNTS_PER_DEGREE = 1919.5;     // For PRM1Z8 + KDC101
    const int MIN_ALLOWED_DEG = -90;
    const int MAX_ALLOWED_DEG = 90;
    const double ZERO_REFERENCE_DEG = 0.0;
    const int DEFAULT_VELOCITY = 1800000;          // Device units per second

    // Identifiers for axes with calibration offsets
    // Sistema Gimbal 1 (antiguo)
    const int MOTOR_AXIS_X_CENTER = 27006796;  // External axis
    const int MOTOR_AXIS_Y_CENTER = 27007072;  // Internal axis
    
    // Sistema Gimbal 2 (nuevo)
    const int MOTOR_AXIS_X_OWP = 27267164;  // External axis
    const int MOTOR_AXIS_Y_OWP = 27602122;  // Internal axis
    
    // Offsets de calibración (misma lógica para ambos sistemas)
    const double OFFSET_MOTOR_X_CENTER = +2.0; // Offset calibration for motor X - External
    const double OFFSET_MOTOR_Y_CENTER = -2.0; // Offset calibration for motor Y - Internal

    const double OFFSET_MOTOR_X_OWP = 1.0; // Offset calibration for motor X - External
    const double OFFSET_MOTOR_Y_OWP = -1.0; // Offset calibration for motor Y - Internal

    // Apply calibration offset based on motor
    double calibratedDeg = deg;
    std::string axisLabel = "Unknown Axis";
    
    if (serialNo == MOTOR_AXIS_X_CENTER) {
        axisLabel = "Axis X";
        calibratedDeg = deg + OFFSET_MOTOR_X_CENTER;
    }
    else if (serialNo == MOTOR_AXIS_Y_CENTER) {
        axisLabel = "Axis Y";
        calibratedDeg = deg + OFFSET_MOTOR_Y_CENTER;
    }
    else if (serialNo == MOTOR_AXIS_X_OWP) {
        axisLabel = "Axis X";
        calibratedDeg = deg + OFFSET_MOTOR_X_OWP;
    }
    else if (serialNo == MOTOR_AXIS_Y_OWP) {
        axisLabel = "Axis Y";
        calibratedDeg = deg + OFFSET_MOTOR_Y_OWP;
    }
    
    char testSerialNo[16];
    sprintf_s(testSerialNo, "%d", serialNo);

    // Clamp the angle to the allowed range
    if (deg < MIN_ALLOWED_DEG || deg > MAX_ALLOWED_DEG)
    {
        cerr << "[Warning] " << axisLabel << ": Requested angle " << deg
             << "° is out of bounds. Allowed range: [-60°, 60°]. Rotation aborted.\n";
        return -3;
    }

    // Build device list
    if (TLI_BuildDeviceList() != 0)
    {
        cerr << "[Error] " << axisLabel << ": Failed to build device list.\n";
        return -1;
    }

    // Open device
    if (CC_Open(testSerialNo) != 0)
    {
        cerr << "[Error] " << axisLabel << ": Failed to open device with serial: " << testSerialNo << endl;
        return -2;
    }

    // Start polling
    CC_StartPolling(testSerialNo, 200);
    Sleep(300); // Allow time to stabilize

    // Set unlimited rotation and quickest path
    CC_SetRotationModes(testSerialNo, RotationalUnlimited, Quickest);
    CC_ClearMessageQueue(testSerialNo);

    // Convert angle to device units (CCW = positive) using calibrated angle
    int devicePosition = static_cast<int>((ZERO_REFERENCE_DEG - calibratedDeg) * COUNTS_PER_DEGREE);

    // Set constant velocity
    int currentVelocity = 0;
    int currentAcceleration = 0;
    CC_GetVelParams(testSerialNo, &currentAcceleration, &currentVelocity);
    CC_SetVelParams(testSerialNo, currentAcceleration, DEFAULT_VELOCITY);
    //printf("[Info] %s: Velocity set to %d device units.\n", axisLabel.c_str(), DEFAULT_VELOCITY);

    // Rotate to position
    printf("[Info] %s: Rotating to %.1f° (calibrated: %.1f°, %d device units)...\n", axisLabel.c_str(), deg, calibratedDeg, devicePosition);
    CC_MoveToPosition(testSerialNo, devicePosition);

    // Wait for movement to complete
    WORD messageType;
    WORD messageId;
    DWORD messageData;
    do {
        CC_WaitForMessage(testSerialNo, &messageType, &messageId, &messageData);
    } while (messageType != 2 || messageId != 1);

    Sleep(500); // Small buffer

    // Report final position
    int finalPosition = CC_GetPosition(testSerialNo);
    double finalAngle = static_cast<double>(finalPosition) / COUNTS_PER_DEGREE;
    printf("[Info] %s: Rotation complete. Final position: %d device units (%.2f°)\n", axisLabel.c_str(), finalPosition, finalAngle);

    return 0;
}

// ----------------------------------------------------------
// Rotate two motors simultaneously to specific angles
// ----------------------------------------------------------
int instrument::rotateMotorsSimultaneously(int serialNo1, double deg1, int serialNo2, double deg2)
{
    // Define lambda that calls rotateMotor
    auto rotate = [this](int serial, double deg) {
        this->rotateMotor(serial, deg);
    };

    // Create threads
    std::thread motorThread1(rotate, serialNo1, deg1);
    std::thread motorThread2(rotate, serialNo2, deg2);

    // Wait for both to finish
    motorThread1.join();
    motorThread2.join();

    // Optional: you could return error codes from each if needed in the future
    return 0;
}


// ----------------------------------------------------------
// Home del motor
// ----------------------------------------------------------
void instrument::homeMotor(int serialNo)
{
    char testSerialNo[16];
    sprintf_s(testSerialNo, "%d", serialNo);

    if (TLI_BuildDeviceList() == 0)
    {
        // open device
        if (CC_Open(testSerialNo) == 0)
        {
            // start the device polling
            CC_StartPolling(testSerialNo, 200);
            CC_SetRotationModes(testSerialNo, RotationalUnlimited, Quickest);
            Sleep(3000);

            // Home device
            CC_ClearMessageQueue(testSerialNo);
            CC_Home(testSerialNo);
            printf("Device %s homing\n", testSerialNo);

            // wait for completion
            WORD messageType;
            WORD messageId;
            DWORD messageData;

            CC_WaitForMessage(testSerialNo, &messageType, &messageId, &messageData);
            while (messageType != 2 || messageId != 0)
            {
                CC_WaitForMessage(testSerialNo, &messageType, &messageId, &messageData);
            }
        }
    }
}

// ----------------------------------------------------------
// Multímetro
// ----------------------------------------------------------
/*void instrument::initializeMultimeter() 
{
    ViChar name[36] = "USB::0x05E6::0x2100::8018542::INSTR";
    ViStatus status = Ke2100_init(name, VI_TRUE, VI_TRUE, &multimeterSession);

    // Configuraciones del Ke2100
    status = Ke2100_ConfigureAutoZeroMode(multimeterSession, KE2100_VAL_AUTO_ZERO_ON);
    status = Ke2100_SetAttributeViInt32(multimeterSession, "", KE2100_ATTR_AUTO_GAIN, KE2100_VAL_AUTO_GAIN_ON);
    status = Ke2100_GetRange2(multimeterSession, KE2100_VAL_CONFIGURATION_FUNCTION2DC_VOLTS, &multimeterRange);
    status = Ke2100_GetResolution(multimeterSession, KE2100_VAL_CONFIGURATION_FUNCTION2DC_VOLTS, &multimeterResolution);
    status = Ke2100_Configure(multimeterSession, KE2100_VAL_FUNCTIONDC_VOLTS, multimeterRange, multimeterResolution);
}

double instrument::measureVoltage() 
{
    ViReal64 volt;
    ViStatus status;
    int attempts = 0;
    const int maxAttempts = 3;

    while (attempts < maxAttempts)
    {
        status = Ke2100_Measure(multimeterSession,
                                KE2100_VAL_FUNCTIONDC_VOLTS,
                                multimeterRange,
                                multimeterResolution,
                                &volt);

        // SOLO imprimimos "LEIDO"
        cout << "LEIDO: " << -volt << endl;

        if (volt >= -16 && volt <= 16) {
            return -volt; // retornamos volt (positivo)
        }

        attempts++;
        Sleep(200);
    }

    cerr << "Error: No se pudo obtener una medicion valida tras "
         << maxAttempts << " intentos." << endl;
    return std::numeric_limits<double>::quiet_NaN();
}

void instrument::closeMultimeter() 
{
    ViStatus status = Ke2100_close(multimeterSession);
    // (Opcional) Manejo de error si status != VI_SUCCESS
}
*/

// ----------------------------------------------------------------------
// Abrir puerto serial
// ----------------------------------------------------------------------
HANDLE instrument::openSerialPort(LPCWSTR portName) {
    HANDLE h_Serial = CreateFile(
        portName, 
        GENERIC_READ | GENERIC_WRITE, 
        0, 
        NULL, 
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL, 
        NULL
    );

    if (h_Serial == INVALID_HANDLE_VALUE) {
        wprintf(L"[Error] No se pudo abrir el puerto: %s. Error: %lu\n", portName, GetLastError());
        return INVALID_HANDLE_VALUE;
    }

    DCB dcbSerialParams = { 0 };
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    if (!GetCommState(h_Serial, &dcbSerialParams)) {
        wprintf(L"[Error] GetCommState fallo. Error: %lu\n", GetLastError());
        CloseHandle(h_Serial);
        return INVALID_HANDLE_VALUE;
    }

    // Ajustar parámetros del puerto
    dcbSerialParams.BaudRate = CBR_115200; // Ajusta al baudrate de tu Arduino
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity   = NOPARITY;

    if (!SetCommState(h_Serial, &dcbSerialParams)) {
        wprintf(L"[Error] SetCommState fallo. Error: %lu\n", GetLastError());
        CloseHandle(h_Serial);
        return INVALID_HANDLE_VALUE;
    }

    // Tiempos de espera
    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout         = 50;
    timeouts.ReadTotalTimeoutConstant    = 50;
    timeouts.ReadTotalTimeoutMultiplier  = 10;
    timeouts.WriteTotalTimeoutConstant   = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;

    if (!SetCommTimeouts(h_Serial, &timeouts)) {
        wprintf(L"[Error] SetCommTimeouts fallo. Error: %lu\n", GetLastError());
        CloseHandle(h_Serial);
        return INVALID_HANDLE_VALUE;
    }

    return h_Serial;
}

// ----------------------------------------------------------------------
// Cerrar puerto serial
// ----------------------------------------------------------------------
void instrument::closeSerialPort(HANDLE h_Serial) {
    CloseHandle(h_Serial);
}


// ----------------------------------------------------------------------
// Enciende LED en Arduino (envía caracter '1')
// ----------------------------------------------------------------------
void instrument::turnOn(HANDLE h_Serial) {
    const char cmdOn = '1';  
    DWORD bytesWritten = 0;
    if (!WriteFile(h_Serial, &cmdOn, 1, &bytesWritten, NULL)) {
        cerr << "[Error] No se pudo enviar comando ON al Arduino." << endl;
    }
}

// ----------------------------------------------------------------------
// Apaga LED en Arduino (envía caracter '0')
// ----------------------------------------------------------------------
void instrument::turnOff(HANDLE h_Serial) {
    const char cmdOff = '0';
    DWORD bytesWritten = 0;
    if (!WriteFile(h_Serial, &cmdOff, 1, &bytesWritten, NULL)) {
        cerr << "[Error] No se pudo enviar comando OFF al Arduino." << endl;
    }
}


// ******************************************************************************
// Nuevas funciones 
// ******************************************************************************
void instrument::setTransmitterPosition(double x, double y, double z)
{
    transmitterPosX = x;
    transmitterPosY = y;
    transmitterPosZ = z;
}

void instrument::setSerialNo_MotorX(int serialNo)
{
    serialNo_MotorX = serialNo;
}

void instrument::setSerialNo_MotorY(int serialNo)
{
    serialNo_MotorY = serialNo;
}

void instrument::transmitterPointingToFloor()
{
    rotateMotorsSimultaneously(serialNo_MotorX, 0, serialNo_MotorY, 0);
}


void instrument::transmitterPointingToReceiver_simple(double rx, double ry, double rz)
{
    // Vector desde transmisor al receptor
    double dx = rx - transmitterPosX;
    double dy = ry - transmitterPosY;
    double dz = rz - transmitterPosZ;

    // Vector unitario hacia el receptor
    double norm = sqrt(dx*dx + dy*dy + dz*dz);
    double vx = dx / norm;
    double vy = dy / norm;
    double vz = dz / norm;

    // Cálculo correcto basado en matriz de rotación
    double angleX_rad = asin(vy);                        // sin(theta_x) = vy
    double angleY_rad = atan2(-vx, -vz);                 // tan(theta_y) = -vx / -vz

    // Conversión a grados
    int angleX_deg = static_cast<int>(round(angleX_rad * 180.0 / PI));
    int angleY_deg = static_cast<int>(round(angleY_rad * 180.0 / PI));

    // Rotar motores en orden adecuado
    rotateMotorsSimultaneously(serialNo_MotorX, angleX_deg, serialNo_MotorY, angleY_deg);
}


void instrument::transmitterPointingToReceiver(double rx, double ry, double rz)
{
    // Diferencias respecto a la posición del transmisor
    double dx = rx - transmitterPosX; // x_R
    double dy = ry - transmitterPosY; // y_R
    // Suponemos que el transmisor está por encima: transmitterPosZ > rz.
    double delta = transmitterPosZ - rz; // Δ = H - h

    // Calcular el ángulo de rotación en el eje Y (motor interno)
    // theta_y = -arctan(x_R / (H - h))
    double angleY_rad = -atan2(dx, delta);

    // Calcular el ángulo de rotación en el eje X (motor externo)
    // theta_x = arctan((y_R * cos(theta_y)) / (H - h))
    double angleX_rad = atan2(dy * cos(angleY_rad), delta);

    // Conversión a grados con 1 decimal
    // Multiplicamos por 10, redondeamos, y dividimos por 10
    double angleX_deg = std::round((angleX_rad * 180.0 / PI) * 10.0) / 10.0;
    double angleY_deg = std::round((angleY_rad * 180.0 / PI) * 10.0) / 10.0;

    // Se recomienda rotar primero el motor del eje Y y luego el de X (o enviarlos de forma sincronizada si el controlador lo permite)
    rotateMotorsSimultaneously(serialNo_MotorX, angleX_deg, serialNo_MotorY, angleY_deg);
}


void instrument::transmitterPointingToReceiver_New(double rx, double ry, double rz)
{
    // Diferencias respecto a la posición del transmisor
    double dx = rx - transmitterPosX; // x_R
    double dy = ry - transmitterPosY; // y_R
    // Suponemos que el transmisor está por encima: transmitterPosZ > rz.
    double delta = transmitterPosZ - rz; // Δ = H - h

    // Paso 1: Calcular el ángulo en el eje X (motor externo)
    // theta_x = arctan(y_R / (H - h))
    double angleX_rad = atan2(dy, delta);

    // Paso 2: Calcular el ángulo en el eje Y (motor interno)
    // theta_y = -arctan((cos(theta_x) * x_R) / (H - h))
    double angleY_rad = -atan2(cos(angleX_rad) * dx, delta);

    // Conversión a grados con 1 decimal
    double angleX_deg = std::round((angleX_rad * 180.0 / PI) * 10.0) / 10.0;
    double angleY_deg = std::round((angleY_rad * 180.0 / PI) * 10.0) / 10.0;

    // Se recomienda rotar primero el motor del eje X y luego el de Y, 
    // o enviarlos de forma sincronizada si el controlador lo permite.
    rotateMotorsSimultaneously(serialNo_MotorX, angleX_deg, serialNo_MotorY, angleY_deg);
}


// ----------------------------------------------------------------------
// Define orientación del transmisor usando ángulos de inclinación y azimuth
// inclination: ángulo entre el vector de orientación y el eje vertical (0-180 grados)
// azimuth: ángulo en el plano XY medido desde el eje X (0-360 grados)
// ----------------------------------------------------------------------
void instrument::setTransmitterOrientation(double inclination, double azimuth)
{
    // Convertir ángulos de grados a radianes
    double inclination_rad = inclination * PI / 180.0;
    double azimuth_rad = azimuth * PI / 180.0;
    
    // Convertir coordenadas esféricas a un vector de dirección
    // Para inclination=0 y azimuth=0, el vector debe ser (0,0,-1)
    // Por eso el sin/cos de inclination están invertidos respecto a las fórmulas tradicionales
    double dirX = sin(inclination_rad) * cos(azimuth_rad);
    double dirY = sin(inclination_rad) * sin(azimuth_rad);
    double dirZ = -cos(inclination_rad);  // Negativo para que apunte hacia abajo cuando inclination=0
    
    // Paso 1: Calcular el ángulo en el eje X (motor externo)
    // theta_x = arctan(y_R / (H - h))
    double angleX_rad = atan2(dirY, -dirZ);

    // Paso 2: Calcular el ángulo en el eje Y (motor interno)
    // theta_y = -arctan((cos(theta_x) * x_R) / (H - h))
    double angleY_rad = -atan2(cos(angleX_rad) * dirX, -dirZ);

    // Conversión a grados con 1 decimal
    double angleX_deg = std::round((angleX_rad * 180.0 / PI) * 10.0) / 10.0;
    double angleY_deg = std::round((angleY_rad * 180.0 / PI) * 10.0) / 10.0;

    // Se recomienda rotar primero el motor del eje X y luego el de Y, 
    // o enviarlos de forma sincronizada si el controlador lo permite.
    rotateMotorsSimultaneously(serialNo_MotorX, angleX_deg, serialNo_MotorY, angleY_deg);
}
// ----------------------------------------------------------------------
// Control directo de motor X
// ----------------------------------------------------------------------
void instrument::rotateMotorX(double deg)
{
    rotateMotor(serialNo_MotorX, deg);
}

// ----------------------------------------------------------------------
// Control directo de motor Y
// ----------------------------------------------------------------------
void instrument::rotateMotorY(double deg)
{
    rotateMotor(serialNo_MotorY, deg);
}
