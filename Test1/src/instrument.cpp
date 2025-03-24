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
    fond = 0;
    x    = 0;
    y    = 0;
    hr   = 0;
    he   = 0;
    der  = 0;
}

instrument::~instrument()
{
    fond = 0;
}

// ----------------------------------------------------------
// MÉTODOS DE INSTRUMENT
// ----------------------------------------------------------
void instrument::set(double fondVal, double posx, double posy)
{
    fond = fondVal;  // almacenar si lo deseas
    x = posx;
    y = posy;
    cout << "************************************************************************* " << endl;
}

double instrument::get()
{
    return fond;
}


// ----------------------------------------------------------
// Rotate motor to a specific angle (in degrees)
// ----------------------------------------------------------
int instrument::rotateMotor(int serialNo, int deg)
{
    const double COUNTS_PER_DEGREE = 1919.5;     // For PRM1Z8 + KDC101
    const int MIN_ALLOWED_DEG = -60;
    const int MAX_ALLOWED_DEG = 60;
    const double ZERO_REFERENCE_DEG = 0.0;
    const int DEFAULT_VELOCITY = 1800000;          // Device units per second

    // Identifiers for axes (should match main())
    const int MOTOR_AXIS_X = 27006796;
    const int MOTOR_AXIS_Y = 27007072;

    // Determine axis label for console output
    std::string axisLabel = "Unknown Axis";
    if (serialNo == MOTOR_AXIS_X)
        axisLabel = "Axis X";
    else if (serialNo == MOTOR_AXIS_Y)
        axisLabel = "Axis Y";

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

    // Convert angle to device units (CCW = positive)
    int devicePosition = static_cast<int>((ZERO_REFERENCE_DEG - deg) * COUNTS_PER_DEGREE);

    // Set constant velocity
    int currentVelocity = 0;
    int currentAcceleration = 0;
    CC_GetVelParams(testSerialNo, &currentAcceleration, &currentVelocity);
    CC_SetVelParams(testSerialNo, currentAcceleration, DEFAULT_VELOCITY);
    //printf("[Info] %s: Velocity set to %d device units.\n", axisLabel.c_str(), DEFAULT_VELOCITY);

    // Rotate to position
    printf("[Info] %s: Rotating to %.1f° (%d device units)...\n", axisLabel.c_str(), static_cast<double>(deg), devicePosition);
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
int instrument::rotateMotorsSimultaneously(int serialNo1, int deg1, int serialNo2, int deg2)
{
    // Define lambda that calls rotateMotor
    auto rotate = [this](int serial, int deg) {
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
