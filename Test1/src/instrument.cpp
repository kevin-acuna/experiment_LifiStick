#include "instrument.h"
#include "Ke2100.h"
#include "IviDmm.h"
#include <string>
#include <iostream>
#include <Windows.h>
#include "stdafx.h"
#include <stdlib.h>
#include <conio.h>
#include "visa.h"
#include <math.h>       
#include <windows.h>

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
// Mover motor a cierto ángulo (mote)
// ----------------------------------------------------------
int instrument::moveMotor(int deg, int serialNo, int velocity)
{
    char testSerialNo[16];
    sprintf_s(testSerialNo, "%d", serialNo);

    // build device list
    if (TLI_BuildDeviceList() != 0)
    {
        // No se pudo construir la lista de dispositivos, retornamos un código de error
        cerr << "[Error] No se pudo construir la lista de dispositivos.\n";
        return -1;
    }

    // open device
    if (CC_Open(testSerialNo) != 0)
    {
        // No se pudo abrir el dispositivo con el serialNo
        cerr << "[Error] No se pudo abrir el dispositivo con serial: " << testSerialNo << endl;
        return -2;
    }

    // Iniciamos el polling
    CC_StartPolling(testSerialNo, 200);

    // Rotación sin límite, modo “Quickest”
    CC_SetRotationModes(testSerialNo, RotationalUnlimited, Quickest);
    Sleep(300);

    // Variables para esperar mensajes
    WORD messageType;
    WORD messageId;
    DWORD messageData;

    // *** IMPORTANTE: Evitar conversión implícita (warning C4244) ***
    int pose = static_cast<int>((90 - deg) * 1919.5);

    // set velocity if desired
    if (velocity > 0)
    {
        int currentVelocity = 0;
        int currentAcceleration = 0;
        CC_GetVelParams(testSerialNo, &currentAcceleration, &currentVelocity);
        CC_SetVelParams(testSerialNo, currentAcceleration, velocity);
    }

    // Mover al ángulo
    CC_ClearMessageQueue(testSerialNo);
    CC_MoveToPosition(testSerialNo, pose);
    printf("Device %s moving to %d deg\n", testSerialNo, deg);

    // Esperar hasta que se complete el movimiento
    CC_WaitForMessage(testSerialNo, &messageType, &messageId, &messageData);
    while (messageType != 2 || messageId != 1)
    {
        CC_WaitForMessage(testSerialNo, &messageType, &messageId, &messageData);
    }
    Sleep(500);

    // Llegó aquí correctamente
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
            Sleep(500);

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
void instrument::initializeMultimeter() 
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



void instrument::setZeroOffsets(int offsetX, int offsetY) {
    zeroOffsetX = offsetX;
    zeroOffsetY = offsetY;
}

void instrument::rotateAxisX(int angle, int serialNo, int velocity) {
    moveMotor(zeroOffsetX + angle, serialNo, velocity);
}

void instrument::rotateAxisY(int angle, int serialNo, int velocity) {
    moveMotor(zeroOffsetY + angle, serialNo, velocity);
}
