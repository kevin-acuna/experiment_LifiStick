#pragma once
#ifndef INSTRUMENT_H
#define INSTRUMENT_H

//#include "Ke2100.h"
//#include "IviDmm.h"
#include <string>
#include <iostream>
#include "stdafx.h"
#include <stdlib.h>
#include <conio.h>
#include <Windows.h>
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

class instrument
{
public:

    instrument();
    ~instrument();

    // Mover y "home" motores
    int rotateMotor(int serialNo, double deg);    
    int rotateMotorsSimultaneously(int serialNo1, double deg1, int serialNo2, double deg2);
    void homeMotor(int serialNo);

    // Métodos para manejo del puerto serial (LED, etc.)
    static HANDLE openSerialPort(LPCWSTR portName);   // Método estático para abrir el puerto
    static void closeSerialPort(HANDLE h_Serial);     // Método estático para cerrar el puerto

    // Encender / apagar LED en Arduino
    void turnOn(HANDLE h_Serial);
    void turnOff(HANDLE h_Serial);
    
    // NUEVAS FUNCIONES: definir posición del transmisor
    void setTransmitterPosition(double x, double y, double z); // Define la posición del transmisor
    void transmitterPointingToFloor(); // Apunta el transmisor al suelo
    void transmitterPointingToReceiver(double rx, double ry, double rz); // Apunta el transmisor al receptor
    void transmitterPointingToReceiver_simple(double rx, double ry, double rz); // Apunta el transmisor al receptor
    void transmitterPointingToReceiver_New(double rx, double ry, double rz);
    int setTransmitterOrientation(double inclination, double azimuth); // Define orientación usando ángulos de inclinación y azimuth (retorna 0 si OK, <0 si error)

    void setSerialNo_MotorX(int serialNo);
    void setSerialNo_MotorY(int serialNo);

    // Funciones para control directo de motores (útil para escaneos)
    void rotateMotorX(double deg);  // Rota solo el motor X al ángulo especificado
    void rotateMotorY(double deg);  // Rota solo el motor Y al ángulo especificado

    private:
        int serialNo_MotorX;
        int serialNo_MotorY;
        double transmitterPosX;
        double transmitterPosY;
        double transmitterPosZ;

};

#endif
