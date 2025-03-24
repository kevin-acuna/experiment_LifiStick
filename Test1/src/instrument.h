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

    double x, y, he, hr, der;
	
    // Para el multímetro
    //ViSession multimeterSession;
    //ViReal64 multimeterRange;
    //ViReal64 multimeterResolution;

    instrument();
    ~instrument();

    void set(double fond, double posx, double posy);
    double get();
    
    // Mover y "home" motores
    int rotateMotor(int serialNo, int deg);    
    int rotateMotorsSimultaneously(int serialNo1, int deg1, int serialNo2, int deg2);
    void homeMotor(int serialNo);

    // Métodos para manejo del puerto serial (LED, etc.)
    static HANDLE openSerialPort(LPCWSTR portName);   // Método estático para abrir el puerto
    static void closeSerialPort(HANDLE h_Serial);     // Método estático para cerrar el puerto

    // NUEVAS FUNCIONES: encender / apagar LED en Arduino
    void turnOn(HANDLE h_Serial);
    void turnOff(HANDLE h_Serial);
    
    // Métodos para manejo del multímetro
    //void initializeMultimeter();
    //double measureVoltage();
    //void closeMultimeter();

    private:
        double fond;
};

#endif
