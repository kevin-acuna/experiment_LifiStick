/*

#include <iostream>
#include <Windows.h>
#include "stdafx.h"         
#include "instrument.h"

using namespace std;

// Adjust according to your hardware
int MOTOR_AXIS_X = 27006796;  // External axis
int MOTOR_AXIS_Y = 27007072;  // Internal axis

const double TRANSMITTER_H = 1.98; // altitude in meters
const double RECEIVER_H = 0.96;    // altitude in meters

int main()
{
    // Optional: enable UTF-8 output in console
    system("chcp 65001 > nul");

    instrument gimbal; // Your controller instance
    gimbal.setSerialNo_MotorX(MOTOR_AXIS_X);
    gimbal.setSerialNo_MotorY(MOTOR_AXIS_Y);
    gimbal.setTransmitterPosition(0, 0, TRANSMITTER_H);
    string command;

    while (true) {

    cout << "========================================\n";
    cout << "   GIMBAL MOTOR TESTING TOOL (2 AXES)   \n";
    cout << "========================================\n\n";

    cout << "Usage examples:\n"
         << "  - Enter 'X 30'   -> rotate X axis to +30 degrees\n"
         << "  - Enter 'Y -45'  -> rotate Y axis to -45 degrees\n"
         << "  - Enter 'B 30 -30' -> rotate both axes simultaneously\n"
         << "  - Enter 'Q' to quit\n\n";

    while (true) {
        cout << "[Input] Enter command: ";

        string command;
        cin >> command;

        if (command == "Q" || command == "q") {
            cout << "[Exit] Quitting program.\n";
            break;
        }
        else if (command == "X" || command == "x") {
            int angle;
            cin >> angle;
            gimbal.rotateMotor(MOTOR_AXIS_X, angle);
        }
        else if (command == "Y" || command == "y") {
            int angle;
            cin >> angle;
            gimbal.rotateMotor(MOTOR_AXIS_Y, angle);
        }
        else if (command == "B" || command == "b") {
            int angleX, angleY;
            cin >> angleX >> angleY;
            gimbal.rotateMotorsSimultaneously(MOTOR_AXIS_X, angleX, MOTOR_AXIS_Y, angleY);
        }
        else {
            cout << "[Error] Invalid command. Use 'X', 'Y', 'B' or 'Q'.\n";
        }
    }

    }

cout << "[Done] Program finished.\n";
return 0;
}



*/