#include <iostream>
#include <Windows.h>
#include "stdafx.h"         // Only if you're using precompiled headers
#include "instrument.h"

using namespace std;

// Adjust according to your hardware
int motorAxisX = 27006796;  // External axis
int motorAxisY = 27007072;  // Internal axis

int main()
{
    // Optional: enable UTF-8 output in console
    system("chcp 65001 > nul");

    instrument gimbal; // Your controller instance

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
            gimbal.rotateMotor(motorAxisX, angle);
        }
        else if (command == "Y" || command == "y") {
            int angle;
            cin >> angle;
            gimbal.rotateMotor(motorAxisY, angle);
        }
        else if (command == "B" || command == "b") {
            int angleX, angleY;
            cin >> angleX >> angleY;
            gimbal.rotateMotorsSimultaneously(motorAxisX, angleX, motorAxisY, angleY);
        }
        else {
            cout << "[Error] Invalid command. Use 'X', 'Y', 'B' or 'Q'.\n";
        }

        cout << endl;
    }

    cout << "[Done] Program finished.\n";
    return 0;
}
