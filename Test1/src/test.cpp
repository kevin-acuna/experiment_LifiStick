#include <iostream>
#include <limits>      // For numeric_limits
#include <cctype>      // For toupper
#define NOMINMAX
#include <Windows.h>
#include "stdafx.h"    // Only if you're using precompiled headers
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

    while (true) {

        // ****************************************************************
        // Receiver Position Input
        // ****************************************************************
        cout << "************************************\n";
        cout << "POSITION: X, Y, Z\n";
        cout << "************************************\n\n";

        double receiverX, receiverY;
        cout << "Enter receiver position (X Y): ";
        cin >> receiverX >> receiverY;
        // Clean the input buffer
        cin.ignore(numeric_limits<streamsize>::max(), '\n');
        cout << "Receiver position set to: (" << receiverX << ", " << receiverY << ", " << RECEIVER_H << ")\n\n";

        // Scenario 1 - Pointing transmitter to floor and receiver to ceiling
        cout << "Scenario 1: Waiting for alignment...\n";
        gimbal.transmitterPointingToFloor();
        // receiverPointingToCeilToPosition(receiverX, receiverY);
        cout << "Scenario 1: Ready!\n";

        // Wait for a valid option (C to continue or Q to quit)
        char option;
        while (true) {
            cout << "Press C to continue or Q to quit: ";
            cin >> option;
            option = toupper(option);
            if (option == 'C' || option == 'Q')
                break;
            cout << "Invalid option. Please try again." << endl;
        }
        if (option == 'Q') {
            cout << "Program terminated by user." << endl;
            break;
        }
        cin.ignore(numeric_limits<streamsize>::max(), '\n'); // Clean the input buffer

        // Scenario 2 - Pointing transmitter to receiver
        cout << "\nScenario 2: Waiting for alignment...\n";
        gimbal.transmitterPointingToReceiver(receiverX, receiverY, RECEIVER_H);
        cout << "Scenario 2: Ready!\n";
        while (true) {
            cout << "Press C to continue or Q to quit: ";
            cin >> option;
            option = toupper(option);
            if (option == 'C' || option == 'Q')
                break;
            cout << "Invalid option. Please try again." << endl;
        }
        if (option == 'Q') {
            cout << "Program terminated by user." << endl;
            break;
        }
        cin.ignore(numeric_limits<streamsize>::max(), '\n');

        // Scenario 3 - Pointing transmitter to floor and receiver to transmitter
        cout << "\nScenario 3: Waiting for alignment...\n";
        gimbal.transmitterPointingToFloor();
        // receiverPointingToTransmitter(receiverX, receiverY);
        cout << "Scenario 3: Ready!\n";
        while (true) {
            cout << "Press C to continue or Q to quit: ";
            cin >> option;
            option = toupper(option);
            if (option == 'C' || option == 'Q')
                break;
            cout << "Invalid option. Please try again." << endl;
        }
        if (option == 'Q') {
            cout << "Program terminated by user." << endl;
            break;
        }
        cin.ignore(numeric_limits<streamsize>::max(), '\n');

        cout << endl;
    }

    cout << "[Done] Program finished.\n";
    return 0;
}
