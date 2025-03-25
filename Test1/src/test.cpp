#include <iostream>
#include <limits>      // For numeric_limits
#include <cctype>      // For toupper
#define NOMINMAX
#include <Windows.h>
#include "stdafx.h"    // Only if you're using precompiled headers
#include "instrument.h"

//Adding libraries //****************************************************************
#include <vector>      // Para std::vector
#include <fstream>     // Para std::ifstream
#include "positions.h" // Mi propia libreria de posiciones


//  ****************************************************************
using namespace std;

const char* PATH_POSITIONS_FILE = "src/positionsToSample/test.txt";

int main()
{
    system("chcp 65001 > nul"); // Optional: enable UTF-8 output in console    

    vector<Position> positions;
    loadPositions(PATH_POSITIONS_FILE, positions);
    
    for (auto& pos : positions) {
        if (!pos.done) { 
            cout << "Position: " << pos.x << ", " << pos.y << endl;
            
        }
    }


    return 0;
}
