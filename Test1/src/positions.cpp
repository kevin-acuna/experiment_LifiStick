#include "positions.h"
#include <fstream>
#include <iostream>
using namespace std;

void loadPositions(const std::string& filePath, std::vector<Position>& positions) {
    std::ifstream file(filePath);
    if (!file) {
        // Si el archivo no se abre correctamente, podrías manejar el error aquí
        return;
    }
    double x, y, z;
    bool done;
    while (file >> x >> y >> z >> done) {
        positions.push_back(Position(x, y, z, done));
        } 
    file.close();
    cout << "[Info] Sampling points loaded successfully" << endl;
}
