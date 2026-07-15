#include "positions.h"
#include <fstream>
#include <iostream>
#include <iomanip>
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

void savePositions(const std::string& filePath, const std::vector<Position>& positions) {
    std::ofstream file(filePath, std::ios::trunc);
    if (!file) {
        cout << "[Warn] Could not rewrite positions file: " << filePath << endl;
        return;
    }
    file << std::setprecision(6);
    for (const auto& p : positions) {
        file << p.x << " " << p.y << " " << p.z << " " << (p.done ? 1 : 0) << "\n";
    }
    file.close();
}
