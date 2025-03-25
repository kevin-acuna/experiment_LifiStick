#ifndef POSITIONS_H
#define POSITIONS_H

#include <vector>
#include <string>

// Definición de la estructura Position
struct Position {
    float x;
    float y;
    bool done;

    // Constructor: permite inicializar x, y y, opcionalmente, done (por defecto false)
    Position(float x, float y, bool done = false) : x(x), y(y), done(done) {}
};

// Declaración de la función para cargar posiciones desde un archivo
// Recibe la ruta del archivo y el vector donde se almacenarán las posiciones.
void loadPositions(const std::string& filePath, std::vector<Position>& positions);

#endif // POSITIONS_H
