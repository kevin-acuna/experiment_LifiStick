#ifndef POSITIONS_H
#define POSITIONS_H

#include <vector>
#include <string>

// Definición de la estructura Position
struct Position {
    double x;
    double y;
    double z;   // Altura del receptor (RECEIVER_H)
    bool done;

    // Constructor: permite inicializar x, y, z y, opcionalmente, done (por defecto false)
    Position(double x, double y, double z, bool done = false) : x(x), y(y), z(z), done(done) {}
};

// Declaración de la función para cargar posiciones desde un archivo
// Recibe la ruta del archivo y el vector donde se almacenarán las posiciones.
// El archivo debe contener: x y [z] [done]
// z y done son opcionales (valores por defecto: z=0.96, done=false)
void loadPositions(const std::string& filePath, std::vector<Position>& positions);

#endif // POSITIONS_H
