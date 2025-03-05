#include <iostream>
#include <Windows.h>
#include "stdafx.h"         // si lo usas para precompiled headers
#include "instrument.h"

using namespace std;

// Ajusta según tu hardware:
int axe1     = 27267164;   // Serial del motor "X" (eje externo)
int axe2     = 27602122;   // Serial del motor "Y" (eje interno)
int velocity = 2000000;    // Velocidad de movimiento

int main()
{
    // Para mostrar caracteres acentuados en consola (opcional):
    system("chcp 65001 > nul");

    instrument gimbal;  // Instancia de la clase que gestiona los motores (y otras cosas)

    cout << "========================================\n";
    cout << "  TEST DE MOTORES GIMBAL (2 EJES)       \n";
    cout << "========================================\n\n";

    // ---------------------------------------------------------
    // 1) Opción de hacer "homing" si se desea
    // ---------------------------------------------------------
    cout << "¿Deseas hacer homing en ambos motores? (S/N): ";
    char ans;
    cin >> ans;
    if (ans == 'S' || ans == 's') {
        cout << "[Homing] Eje X...\n";
        gimbal.homeMotor(axe1);

        cout << "[Homing] Eje Y...\n";
        gimbal.homeMotor(axe2);

        cout << "[OK] Homing completado.\n\n";
    } else {
        cout << "[Aviso] Omitiendo Homing.\n\n";
    }

    // ---------------------------------------------------------
    // 2) Definir offsets de cero
    // ---------------------------------------------------------
    // La idea es que, mediante "prueba y error", encuentres manualmente
    // los ángulos que realmente dejan tu LED apuntando al piso.
    // Ejemplo: supón que physicallyX=135 y physicallyY=270
    // apuntan exactamente al piso. Entonces esos son tus offsets.

    int offsetX = 0;
    int offsetY = 0;

    cout << "Ingresa el offset (en grados) para el eje X "
         << "(donde se considera 0° apunta exactamente al suelo): ";
    cin >> offsetX;

    cout << "Ingresa el offset (en grados) para el eje Y "
         << "(donde se considera 0° apunta exactamente al suelo): ";
    cin >> offsetY;

    cout << "\n";
    cout << "[INFO] A partir de ahora, llamar a 'rotar eje X a A grados' \n"
         << "       usará la fórmula (offsetX + A) para mandar al motor.\n"
         << "       Lo mismo para Y con offsetY.\n\n";

    // ---------------------------------------------------------
    // 3) Loop interactivo para probar ángulos
    // ---------------------------------------------------------
    cout << "Ejemplo de uso:\n"
         << "  - Teclear 'X 30' => mover eje X a (offsetX + 30) grados.\n"
         << "  - Teclear 'Y -45' => mover eje Y a (offsetY - 45) grados.\n"
         << "  - Teclear 'Q' para salir.\n\n";

    while (true) {
        cout << "Ingresa comando [X|Y] [angulo] o 'Q': ";

        char eje;
        cin >> eje;

        if (eje == 'Q' || eje == 'q') {
            cout << "[Fin] Saliendo del programa.\n";
            break;
        }

        int angulo;
        cin >> angulo;   // leer el ángulo

        if (eje == 'X' || eje == 'x') {
            // El motor X se mueve a offsetX + angulo
            gimbal.moveMotor(offsetX + angulo, axe1, velocity);
        }
        else if (eje == 'Y' || eje == 'y') {
            // El motor Y se mueve a offsetY + angulo
            gimbal.moveMotor(offsetY + angulo, axe2, velocity);
        }
        else {
            cout << "[Error] Comando inválido. Usa 'X', 'Y' o 'Q'.\n";
        }
    }

    cout << "Programa finalizado.\n";
    return 0;
}