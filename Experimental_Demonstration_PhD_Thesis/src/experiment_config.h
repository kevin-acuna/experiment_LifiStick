#pragma once
#ifndef EXPERIMENT_CONFIG_H
#define EXPERIMENT_CONFIG_H

// =============================================================================
// experiment_config.h
// Hiperparametros centralizados para los tres sub-datasets (Journal Track 3).
// Todos los valores son configurables aqui; cada programa (sub1/sub2/sub3) los
// consume via el namespace cfg. Ver dataset_specification.md.
// =============================================================================

namespace cfg {

// -----------------------------------------------------------------------------
// Gimbal del transmisor (LED) - IDs de los motores Thorlabs KDC101
// -----------------------------------------------------------------------------
inline constexpr int MOTOR_AXIS_X = 27267164;  // eje externo
inline constexpr int MOTOR_AXIS_Y = 27602122;  // eje interno

// -----------------------------------------------------------------------------
// Geometria del testbed (marco global {G}, unidades en metros)
// -----------------------------------------------------------------------------
inline constexpr double TRANSMITTER_H  = 2.00;   // altura del LED sobre el origen
inline constexpr double ROBOT_OFFSET_X = -0.01;  // offset de calibracion X del robot
inline constexpr double ROBOT_OFFSET_Y = 0.012;  // offset de calibracion Y del robot
inline constexpr double ROBOT_BASE_Z   = 0.646;  // altura de la base del robot

// -----------------------------------------------------------------------------
// DAQ (NI-DAQmx)
// -----------------------------------------------------------------------------
inline constexpr const char* DAQ_CHANNEL = "Dev1/ai1";  // canal de entrada analogica
inline constexpr int    DAQ_FSAMPLE          = 1000;    // frecuencia de muestreo [Hz]
inline constexpr int    DAQ_ACQ_TIME_SEC     = 1;       // tiempo de adquisicion por medida [s]
inline constexpr int    DAQ_N_SAMPLES        = DAQ_ACQ_TIME_SEC * DAQ_FSAMPLE;  // muestras por medida
inline constexpr double DAQ_V_MIN            = -10.0;   // rango minimo [V]
inline constexpr double DAQ_V_MAX            =  10.0;   // rango maximo [V]

// -----------------------------------------------------------------------------
// Tiempos
// -----------------------------------------------------------------------------
inline constexpr int STABILIZATION_TIME_MS = 500;  // espera tras mover el gimbal [ms]

// -----------------------------------------------------------------------------
// Servidor del robot (cliente C++ -> servidor Python -> UR5)
// -----------------------------------------------------------------------------
inline constexpr const char* SERVER_IP   = "127.0.0.1";
inline constexpr int         SERVER_PORT = 12345;

// =============================================================================
// Sub-dataset 1 - Calibracion radiometrica R(phi)
// Barrido de inclinacion x azimut del LED con el PD fijo apuntando al cenit.
// =============================================================================
inline constexpr double S1_INCLINATION_START = 0.0;   // [deg]
inline constexpr double S1_INCLINATION_END   = 90.0;  // [deg]
inline constexpr double S1_INCLINATION_STEP  = 30.0;   // [deg]
inline constexpr double S1_AZIMUTH_STEP      = 90.0;   // [deg] (0 a 360, 360 excluido)

// Posicion fija del PD durante el barrido (marco global)
inline constexpr double S1_RECEIVER_X = 0.0;
inline constexpr double S1_RECEIVER_Y = 0.0;
inline constexpr double S1_RECEIVER_Z = 1.0;
// Distancia fija LED-PD (constante conocida). Por geometria: TRANSMITTER_H - S1_RECEIVER_Z.
inline constexpr double S1_D_FIXED = TRANSMITTER_H - S1_RECEIVER_Z;

// =============================================================================
// Sub-dataset 0 - Barrido de un solo eje (corte del patron R(phi))
// Igual que sub1 pero, en vez del barrido completo de azimut, recorre un unico
// eje (X, Y o ambos) con un angulo CON SIGNO de S0_ANGLE_START a S0_ANGLE_END.
// El angulo con signo se mapea a (inclinacion=|a|, azimut = semieje +/-). Reutiliza
// la posicion fija del PD (S1_RECEIVER_*) y la distancia (S1_D_FIXED) de sub1.
// =============================================================================
inline constexpr double S0_ANGLE_START = -90.0;  // [deg]
inline constexpr double S0_ANGLE_END   =  90.0;  // [deg]
inline constexpr double S0_ANGLE_STEP  =  10.0;  // [deg]

// Azimut logico de cada semieje (lado positivo / negativo del angulo con signo).
inline constexpr double S0_AXIS_X_AZ_POS = 0.0;    // eje X, angulo > 0
inline constexpr double S0_AXIS_X_AZ_NEG = 180.0;  // eje X, angulo < 0
inline constexpr double S0_AXIS_Y_AZ_POS = 90.0;   // eje Y, angulo > 0
inline constexpr double S0_AXIS_Y_AZ_NEG = 270.0;  // eje Y, angulo < 0

// =============================================================================
// Sub-dataset 2 - Calibracion de la constante radiometrica C
// PD bajo el LED, LED al nadir, PD al cenit, a distancias conocidas.
// =============================================================================
inline constexpr int    S2_N_DISTANCES = 5;
inline constexpr double S2_DISTANCES[S2_N_DISTANCES] = { 0.4, 0.6, 0.8, 1.0, 1.2 };  // [m]
inline constexpr int    S2_REPEATS_PER_DISTANCE = 1;

// =============================================================================
// Sub-dataset 3 - Campana espacial principal
// =============================================================================
// Ruta al archivo de posiciones (x y z done por linea)
inline constexpr const char* S3_POSITIONS_FILE = "src/positionsToSample/positions3D.txt";

// Codebook de orientaciones del LED {inclinacion, azimut} en grados (K=9 de TCOM).
inline constexpr int K_ORIENTATIONS = 9;
inline constexpr double CODEBOOK[K_ORIENTATIONS][2] = {
    {  0.0,   0.0 },
    { 34.0, 182.0 },
    { 37.0, 267.0 },
    { 37.0, 355.0 },
    { 42.0,  78.0 },
    { 53.0,  97.0 },
    { 57.0, 179.0 },
    { 58.0, 360.0 },
    { 58.0, 272.0 }
};

// Repeticiones del escaneo {K} por configuracion (M_repeats en la spec).
inline constexpr int M_REPEATS = 1;

// Barrido de tilt del PD (seccion 4.6). El orquestador C++ genera el tilt de forma
// aleatoria con distribucion UNIFORME: inclinacion en [0, TILT_MAX_DEG], azimut en [0,360).
inline constexpr int    N_TILT_SCANS_PER_POINT = 1;    // scans con tilt aleatorio por punto (ademas del vertical)
inline constexpr double TILT_MAX_DEG           = 20.0; // inclinacion maxima del tilt [deg]

// -----------------------------------------------------------------------------
// Offset de alineamiento azimutal del gimbal del LED.
// Se aplica al comandar el motor: motor_az = fmod(az + AZIMUTH_CMD_OFFSET, 360).
// Alinea el cero mecanico del gimbal con el eje +X global. El azimut REGISTRADO
// en el dataset es el intencional (az), no el del motor.
// -----------------------------------------------------------------------------
inline constexpr double AZIMUTH_CMD_OFFSET = 180.0;

} // namespace cfg

#endif // EXPERIMENT_CONFIG_H
