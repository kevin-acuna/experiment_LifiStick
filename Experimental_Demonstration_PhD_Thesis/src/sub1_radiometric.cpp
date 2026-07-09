// =============================================================================
// sub1_radiometric.cpp  -  Sub-dataset 1: Calibracion radiometrica R(phi)
//
// Barre la orientacion del LED (inclinacion x azimut) con el PD fijo apuntando
// al cenit y registra, por cada angulo, el resumen estadistico del voltaje
// (media / mediana / std) en vez de las muestras crudas.
//
// V_dark se mide UNA sola vez por sesion (LED apagado) y se guarda en la
// metadata de la sesion (no por fila). I_LED y temperaturas son metadata manual.
//
// Referencia de parametros: exp_cone_mapping.cpp (version validada del proyecto).
// Ver dataset_specification.md, seccion 2.
// =============================================================================

#define NOMINMAX
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")

#include <Windows.h>
#include <iostream>
#include <limits>
#include <string>
#include <vector>
#include <cmath>
#include <iomanip>

#include "stdafx.h"
#include "experiment_config.h"
#include "instrument.h"
#include "network_utils.h"
#include "daq.h"
#include "datalog.h"

using namespace std;

// Lee una linea de metadata; devuelve "NA" si el operador la deja vacia.
static string promptLine(const string& label) {
    cout << label << ": ";
    string line;
    getline(cin, line);
    // Recorte simple de espacios
    size_t a = line.find_first_not_of(" \t\r\n");
    if (a == string::npos) return "NA";
    size_t b = line.find_last_not_of(" \t\r\n");
    return line.substr(a, b - a + 1);
}

int main() {
    system("chcp 65001 > nul");

    // -------------------------------------------------------------------------
    // Construir la lista de orientaciones (inc, az)
    // -------------------------------------------------------------------------
    struct Orientation { double inclination; double azimuth; };
    vector<Orientation> orientations;
    for (double inc = cfg::S1_INCLINATION_START; inc <= cfg::S1_INCLINATION_END + 1e-9;
         inc += cfg::S1_INCLINATION_STEP) {
        if (inc < 1e-9) {
            orientations.push_back({ 0.0, 0.0 });  // inc~0: azimut irrelevante
        } else {
            for (double az = 0.0; az < 360.0 - 1e-9; az += cfg::S1_AZIMUTH_STEP) {
                orientations.push_back({ inc, az });
            }
        }
    }
    const int total = static_cast<int>(orientations.size());
    const double estMin = total * (cfg::STABILIZATION_TIME_MS / 1000.0 + cfg::DAQ_ACQ_TIME_SEC) / 60.0;

    cout << "=========================================================\n";
    cout << " SUB-DATASET 1: Calibracion radiometrica R(phi)\n";
    cout << "=========================================================\n";
    cout << "  Inclinacion: " << cfg::S1_INCLINATION_START << " a " << cfg::S1_INCLINATION_END
         << " deg (paso " << cfg::S1_INCLINATION_STEP << ")\n";
    cout << "  Azimut: 0 a 360 deg (paso " << cfg::S1_AZIMUTH_STEP << ")\n";
    cout << "  Distancia fija LED-PD: " << cfg::S1_D_FIXED << " m\n";
    cout << "  Adquisicion: " << cfg::DAQ_ACQ_TIME_SEC << " s (" << cfg::DAQ_N_SAMPLES
         << " muestras a " << cfg::DAQ_FSAMPLE << " Hz)\n";
    cout << "  Total orientaciones: " << total << "\n";
    cout << "  Tiempo estimado: " << fixed << setprecision(1) << estMin << " min\n";
    cout << "=========================================================\n\n";

    // -------------------------------------------------------------------------
    // Conexion al servidor y colocacion del PD (vertical, apuntando al cenit)
    // -------------------------------------------------------------------------
    initializeWinsock();
    SOCKET sock = connectToServer(cfg::SERVER_IP, cfg::SERVER_PORT);

    // Base del robot
    sendCoordinates(sock, cfg::ROBOT_OFFSET_X, cfg::ROBOT_OFFSET_Y, cfg::ROBOT_BASE_Z);

    // Posicion del PD (receptor)
    cout << "Enviando PD a (" << cfg::S1_RECEIVER_X << ", " << cfg::S1_RECEIVER_Y
         << ", " << cfg::S1_RECEIVER_Z << ")...\n";
    sendCoordinates(sock, cfg::S1_RECEIVER_X, cfg::S1_RECEIVER_Y, cfg::S1_RECEIVER_Z);
    string response = receiveResponse(sock, 2);
    if (response != "reachable") {
        cerr << "Error: posicion del PD no alcanzable. Respuesta: " << response << "\n";
        closesocket(sock); WSACleanup(); return 1;
    }
    receiverPointingToCeil(sock);
    RobotPose pdPose = receivePose(sock, 30);
    if (!pdPose.valid) {
        cerr << "Error: el PD no alcanzo la orientacion vertical.\n";
        closesocket(sock); WSACleanup(); return 1;
    }
    cout << "PD en posicion, apuntando al cenit.\n\n";

    // -------------------------------------------------------------------------
    // Gimbal del transmisor
    // -------------------------------------------------------------------------
    instrument gimbal;
    gimbal.setSerialNo_MotorX(cfg::MOTOR_AXIS_X);
    gimbal.setSerialNo_MotorY(cfg::MOTOR_AXIS_Y);
    gimbal.setTransmitterPosition(0, 0, cfg::TRANSMITTER_H);

    // -------------------------------------------------------------------------
    // Sesion: directorio + metadata (manual) + V_dark (una vez) + CSV
    // -------------------------------------------------------------------------
    const string baseDir  = "output/sub1_radiometric";
    const string stateFile = baseDir + "/state.txt";
    datalog::ensureDir(baseDir);

    datalog::ResumeState rs = datalog::loadState(stateFile);
    int startIndex = 0, counter = 0;
    string sessionStamp, csvPath;
    datalog::CsvWriter csv;
    const string header = "sample_id,date,time,phi_cmd,azimuth_cmd,phi_meas,d_fixed,"
                          "v_mean,v_median,v_std,n_samples,fs";

    bool resuming = rs.valid;
    if (resuming) {
        cout << "Estado previo encontrado (" << rs.nextIndex << "/" << total
             << "). C = continuar, R = reiniciar, Q = salir: ";
        char op; cin >> op; op = toupper(op);
        cin.ignore(numeric_limits<streamsize>::max(), '\n');
        if (op == 'Q') { receiverFinished(sock); closesocket(sock); WSACleanup(); return 0; }
        if (op == 'C') {
            startIndex = rs.nextIndex; counter = rs.counter;
            sessionStamp = rs.sessionStamp; csvPath = rs.csvPath;
            csv.open(csvPath, header, /*append*/true);
        } else {
            resuming = false;  // R -> nueva sesion
        }
    }

    if (!resuming) {
        // Metadata de la sesion (trazabilidad). Enter para dejar "NA".
        cout << "\n--- Metadata de sesion (Enter para omitir un campo) ---\n";
        datalog::Metadata meta;
        meta.set("session_date", datalog::date());
        meta.set("session_time", datalog::clockTime());
        meta.set("subdataset", std::string("1_radiometric_Rphi"));
        meta.set("operator", promptLine("Operador"));
        meta.set("LED_serial", promptLine("LED serial"));
        meta.set("PD_serial", promptLine("PD serial"));
        meta.set("amp_gain", promptLine("Ganancia TIA+OPAM"));
        meta.set("ambient_light_state", promptLine("Luz ambiente (on/off/nivel)"));
        meta.set("I_LED", promptLine("I_LED [mA] (manual)"));
        meta.set("T_ambient", promptLine("T_ambient [C] (manual)"));
        meta.set("T_LED", promptLine("T_LED [C] (manual)"));
        meta.set("d_fixed_m", cfg::S1_D_FIXED);
        meta.set("daq_channel", std::string(cfg::DAQ_CHANNEL));
        meta.set("daq_sample_rate_hz", cfg::DAQ_FSAMPLE);
        meta.set("n_samples", cfg::DAQ_N_SAMPLES);

        // V_dark: una sola medida con el LED apagado.
        cout << "\n[V_dark] APAGUE el LED y pulse Enter para medir la linea base...";
        { string tmp; getline(cin, tmp); }
        DaqStats dark = daqAcquireStats(cfg::DAQ_N_SAMPLES, cfg::DAQ_FSAMPLE);
        if (dark.ok) {
            meta.set("v_dark_mean", dark.mean);
            meta.set("v_dark_median", dark.median);
            meta.set("v_dark_std", dark.std);
            cout << "  V_dark media=" << dark.mean << " V, std=" << dark.std << " V\n";
        } else {
            meta.set("v_dark_mean", std::string("NA"));
            cout << "  [Aviso] Fallo al medir V_dark.\n";
        }
        cout << "[V_dark] ENCIENDA el LED y pulse Enter para iniciar el barrido...";
        { string tmp; getline(cin, tmp); }

        // Crear sesion
        sessionStamp = datalog::stamp();
        const string sessionDir = baseDir + "/" + sessionStamp;
        datalog::ensureDir(sessionDir);
        meta.write(sessionDir + "/metadata.txt");
        csvPath = sessionDir + "/data.csv";
        if (!csv.open(csvPath, header)) {
            cerr << "Error: no se pudo crear " << csvPath << "\n";
            receiverFinished(sock); closesocket(sock); WSACleanup(); return 1;
        }
        cout << "\nSesion creada en: " << sessionDir << "\n";

        // Reset del gimbal a (0,0)
        cout << "Moviendo gimbal a (0,0)...\n";
        gimbal.rotateMotorX(0.0); Sleep(2000);
        gimbal.rotateMotorY(0.0); Sleep(2000);
        startIndex = 0; counter = 0;
    }

    if (!csv.isOpen()) {
        cerr << "Error: CSV no abierto.\n";
        receiverFinished(sock); closesocket(sock); WSACleanup(); return 1;
    }

    // -------------------------------------------------------------------------
    // Bucle principal del barrido
    // -------------------------------------------------------------------------
    cout << "\n=== INICIANDO BARRIDO R(phi) ===\n\n";
    for (int i = startIndex; i < total; i++) {
        const auto& o = orientations[i];
        counter++;
        const string sampleId = sessionStamp + "_" + std::to_string(counter);

        cout << "[" << datalog::clockTime() << "] " << (i + 1) << "/" << total
             << "  inc=" << o.inclination << "  az=" << o.azimuth << "\n";

        int mr = gimbal.setTransmitterOrientation(
            o.inclination, fmod(o.azimuth + cfg::AZIMUTH_CMD_OFFSET, 360.0));
        if (mr != 0) {
            cerr << "[ABORT] Error de motores (codigo " << mr << ").\n";
            csv.close();
            datalog::saveState(stateFile, { i, csvPath, sessionStamp, counter - 1, true });
            receiverFinished(sock); closesocket(sock); WSACleanup();
            cout << "Estado guardado. Puede reanudar.\n";
            return 1;
        }

        Sleep(cfg::STABILIZATION_TIME_MS);
        DaqStats st = daqAcquireStats(cfg::DAQ_N_SAMPLES, cfg::DAQ_FSAMPLE);

        csv.stream() << sampleId << "," << datalog::date() << "," << datalog::clockTime() << ","
                     << o.inclination << "," << o.azimuth << ",NA,"
                     << cfg::S1_D_FIXED << ",";
        if (st.ok) {
            csv.stream() << fixed << setprecision(6)
                         << st.mean << "," << st.median << "," << st.std << ","
                         << st.n << "," << cfg::DAQ_FSAMPLE << "\n";
        } else {
            csv.stream() << "NA,NA,NA,0," << cfg::DAQ_FSAMPLE << "\n";
        }
        csv.flush();

        datalog::saveState(stateFile, { i + 1, csvPath, sessionStamp, counter, true });
    }

    csv.close();
    datalog::deleteState(stateFile);

    cout << "\n=== BARRIDO COMPLETADO ===\n";
    cout << "Datos: " << csvPath << "\n";

    receiverFinished(sock);
    closesocket(sock);
    WSACleanup();
    cout << "Pulse Enter para salir...";
    { string tmp; getline(cin, tmp); }
    return 0;
}
