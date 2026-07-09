// =============================================================================
// sub2_constant_c.cpp  -  Sub-dataset 2: Calibracion de la constante C
//
// PD directamente bajo el LED, LED apuntando al nadir, PD al cenit, a varias
// distancias conocidas. Automatiza lo que antes se hacia de forma manual:
// el gimbal fija el LED al nadir (0,0) y el programa guia la toma de datos por
// cada distancia, registrando el resumen estadistico del voltaje.
//
// La colocacion del PD a cada distancia es manual (el PD no lo posiciona el
// robot en este caso), por lo que este programa NO usa el servidor del robot.
//
// V_dark se mide una sola vez (LED apagado). I_LED y temperaturas: metadata manual.
// Ver dataset_specification.md, seccion 3.
// =============================================================================

#include <Windows.h>
#include <iostream>
#include <limits>
#include <string>
#include <cmath>
#include <iomanip>

#include "stdafx.h"
#include "experiment_config.h"
#include "instrument.h"
#include "daq.h"
#include "datalog.h"

using namespace std;

static string promptLine(const string& label) {
    cout << label << ": ";
    string line;
    getline(cin, line);
    size_t a = line.find_first_not_of(" \t\r\n");
    if (a == string::npos) return "NA";
    size_t b = line.find_last_not_of(" \t\r\n");
    return line.substr(a, b - a + 1);
}

int main() {
    system("chcp 65001 > nul");

    cout << "=========================================================\n";
    cout << " SUB-DATASET 2: Calibracion de la constante C\n";
    cout << "=========================================================\n";
    cout << "  Distancias [m]: ";
    for (int i = 0; i < cfg::S2_N_DISTANCES; i++) cout << cfg::S2_DISTANCES[i] << " ";
    cout << "\n  Repeticiones por distancia: " << cfg::S2_REPEATS_PER_DISTANCE << "\n";
    cout << "  Adquisicion: " << cfg::DAQ_ACQ_TIME_SEC << " s (" << cfg::DAQ_N_SAMPLES
         << " muestras a " << cfg::DAQ_FSAMPLE << " Hz)\n";
    cout << "=========================================================\n\n";

    // -------------------------------------------------------------------------
    // Gimbal: fijar el LED al nadir (0,0)
    // -------------------------------------------------------------------------
    instrument gimbal;
    gimbal.setSerialNo_MotorX(cfg::MOTOR_AXIS_X);
    gimbal.setSerialNo_MotorY(cfg::MOTOR_AXIS_Y);
    gimbal.setTransmitterPosition(0, 0, cfg::TRANSMITTER_H);

    cout << "Moviendo el LED al nadir (0,0)...\n";
    gimbal.rotateMotorX(0.0); Sleep(2000);
    gimbal.rotateMotorY(0.0); Sleep(2000);
    cout << "LED apuntando al nadir.\n\n";

    // -------------------------------------------------------------------------
    // Metadata de sesion (manual)
    // -------------------------------------------------------------------------
    cout << "--- Metadata de sesion (Enter para omitir un campo) ---\n";
    datalog::Metadata meta;
    meta.set("session_date", datalog::date());
    meta.set("session_time", datalog::clockTime());
    meta.set("subdataset", std::string("2_constant_C"));
    meta.set("operator", promptLine("Operador"));
    meta.set("LED_serial", promptLine("LED serial"));
    meta.set("PD_serial", promptLine("PD serial"));
    meta.set("amp_gain", promptLine("Ganancia TIA+OPAM"));
    meta.set("ambient_light_state", promptLine("Luz ambiente (on/off/nivel)"));
    meta.set("I_LED", promptLine("I_LED [mA] (manual)"));
    meta.set("T_ambient", promptLine("T_ambient [C] (manual)"));
    meta.set("Rphi_ref", promptLine("Referencia calib R(phi) usada (sesion sub1)"));
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
    cout << "[V_dark] ENCIENDA el LED y pulse Enter para continuar...";
    { string tmp; getline(cin, tmp); }

    // -------------------------------------------------------------------------
    // Sesion: directorio + CSV
    // -------------------------------------------------------------------------
    const string baseDir = "output/sub2_constant_c";
    datalog::ensureDir(baseDir);
    const string sessionStamp = datalog::stamp();
    const string sessionDir = baseDir + "/" + sessionStamp;
    datalog::ensureDir(sessionDir);
    meta.write(sessionDir + "/metadata.txt");

    const string csvPath = sessionDir + "/data.csv";
    const string header = "sample_id,date,time,d_calib,repeat_id,"
                          "v_mean,v_median,v_std,n_samples,fs";
    datalog::CsvWriter csv;
    if (!csv.open(csvPath, header)) {
        cerr << "Error: no se pudo crear " << csvPath << "\n";
        return 1;
    }
    cout << "\nSesion creada en: " << sessionDir << "\n\n";

    // -------------------------------------------------------------------------
    // Toma de datos por distancia
    // -------------------------------------------------------------------------
    int counter = 0;
    for (int di = 0; di < cfg::S2_N_DISTANCES; di++) {
        const double d = cfg::S2_DISTANCES[di];
        cout << "-------------------------------------------------\n";
        cout << "Coloque el PD a " << d << " m del LED (PD al cenit, bajo el LED).\n";
        cout << "Pulse Enter cuando este listo (o escriba 's' + Enter para saltar): ";
        string in; getline(cin, in);
        if (in == "s" || in == "S") { cout << "Distancia " << d << " m saltada.\n"; continue; }

        for (int r = 1; r <= cfg::S2_REPEATS_PER_DISTANCE; r++) {
            counter++;
            const string sampleId = sessionStamp + "_" + std::to_string(counter);
            DaqStats st = daqAcquireStats(cfg::DAQ_N_SAMPLES, cfg::DAQ_FSAMPLE);

            csv.stream() << sampleId << "," << datalog::date() << "," << datalog::clockTime() << ","
                         << d << "," << r << ",";
            if (st.ok) {
                csv.stream() << fixed << setprecision(6)
                             << st.mean << "," << st.median << "," << st.std << ","
                             << st.n << "," << cfg::DAQ_FSAMPLE << "\n";
                cout << "  d=" << d << " m  rep " << r << "/" << cfg::S2_REPEATS_PER_DISTANCE
                     << "  media=" << fixed << setprecision(6) << st.mean << " V\n";
            } else {
                csv.stream() << "NA,NA,NA,0," << cfg::DAQ_FSAMPLE << "\n";
                cout << "  [Aviso] Fallo de adquisicion en d=" << d << " m rep " << r << "\n";
            }
            csv.flush();
        }
    }

    csv.close();
    cout << "\n=== CALIBRACION C COMPLETADA ===\n";
    cout << "Datos: " << csvPath << "\n";
    cout << "Pulse Enter para salir...";
    { string tmp; getline(cin, tmp); }
    return 0;
}
