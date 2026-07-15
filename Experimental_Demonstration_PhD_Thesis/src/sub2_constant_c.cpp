// =============================================================================
// sub2_constant_c.cpp  -  Sub-dataset 2: Constant C calibration
//
// PD directly under the LED, LED pointing to the nadir, PD to the zenith, at
// several known distances. Automates what used to be done manually: the gimbal
// fixes the LED to the nadir (0,0) and the program guides the data taking for
// each distance, recording the statistical summary of the voltage.
//
// Placing the PD at each distance is manual (the robot does not position the PD
// in this case), so this program does NOT use the robot server.
//
// V_dark is measured once (LED off). I_LED and temperatures: manual metadata.
// See dataset_specification.md, section 3.
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
    cout << " SUB-DATASET 2: Constant C calibration\n";
    cout << "=========================================================\n";
    cout << "  Distances [m]: ";
    for (int i = 0; i < cfg::S2_N_DISTANCES; i++) cout << cfg::S2_DISTANCES[i] << " ";
    cout << "\n  Repeats per distance: " << cfg::S2_REPEATS_PER_DISTANCE << "\n";
    cout << "  Acquisition: " << cfg::DAQ_ACQ_TIME_SEC << " s (" << cfg::DAQ_N_SAMPLES
         << " samples @ " << cfg::DAQ_FSAMPLE << " Hz)\n";
    cout << "=========================================================\n\n";

    // -------------------------------------------------------------------------
    // Gimbal: fix the LED to the nadir (0,0)
    // -------------------------------------------------------------------------
    instrument gimbal;
    gimbal.setSerialNo_MotorX(cfg::MOTOR_AXIS_X);
    gimbal.setSerialNo_MotorY(cfg::MOTOR_AXIS_Y);
    gimbal.setTransmitterPosition(0, 0, cfg::TRANSMITTER_H);

    cout << "Moving the LED to the nadir (0,0)...\n";
    gimbal.rotateMotorX(0.0); Sleep(2000);
    gimbal.rotateMotorY(0.0); Sleep(2000);
    cout << "LED pointing to the nadir.\n\n";

    // -------------------------------------------------------------------------
    // Session metadata (manual)
    // -------------------------------------------------------------------------
    cout << "--- Session metadata (press Enter to skip a field) ---\n";
    datalog::Metadata meta;
    meta.set("session_date", datalog::date());
    meta.set("session_time", datalog::clockTime());
    meta.set("subdataset", std::string("2_constant_C"));
    meta.set("operator", promptLine("Operator"));
    meta.set("LED_serial", promptLine("LED serial"));
    meta.set("PD_serial", promptLine("PD serial"));
    meta.set("amp_gain", promptLine("Amplifier gain (TIA+OPAM)"));
    meta.set("ambient_light_state", promptLine("Ambient light (on/off/level)"));
    meta.set("I_LED", promptLine("I_LED [mA] (manual)"));
    meta.set("T_ambient", promptLine("T_ambient [C] (manual)"));
    meta.set("Rphi_ref", promptLine("R(phi) calibration reference used (sub1 session)"));
    meta.set("daq_channel", std::string(cfg::DAQ_CHANNEL));
    meta.set("daq_sample_rate_hz", cfg::DAQ_FSAMPLE);
    meta.set("n_samples", cfg::DAQ_N_SAMPLES);

    // V_dark: a single measurement with the LED off.
    cout << "\n[V_dark] Turn the LED OFF and press Enter to measure the baseline...";
    { string tmp; getline(cin, tmp); }
    DaqStats dark = daqAcquireStats(cfg::DAQ_N_SAMPLES, cfg::DAQ_FSAMPLE);
    if (dark.ok) {
        meta.set("v_dark_mean", dark.mean);
        meta.set("v_dark_median", dark.median);
        meta.set("v_dark_std", dark.std);
        cout << "  V_dark mean=" << dark.mean << " V, std=" << dark.std << " V\n";
    } else {
        meta.set("v_dark_mean", std::string("NA"));
        cout << "  [Warn] Failed to measure V_dark.\n";
    }
    cout << "[V_dark] Turn the LED ON and press Enter to continue...";
    { string tmp; getline(cin, tmp); }

    // -------------------------------------------------------------------------
    // Session: directory + CSV
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
        cerr << "Error: could not create " << csvPath << "\n";
        return 1;
    }

    // Console logger: mirror all console output to <session>/run.log.
    datalog::ConsoleLogger logger;
    logger.start(sessionDir + "/run.log");

    cout << "\nSession created at: " << sessionDir << "\n";
    cout << "Console log: " << sessionDir << "/run.log\n\n";

    // -------------------------------------------------------------------------
    // Data taking per distance
    // -------------------------------------------------------------------------
    int counter = 0;
    for (int di = 0; di < cfg::S2_N_DISTANCES; di++) {
        const double d = cfg::S2_DISTANCES[di];
        cout << "-------------------------------------------------\n";
        cout << "Place the PD at " << d << " m from the LED (PD to zenith, under the LED).\n";
        cout << "Press Enter when ready (or type 's' + Enter to skip): ";
        string in; getline(cin, in);
        if (in == "s" || in == "S") { cout << "Distance " << d << " m skipped.\n"; continue; }

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
                cout << "  [" << datalog::clockTime() << "] d=" << d << " m  rep " << r << "/" << cfg::S2_REPEATS_PER_DISTANCE
                     << "  ->  V = " << fixed << setprecision(6) << st.mean << " V\n";
            } else {
                csv.stream() << "NA,NA,NA,0," << cfg::DAQ_FSAMPLE << "\n";
                cout << "  [Warn] Acquisition failed at d=" << d << " m rep " << r << "\n";
            }
            csv.flush();
        }
    }

    csv.close();
    cout << "\n=== CONSTANT C CALIBRATION COMPLETED ===\n";
    cout << "Data: " << csvPath << "\n";
    logger.stop();
    cout << "Press Enter to exit...";
    { string tmp; getline(cin, tmp); }
    return 0;
}
