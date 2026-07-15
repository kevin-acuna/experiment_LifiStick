// =============================================================================
// sub1_radiometric.cpp  -  Sub-dataset 1: Radiometric calibration R(phi)
//
// Sweeps the LED orientation (inclination x azimuth) with the PD fixed and
// pointing to the zenith, and records, for each angle, the statistical summary
// of the voltage (mean / median / std) instead of the raw samples.
//
// V_dark is measured ONCE per session (LED off) and stored in the session
// metadata (not per row). I_LED and temperatures are manual metadata.
//
// Parameter reference: exp_cone_mapping.cpp (validated version of the project).
// See dataset_specification.md, section 2.
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

// Reads a metadata line; returns "NA" if the operator leaves it empty.
static string promptLine(const string& label) {
    cout << label << ": ";
    string line;
    getline(cin, line);
    // Simple whitespace trim
    size_t a = line.find_first_not_of(" \t\r\n");
    if (a == string::npos) return "NA";
    size_t b = line.find_last_not_of(" \t\r\n");
    return line.substr(a, b - a + 1);
}

int main() {
    system("chcp 65001 > nul");

    // -------------------------------------------------------------------------
    // Build the list of orientations (inc, az)
    // -------------------------------------------------------------------------
    struct Orientation { double inclination; double azimuth; };
    vector<Orientation> orientations;
    for (double inc = cfg::S1_INCLINATION_START; inc <= cfg::S1_INCLINATION_END + 1e-9;
         inc += cfg::S1_INCLINATION_STEP) {
        if (inc < 1e-9) {
            orientations.push_back({ 0.0, 0.0 });  // inc~0: azimuth irrelevant
        } else {
            for (double az = 0.0; az < 360.0 - 1e-9; az += cfg::S1_AZIMUTH_STEP) {
                orientations.push_back({ inc, az });
            }
        }
    }
    const int total = static_cast<int>(orientations.size());
    const double estMin = total * (cfg::STABILIZATION_TIME_MS / 1000.0 + cfg::DAQ_ACQ_TIME_SEC) / 60.0;

    cout << "=========================================================\n";
    cout << " SUB-DATASET 1: Radiometric calibration R(phi)\n";
    cout << "=========================================================\n";
    cout << "  Inclination:        " << cfg::S1_INCLINATION_START << " to " << cfg::S1_INCLINATION_END
         << " deg (step " << cfg::S1_INCLINATION_STEP << ")\n";
    cout << "  Azimuth:            0 to 360 deg (step " << cfg::S1_AZIMUTH_STEP << ")\n";
    cout << "  Fixed LED-PD dist:  " << cfg::S1_D_FIXED << " m\n";
    cout << "  Acquisition:        " << cfg::DAQ_ACQ_TIME_SEC << " s (" << cfg::DAQ_N_SAMPLES
         << " samples @ " << cfg::DAQ_FSAMPLE << " Hz)\n";
    cout << "  Total orientations: " << total << "\n";
    cout << "  Estimated time:     " << fixed << setprecision(1) << estMin << " min\n";
    cout << "=========================================================\n\n";

    // -------------------------------------------------------------------------
    // Connect to server and place the PD (vertical, pointing to the zenith)
    // -------------------------------------------------------------------------
    initializeWinsock();
    SOCKET sock = connectToServer(cfg::SERVER_IP, cfg::SERVER_PORT);

    // Robot base
    sendCoordinates(sock, cfg::ROBOT_OFFSET_X, cfg::ROBOT_OFFSET_Y, cfg::ROBOT_BASE_Z);

    // PD (receiver) position
    cout << "Sending PD to (" << cfg::S1_RECEIVER_X << ", " << cfg::S1_RECEIVER_Y
         << ", " << cfg::S1_RECEIVER_Z << ")...\n";
    sendCoordinates(sock, cfg::S1_RECEIVER_X, cfg::S1_RECEIVER_Y, cfg::S1_RECEIVER_Z);
    string response = receiveResponse(sock, 2);
    if (response != "reachable") {
        cerr << "Error: PD position not reachable. Response: " << response << "\n";
        closesocket(sock); WSACleanup(); return 1;
    }
    receiverPointingToCeil(sock);
    RobotPose pdPose = receivePose(sock, 30);
    if (!pdPose.valid) {
        cerr << "Error: the PD did not reach the vertical orientation.\n";
        closesocket(sock); WSACleanup(); return 1;
    }
    cout << "PD in position, pointing to the zenith.\n\n";

    // -------------------------------------------------------------------------
    // Transmitter gimbal
    // -------------------------------------------------------------------------
    instrument gimbal;
    gimbal.setSerialNo_MotorX(cfg::MOTOR_AXIS_X);
    gimbal.setSerialNo_MotorY(cfg::MOTOR_AXIS_Y);
    gimbal.setTransmitterPosition(0, 0, cfg::TRANSMITTER_H);

    // -------------------------------------------------------------------------
    // Session: directory + metadata (manual) + V_dark (once) + CSV
    // -------------------------------------------------------------------------
    const string baseDir  = "output/sub1_radiometric";
    const string stateFile = baseDir + "/state.txt";
    datalog::ensureDir(baseDir);

    datalog::ResumeState rs = datalog::loadState(stateFile);
    int startIndex = 0, counter = 0;
    string sessionStamp, csvPath, sessionDir;
    datalog::CsvWriter csv;
    const string header = "sample_id,date,time,phi_cmd,azimuth_cmd,phi_meas,d_fixed,"
                          "v_mean,v_median,v_std,n_samples,fs";

    bool resuming = rs.valid;
    if (resuming) {
        cout << "Previous state found (" << rs.nextIndex << "/" << total
             << "). C = continue, R = restart, Q = quit: ";
        char op; cin >> op; op = toupper(op);
        cin.ignore(numeric_limits<streamsize>::max(), '\n');
        if (op == 'Q') { receiverFinished(sock); closesocket(sock); WSACleanup(); return 0; }
        if (op == 'C') {
            startIndex = rs.nextIndex; counter = rs.counter;
            sessionStamp = rs.sessionStamp; csvPath = rs.csvPath;
            sessionDir = baseDir + "/" + sessionStamp;
            csv.open(csvPath, header, /*append*/true);
        } else {
            resuming = false;  // R -> new session
        }
    }

    if (!resuming) {
        // Session metadata (traceability). Press Enter to leave "NA".
        cout << "\n--- Session metadata (press Enter to skip a field) ---\n";
        datalog::Metadata meta;
        meta.set("session_date", datalog::date());
        meta.set("session_time", datalog::clockTime());
        meta.set("subdataset", std::string("1_radiometric_Rphi"));
        meta.set("operator", promptLine("Operator"));
        meta.set("LED_serial", promptLine("LED serial"));
        meta.set("PD_serial", promptLine("PD serial"));
        meta.set("amp_gain", promptLine("Amplifier gain (TIA+OPAM)"));
        meta.set("ambient_light_state", promptLine("Ambient light (on/off/level)"));
        meta.set("I_LED", promptLine("I_LED [mA] (manual)"));
        meta.set("T_ambient", promptLine("T_ambient [C] (manual)"));
        meta.set("T_LED", promptLine("T_LED [C] (manual)"));
        meta.set("d_fixed_m", cfg::S1_D_FIXED);
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
        cout << "[V_dark] Turn the LED ON and press Enter to start the sweep...";
        { string tmp; getline(cin, tmp); }

        // Create session
        sessionStamp = datalog::stamp();
        sessionDir = baseDir + "/" + sessionStamp;
        datalog::ensureDir(sessionDir);
        meta.write(sessionDir + "/metadata.txt");
        csvPath = sessionDir + "/data.csv";
        if (!csv.open(csvPath, header)) {
            cerr << "Error: could not create " << csvPath << "\n";
            receiverFinished(sock); closesocket(sock); WSACleanup(); return 1;
        }
        cout << "\nSession created at: " << sessionDir << "\n";

        // Reset gimbal to (0,0)
        cout << "Moving gimbal to (0,0)...\n";
        gimbal.rotateMotorX(0.0); Sleep(2000);
        gimbal.rotateMotorY(0.0); Sleep(2000);
        startIndex = 0; counter = 0;
    }

    if (!csv.isOpen()) {
        cerr << "Error: CSV not open.\n";
        receiverFinished(sock); closesocket(sock); WSACleanup(); return 1;
    }

    // Console logger: mirror all console output to <session>/run.log.
    datalog::ConsoleLogger logger;
    logger.start(sessionDir + "/run.log", resuming);
    cout << "Console log: " << sessionDir << "/run.log\n";

    // -------------------------------------------------------------------------
    // Main sweep loop
    // -------------------------------------------------------------------------
    cout << "\n=== STARTING R(phi) SWEEP ===\n\n";
    for (int i = startIndex; i < total; i++) {
        const auto& o = orientations[i];
        counter++;
        const string sampleId = sessionStamp + "_" + std::to_string(counter);

        int mr = gimbal.setTransmitterOrientation(
            o.inclination, fmod(o.azimuth + cfg::AZIMUTH_CMD_OFFSET, 360.0));
        if (mr != 0) {
            cerr << "[ABORT] Motor error (code " << mr << ").\n";
            csv.close();
            datalog::saveState(stateFile, { i, csvPath, sessionStamp, counter - 1, true });
            receiverFinished(sock); closesocket(sock); WSACleanup();
            cout << "State saved. You can resume.\n";
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

        cout << "[" << datalog::clockTime() << "] " << (i + 1) << "/" << total
             << "  inc=" << o.inclination << "  az=" << o.azimuth << "  ->  V = ";
        if (st.ok) { cout << fixed << setprecision(6) << st.mean << " V\n"; }
        else       { cout << "ACQ_FAIL\n"; }

        datalog::saveState(stateFile, { i + 1, csvPath, sessionStamp, counter, true });
    }

    csv.close();
    datalog::deleteState(stateFile);

    cout << "\n=== R(phi) SWEEP COMPLETED ===\n";
    cout << "Data: " << csvPath << "\n";

    receiverFinished(sock);
    logger.stop();
    closesocket(sock);
    WSACleanup();
    cout << "Press Enter to exit...";
    { string tmp; getline(cin, tmp); }
    return 0;
}
