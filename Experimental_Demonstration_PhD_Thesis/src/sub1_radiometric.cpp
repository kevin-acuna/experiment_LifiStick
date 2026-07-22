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
#include <cctype>
#include <iomanip>
#include <fstream>

#include "stdafx.h"
#include "experiment_config.h"
#include "instrument.h"
#include "network_utils.h"
#include "daq.h"
#include "datalog.h"

using namespace std;

// Reads a metadata line. The default (shown in [brackets]) is returned when the
// operator just presses Enter, so common values need not be retyped every run.
static string promptLine(const string& label, const string& def = "NA") {
    cout << label << " [" << def << "]: ";
    string line;
    getline(cin, line);
    // Simple whitespace trim
    size_t a = line.find_first_not_of(" \t\r\n");
    if (a == string::npos) return def;
    size_t b = line.find_last_not_of(" \t\r\n");
    return line.substr(a, b - a + 1);
}

// Reads a numeric value (meters) from the console, repeating until valid.
// Uses getline to stay consistent with the other getline-based prompts.
static double promptDouble(const string& label) {
    while (true) {
        cout << label << ": ";
        string line;
        getline(cin, line);
        try {
            size_t pos = 0;
            double value = stod(line, &pos);
            while (pos < line.size() && isspace(static_cast<unsigned char>(line[pos]))) pos++;
            if (pos == line.size()) return value;
        } catch (...) {}
        cout << "  [Error] Invalid number, please try again.\n";
    }
}

// Measures V_dark (LED off) ONCE and appends it to the session metadata file.
// Called at the END of the run: turning the LED off/on at the start could shift
// its operating point and perturb the sweep, while at the end it is harmless.
static void measureVdarkToMetadata(const string& metadataPath) {
    cout << "\n[V_dark] Sweep finished. Turn the LED OFF and press Enter to measure the "
            "baseline (or 's' + Enter to skip)...";
    string tmp; getline(cin, tmp);
    if (tmp == "s" || tmp == "S") { cout << "  V_dark skipped.\n"; return; }

    DaqStats dark = daqAcquireStats(cfg::DAQ_N_SAMPLES, cfg::DAQ_FSAMPLE);
    std::ofstream mf(metadataPath, std::ios::app);
    if (!mf.is_open()) { cout << "  [Warn] Could not open metadata to store V_dark.\n"; return; }
    if (dark.ok) {
        mf << "v_dark_mean="   << std::setprecision(10) << dark.mean   << "\n";
        mf << "v_dark_median=" << std::setprecision(10) << dark.median << "\n";
        mf << "v_dark_std="    << std::setprecision(10) << dark.std    << "\n";
        cout << "  V_dark mean=" << dark.mean << " V, std=" << dark.std << " V\n";
    } else {
        mf << "v_dark_mean=NA\n";
        cout << "  [Warn] Failed to measure V_dark.\n";
    }
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

    // -------------------------------------------------------------------------
    // Connect to the robot server
    // -------------------------------------------------------------------------
    initializeWinsock();
    SOCKET sock = connectToServer(cfg::SERVER_IP, cfg::SERVER_PORT);

    // -------------------------------------------------------------------------
    // Session setup + resume decision. Done BEFORE placing the PD so the console
    // logger can capture the banner, the reachability result and V_dark.
    // -------------------------------------------------------------------------
    const string baseDir  = "output/sub1_radiometric";
    const string stateFile = baseDir + "/state.txt";
    datalog::ensureDir(baseDir);

    datalog::ResumeState rs = datalog::loadState(stateFile);
    int startIndex = 0, counter = 0;
    string sessionStamp, csvPath, sessionDir;
    datalog::CsvWriter csv;
    const string header = "sample_id,date,time,phi_cmd,azimuth_cmd,d_fixed,"
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
        } else {
            resuming = false;  // R -> new session
        }
    }
    if (!resuming) {
        sessionStamp = datalog::stamp();
        sessionDir   = baseDir + "/" + sessionStamp;
        csvPath      = sessionDir + "/data.csv";
    }
    datalog::ensureDir(sessionDir);

    // Console logger: from here on, mirror all console output to <session>/run.log
    // so the log keeps a full record (banner, reachability, V_dark, sweep).
    datalog::ConsoleLogger logger;
    logger.start(sessionDir + "/run.log", resuming);
    cout << "Session folder: " << sessionDir << "\n";
    cout << "Console log:    " << sessionDir << "/run.log\n";

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
    // Robot base position (global frame). Asked to the operator because the
    // reachability of the fixed PD target depends on where the robot is placed.
    // The calibration offset (ROBOT_OFFSET_X/Y) is added on top of the entered value.
    // -------------------------------------------------------------------------
    cout << "--- Robot base position (global frame, meters) ---\n";
    const double robotX = promptDouble("Robot base X [m]") + cfg::ROBOT_OFFSET_X;
    const double robotY = promptDouble("Robot base Y [m]") + cfg::ROBOT_OFFSET_Y;
    cout << "Sending robot base to (" << robotX << ", " << robotY << ", "
         << cfg::ROBOT_BASE_Z << ") [calibration offset included]...\n";
    sendCoordinates(sock, robotX, robotY, cfg::ROBOT_BASE_Z);

    // PD (receiver) position
    cout << "Sending PD to (" << cfg::S1_RECEIVER_X << ", " << cfg::S1_RECEIVER_Y
         << ", " << cfg::S1_RECEIVER_Z << ")...\n";
    sendCoordinates(sock, cfg::S1_RECEIVER_X, cfg::S1_RECEIVER_Y, cfg::S1_RECEIVER_Z);
    string response = receiveResponse(sock, 2);
    if (response != "reachable") {
        cerr << "Error: PD position not reachable. Response: " << response << "\n";
        receiverFinished(sock); closesocket(sock); WSACleanup(); return 1;
    }
    receiverPointingToCeil(sock);
    RobotPose pdPose = receivePose(sock, 30);
    if (!pdPose.valid) {
        cerr << "Error: the PD did not reach the vertical orientation.\n";
        receiverFinished(sock); closesocket(sock); WSACleanup(); return 1;
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
    // Fresh session: metadata (manual) + V_dark (once) + CSV.
    // Resume: just re-open the CSV in append mode.
    // -------------------------------------------------------------------------
    if (resuming) {
        csv.open(csvPath, header, /*append*/true);
    } else {
        // Session metadata (traceability). Press Enter to accept the [default].
        cout << "--- Session metadata (press Enter to accept the [default]) ---\n";
        datalog::Metadata meta;
        meta.set("session_date", datalog::date());
        meta.set("session_time", datalog::clockTime());
        meta.set("subdataset", std::string("1_radiometric_Rphi"));
        meta.set("operator", promptLine("Operator", "Kevin"));
        meta.set("LED_serial", promptLine("LED serial", "SFH4725S"));
        meta.set("PD_serial", promptLine("PD serial", "BPX61"));
        meta.set("I_LED", promptLine("I_LED [mA] (manual)", "500"));
        meta.set("T_ambient", promptLine("T_ambient [C] (manual)", "26"));
        meta.set("T_LED", promptLine("T_LED [C] (manual)", "31"));
        meta.set("comment", promptLine("Comment / reason", "NA"));
        meta.set("d_fixed_m", cfg::S1_D_FIXED);
        meta.set("robot_base_x", robotX);
        meta.set("robot_base_y", robotY);
        meta.set("daq_channel", std::string(cfg::DAQ_CHANNEL));
        meta.set("daq_sample_rate_hz", cfg::DAQ_FSAMPLE);
        meta.set("n_samples", cfg::DAQ_N_SAMPLES);

        meta.write(sessionDir + "/metadata.txt");
        if (!csv.open(csvPath, header)) {
            cerr << "Error: could not create " << csvPath << "\n";
            receiverFinished(sock); closesocket(sock); WSACleanup(); return 1;
        }

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
    // Fix the numeric format ONCE so every CSV row is consistent.
    csv.stream() << std::fixed << std::setprecision(6);

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
                     << o.inclination << "," << o.azimuth << ","
                     << cfg::S1_D_FIXED << ",";
        if (st.ok) {
            csv.stream() << st.mean << "," << st.median << "," << st.std << ","
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

    // V_dark: measured ONCE at the END (LED off) so turning the LED off/on does
    // not perturb the sweep. Appended to <session>/metadata.txt.
    measureVdarkToMetadata(sessionDir + "/metadata.txt");

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
