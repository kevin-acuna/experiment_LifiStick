// =============================================================================
// sub0_axis_sweep.cpp  -  Sub-dataset 0: Single-axis R(phi) cut
//
// Same idea as sub1_radiometric.cpp (PD fixed, pointing to the zenith; the LED
// orientation is swept) but instead of the full 360 deg azimuth scan it sweeps a
// SINGLE axis with a SIGNED angle from S0_ANGLE_START to S0_ANGLE_END.
//
// The signed angle 'a' is mapped to a gimbal orientation as:
//   inclination = |a|                (0 = nadir / vertical)
//   azimuth     = azPos if a >= 0, azNeg otherwise   (the two half-axes)
// so the sweep traces one planar cut of the radiation pattern through the nadir.
//
// The operator chooses the axis at startup: X, Y or Both (X then Y).
//   X axis -> azimuth  0 / 180 deg    Y axis -> azimuth 90 / 270 deg
//
// V_dark is measured ONCE at the END (LED off) and appended to the session
// metadata; I_LED and temperatures are manual metadata.
//
// Parameter reference: sub1_radiometric.cpp.
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
    size_t a = line.find_first_not_of(" \t\r\n");
    if (a == string::npos) return def;
    size_t b = line.find_last_not_of(" \t\r\n");
    return line.substr(a, b - a + 1);
}

// Reads a numeric value (meters) from the console, repeating until valid.
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

// Asks which axis to sweep. Returns "X", "Y" or "BOTH".
// Convention: choosing X sweeps the beam from -X to +X (through the nadir);
// choosing Y sweeps from -Y to +Y. The signed axis_angle runs S0_ANGLE_START..END.
static string promptAxis() {
    while (true) {
        cout << "\n--- Axis to sweep ---\n"
             << "  x) X axis:  sweep from -X to +X (azimuth " << cfg::S0_AXIS_X_AZ_NEG << " -> " << cfg::S0_AXIS_X_AZ_POS << " deg, through nadir)\n"
             << "  y) Y axis:  sweep from -Y to +Y (azimuth " << cfg::S0_AXIS_Y_AZ_NEG << " -> " << cfg::S0_AXIS_Y_AZ_POS << " deg, through nadir)\n"
             << "  b) Both    (X then Y)\n"
             << "Choose [x/y/b]: ";
        string in;
        getline(cin, in);
        size_t a = in.find_first_not_of(" \t\r\n");
        if (a == string::npos) { cout << "  [Error] Invalid choice.\n"; continue; }
        char c = static_cast<char>(tolower(static_cast<unsigned char>(in[a])));
        if (c == 'x') return "X";
        if (c == 'y') return "Y";
        if (c == 'b') return "BOTH";
        cout << "  [Error] Invalid choice.\n";
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

// Startup alignment check: moves the gimbal to the nominal nadir (axis_angle = 0,
// inclination 0) and WAITS for the operator to confirm alignment before sweeping.
// This is the nominal LED->PD direction; the operator uses the live reading to
// check that the maximum falls here (any offset is the mechanical/optical misalignment).
static void alignmentCheck(instrument& gimbal) {
    cout << "\n--- Alignment check (nominal nadir, axis_angle = 0) ---\n";
    cout << "Moving gimbal to inclination 0 (LED pointing straight at the PD)...\n";
    int mr = gimbal.setTransmitterOrientation(0.0, fmod(0.0 + cfg::AZIMUTH_CMD_OFFSET, 360.0));
    if (mr != 0) cout << "  [Warn] Motor returned code " << mr << " while aligning.\n";
    Sleep(cfg::STABILIZATION_TIME_MS);

    while (true) {
        DaqStats st = daqAcquireStats(cfg::DAQ_N_SAMPLES, cfg::DAQ_FSAMPLE);
        if (st.ok) cout << "  V(nadir) = " << std::fixed << std::setprecision(6)
                        << st.mean << " V (std=" << st.std << ")\n";
        else       cout << "  [Warn] Acquisition failed.\n";
        cout << "Verify the LED is aligned with the PD. Press Enter to START the sweep, "
                "or 'm' + Enter to measure again: ";
        string in; getline(cin, in);
        if (in != "m" && in != "M") break;
    }
}

int main() {
    system("chcp 65001 > nul");

    // -------------------------------------------------------------------------
    // Axis selection + orientation list (signed angle -> inclination + azimuth)
    // -------------------------------------------------------------------------
    const string axis = promptAxis();

    struct Orientation { string axis; double angle; double inclination; double azimuth; };
    vector<Orientation> orientations;

    // Sweep convention: X goes from -X to +X and Y from -Y to +Y, both through the
    // nadir. axis_angle<0 uses the negative half-axis (azNeg), axis_angle>0 the
    // positive half (azPos); inclination = |axis_angle|.
    auto addAxis = [&](const string& axisName, double azPos, double azNeg) {
        for (double a = cfg::S0_ANGLE_START; a <= cfg::S0_ANGLE_END + 1e-9; a += cfg::S0_ANGLE_STEP) {
            const double incl = std::fabs(a);
            const double az   = (a >= 0.0) ? azPos : azNeg;
            orientations.push_back({ axisName, a, incl, az });
        }
    };
    if (axis == "X" || axis == "BOTH") addAxis("X", cfg::S0_AXIS_X_AZ_POS, cfg::S0_AXIS_X_AZ_NEG);
    if (axis == "Y" || axis == "BOTH") addAxis("Y", cfg::S0_AXIS_Y_AZ_POS, cfg::S0_AXIS_Y_AZ_NEG);

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
    // NOTE: when resuming, choose the SAME axis so the sample indices line up.
    // -------------------------------------------------------------------------
    const string baseDir  = "output/sub0_axis_sweep";
    const string stateFile = baseDir + "/state.txt";
    datalog::ensureDir(baseDir);

    datalog::ResumeState rs = datalog::loadState(stateFile);
    int startIndex = 0, counter = 0;
    string sessionStamp, sessionDir;
    datalog::CsvWriter csvX, csvY;   // one file per axis (data_x.csv / data_y.csv)
    const string header = "sample_id,date,time,axis,axis_angle,phi_cmd,azimuth_cmd,d_fixed,"
                          "v_mean,v_median,v_std,n_samples,fs";

    // Active axes: a single axis writes one file; BOTH writes data_x.csv AND data_y.csv.
    vector<string> activeAxes;
    if (axis == "X" || axis == "BOTH") activeAxes.push_back("X");
    if (axis == "Y" || axis == "BOTH") activeAxes.push_back("Y");

    bool resuming = rs.valid;
    if (resuming) {
        cout << "Previous state found (" << rs.nextIndex << "/" << total
             << "). C = continue, R = restart, Q = quit: ";
        char op; cin >> op; op = toupper(op);
        cin.ignore(numeric_limits<streamsize>::max(), '\n');
        if (op == 'Q') { receiverFinished(sock); closesocket(sock); WSACleanup(); return 0; }
        if (op == 'C') {
            startIndex = rs.nextIndex; counter = rs.counter;
            sessionStamp = rs.sessionStamp;
            sessionDir = baseDir + "/" + sessionStamp;
        } else {
            resuming = false;  // R -> new session
        }
    }
    if (!resuming) {
        sessionStamp = datalog::stamp();
        sessionDir   = baseDir + "/" + sessionStamp;
    }
    datalog::ensureDir(sessionDir);

    // Per-axis CSV paths (derived from the session folder). The '_x' / '_y' suffix
    // tells which axis a file belongs to; for BOTH both files are written in parallel.
    const string csvPathX = sessionDir + "/data_x.csv";
    const string csvPathY = sessionDir + "/data_y.csv";
    auto writerForAxis = [&](const string& ax) -> datalog::CsvWriter& { return ax == "X" ? csvX : csvY; };
    auto pathForAxis   = [&](const string& ax) -> const string&      { return ax == "X" ? csvPathX : csvPathY; };
    const string stateCsvPath = pathForAxis(activeAxes.front());  // representative path stored in state

    // Console logger: from here on, mirror all console output to <session>/run.log
    // so the log keeps a full record (banner, reachability, V_dark, sweep).
    datalog::ConsoleLogger logger;
    logger.start(sessionDir + "/run.log", resuming);
    cout << "Session folder: " << sessionDir << "\n";
    cout << "Console log:    " << sessionDir << "/run.log\n";

    cout << "=========================================================\n";
    cout << " SUB-DATASET 0: Single-axis R(phi) cut\n";
    cout << "=========================================================\n";
    cout << "  Axis:               " << axis << "\n";
    cout << "  Angle:              " << cfg::S0_ANGLE_START << " to " << cfg::S0_ANGLE_END
         << " deg (step " << cfg::S0_ANGLE_STEP << ")\n";
    cout << "  Sweep convention:   X = -X to +X, Y = -Y to +Y (through nadir)\n";
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
    // Fresh session: metadata (manual) + CSV. Resume: re-open CSV in append mode.
    // V_dark is measured at the END (see below).
    // -------------------------------------------------------------------------
    if (resuming) {
        for (const auto& ax : activeAxes) writerForAxis(ax).open(pathForAxis(ax), header, /*append*/true);
    } else {
        // Session metadata (traceability). Press Enter to accept the [default].
        cout << "--- Session metadata (press Enter to accept the [default]) ---\n";
        datalog::Metadata meta;
        meta.set("session_date", datalog::date());
        meta.set("session_time", datalog::clockTime());
        meta.set("subdataset", std::string("0_axis_sweep"));
        meta.set("axis", axis);
        meta.set("angle_start_deg", cfg::S0_ANGLE_START);
        meta.set("angle_end_deg", cfg::S0_ANGLE_END);
        meta.set("angle_step_deg", cfg::S0_ANGLE_STEP);
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
        for (const auto& ax : activeAxes) {
            if (!writerForAxis(ax).open(pathForAxis(ax), header)) {
                cerr << "Error: could not create " << pathForAxis(ax) << "\n";
                receiverFinished(sock); closesocket(sock); WSACleanup(); return 1;
            }
        }

        // Reset gimbal to (0,0)
        cout << "Moving gimbal to (0,0)...\n";
        gimbal.rotateMotorX(0.0); Sleep(2000);
        gimbal.rotateMotorY(0.0); Sleep(2000);
        startIndex = 0; counter = 0;
    }

    for (const auto& ax : activeAxes) {
        if (!writerForAxis(ax).isOpen()) {
            cerr << "Error: CSV not open (" << pathForAxis(ax) << ").\n";
            receiverFinished(sock); closesocket(sock); WSACleanup(); return 1;
        }
        // Fix the numeric format ONCE so every CSV row is consistent.
        writerForAxis(ax).stream() << std::fixed << std::setprecision(6);
    }

    // -------------------------------------------------------------------------
    // Alignment check: go to nominal nadir (axis_angle = 0) and wait for the
    // operator to confirm the LED is aligned with the PD before sweeping.
    // -------------------------------------------------------------------------
    alignmentCheck(gimbal);

    // -------------------------------------------------------------------------
    // Main sweep loop
    // -------------------------------------------------------------------------
    cout << "\n=== STARTING SINGLE-AXIS SWEEP ===\n\n";
    for (int i = startIndex; i < total; i++) {
        const auto& o = orientations[i];
        counter++;
        const string sampleId = sessionStamp + "_" + std::to_string(counter);

        int mr = gimbal.setTransmitterOrientation(
            o.inclination, fmod(o.azimuth + cfg::AZIMUTH_CMD_OFFSET, 360.0));
        if (mr != 0) {
            cerr << "[ABORT] Motor error (code " << mr << ").\n";
            csvX.close(); csvY.close();
            datalog::saveState(stateFile, { i, stateCsvPath, sessionStamp, counter - 1, true });
            receiverFinished(sock); closesocket(sock); WSACleanup();
            cout << "State saved. You can resume (choose the same axis).\n";
            return 1;
        }

        Sleep(cfg::STABILIZATION_TIME_MS);
        DaqStats st = daqAcquireStats(cfg::DAQ_N_SAMPLES, cfg::DAQ_FSAMPLE);

        datalog::CsvWriter& w = writerForAxis(o.axis);  // route the row to its axis file
        w.stream() << sampleId << "," << datalog::date() << "," << datalog::clockTime() << ","
                   << o.axis << "," << o.angle << "," << o.inclination << "," << o.azimuth << ","
                   << cfg::S1_D_FIXED << ",";
        if (st.ok) {
            w.stream() << st.mean << "," << st.median << "," << st.std << ","
                       << st.n << "," << cfg::DAQ_FSAMPLE << "\n";
        } else {
            w.stream() << "NA,NA,NA,0," << cfg::DAQ_FSAMPLE << "\n";
        }
        w.flush();

        cout << "[" << datalog::clockTime() << "] " << (i + 1) << "/" << total
             << "  axis=" << o.axis << "  angle=" << o.angle
             << "  (inc=" << o.inclination << ", az=" << o.azimuth << ")  ->  V = ";
        if (st.ok) { cout << fixed << setprecision(6) << st.mean << " V\n"; }
        else       { cout << "ACQ_FAIL\n"; }

        datalog::saveState(stateFile, { i + 1, stateCsvPath, sessionStamp, counter, true });
    }

    csvX.close(); csvY.close();

    // V_dark: measured ONCE at the END (LED off) so turning the LED off/on does
    // not perturb the sweep. Appended to <session>/metadata.txt.
    measureVdarkToMetadata(sessionDir + "/metadata.txt");

    datalog::deleteState(stateFile);

    cout << "\n=== SINGLE-AXIS SWEEP COMPLETED ===\n";
    for (const auto& ax : activeAxes) cout << "Data: " << pathForAxis(ax) << "\n";

    receiverFinished(sock);
    logger.stop();
    closesocket(sock);
    WSACleanup();
    cout << "Press Enter to exit...";
    { string tmp; getline(cin, tmp); }
    return 0;
}
