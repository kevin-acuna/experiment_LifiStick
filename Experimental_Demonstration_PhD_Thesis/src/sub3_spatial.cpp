// =============================================================================
// sub3_spatial.cpp  -  Sub-dataset 3: Campana espacial principal
//
// Por cada punto (posicion del receptor):
//   1) Scan vertical {K}: PD al cenit, el LED barre el codebook de K orientaciones.
//   2) Medicion cooperativa {K+1}: PD apuntando al LED y LED apuntando al receptor.
//   3) Scan(s) con tilt aleatorio {K}: PD inclinado con tilt UNIFORME (theta en
//      [0,TILT_MAX_DEG], azimut en [0,360)), generado por este orquestador C++.
//      OPCIONAL: se OMITE por completo si cfg::N_TILT_SCANS_PER_POINT == 0.
//
// Almacenamiento (dataset_specification.md seccion 5):
//   - master.csv : una fila por (point_id, orientation_id, repeat_id) -> 4.1-4.2
//   - kp1.csv    : mediciones cooperativas (K+1)                      -> 4.3
// Se guarda el resumen estadistico (media/mediana/std) por medida, NO las crudas.
// La orientacion del PD se guarda como cuaternion (pose_UR5) y como inclinacion
// + azimut globales (nr_incl, nr_az). I_LED/temperaturas: metadata manual.
//
// Referencia de parametros: exp_3D.cpp (version validada del proyecto).
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
#include <sstream>
#include <random>
#include <fstream>

#include "stdafx.h"
#include "experiment_config.h"
#include "instrument.h"
#include "network_utils.h"
#include "positions.h"
#include "daq.h"
#include "datalog.h"

using namespace std;

static constexpr double kPi = 3.14159265358979323846;

// Operator's answer to the anti-stuck warning (see askStuckDecision in main).
enum class StuckAnswer {
    Stop,    // [N] halt the campaign and leave the current point NOT done
    Retry,   // [C] robot fixed: discard the suspect rows and re-measure the point
    Accept   // [A] robot verified healthy: keep the rows, it was a false positive
};

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

// Measures V_dark (LED off) ONCE and appends it to the session metadata file.
// Called at the END of the run: turning the LED off/on at the start could shift
// its operating point and perturb the campaign, while at the end it is harmless.
static void measureVdarkToMetadata(const string& metadataPath) {
    cout << "\n[V_dark] Campaign finished. Turn OFF the LED MANUALLY (it cannot be switched "
            "off by software) and press Enter to measure the baseline (or 's' + Enter to skip)...";
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

// Orientacion del LED (inclinacion, azimut globales) para apuntar al receptor.
// LED en (0,0,TRANSMITTER_H); receptor en (rx,ry,rz). Inclinacion desde el nadir.
static void ledToReceiver(double rx, double ry, double rz, double& incl, double& az) {
    double dx = rx, dy = ry, dz = rz - cfg::TRANSMITTER_H;  // dz < 0 (receptor debajo)
    double r = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (r < 1e-9) { incl = 0.0; az = 0.0; return; }
    incl = std::acos(-dz / r) * 180.0 / kPi;   // 0 = nadir
    az = std::atan2(dy, dx) * 180.0 / kPi;
    if (az < 0) az += 360.0;
}

// -----------------------------------------------------------------------------
// Predefined robot base positions (X, Y) in the global frame [m].
// The array size is deduced automatically from the number of initializers.
// -----------------------------------------------------------------------------
static const double PREDEFINED_POSITIONS[][2] = {
    { -0.5, -0.5 }, { -0.5,  0.0 }, { -0.5,  0.5 }, { -0.5,  1.0 }, { -0.5,  1.5 },
    {  0.0, -0.5 }, {  0.0,  0.0 }, {  0.0,  0.5 }, {  0.0,  1.0 }, {  0.0,  1.5 },
    {  0.5, -0.5 }, {  0.5,  0.0 }, {  0.5,  0.5 }, {  0.5,  1.0 }, {  0.5,  1.5 },
    {  1.0, -0.5 }, {  1.0,  0.0 }, {  1.0,  0.5 }, {  1.0,  1.0 }, {  1.0,  1.5 },
    {  1.5, -0.5 }, {  1.5,  0.0 }, {  1.5,  0.5 }, {  1.5,  1.0 }, {  1.5,  1.5 }
};

// Holds a manually entered (custom) robot base position.
struct CustomPosition { double x; double y; bool isCustom; };
static CustomPosition customPos = { 0.0, 0.0, false };

// Shows the predefined-position menu and returns the 1-based selection index.
// Returns -1 if the operator chose to quit, or -2 if a custom position was entered.
static int promptUserForRobotPositionIndex() {
    const size_t numPositions = sizeof(PREDEFINED_POSITIONS) / sizeof(PREDEFINED_POSITIONS[0]);
    while (true) {
        cout << "\n"
             << "====================================================\n"
             << " Select Robot Position [1.." << numPositions << "], 'custom', or 'q' to quit\n"
             << "----------------------------------------------------\n";
        for (size_t i = 0; i < numPositions; i++) {
            cout << " " << (i + 1) << ")  ("
                 << PREDEFINED_POSITIONS[i][0] << ", "
                 << PREDEFINED_POSITIONS[i][1] << ")\n";
        }
        cout << " custom) Enter a custom position\n";
        cout << "----------------------------------------------------\n"
             << "Choose an option: ";

        string input;
        cin >> input;

        if (input == "q" || input == "Q") return -1;

        if (input == "custom" || input == "Custom" || input == "CUSTOM" || input == "c" || input == "C") {
            cout << "\n[Custom position]\n";
            cout << "Enter X coordinate [m]: ";
            while (!(cin >> customPos.x)) {
                cout << "[Error] Invalid value. Enter X coordinate [m]: ";
                cin.clear();
                cin.ignore(numeric_limits<streamsize>::max(), '\n');
            }
            cout << "Enter Y coordinate [m]: ";
            while (!(cin >> customPos.y)) {
                cout << "[Error] Invalid value. Enter Y coordinate [m]: ";
                cin.clear();
                cin.ignore(numeric_limits<streamsize>::max(), '\n');
            }
            customPos.isCustom = true;
            cout << "Custom position set: (" << customPos.x << ", " << customPos.y << ")\n";
            return -2;
        }

        try {
            int index = stoi(input);
            if (index >= 1 && index <= static_cast<int>(numPositions)) {
                customPos.isCustom = false;
                return index;
            }
        } catch (...) { /* not a number */ }

        cout << "[Error] Invalid selection. Please try again.\n";
        cin.clear();
        cin.ignore(numeric_limits<streamsize>::max(), '\n');
    }
}

int main() {
    system("chcp 65001 > nul");

    // -------------------------------------------------------------------------
    // Cargar posiciones del receptor
    // -------------------------------------------------------------------------
    vector<Position> positions;
    loadPositions(cfg::S3_POSITIONS_FILE, positions);
    if (positions.empty()) {
        cerr << "Error: could not load positions from " << cfg::S3_POSITIONS_FILE << "\n";
        return 1;
    }

    cout << "=========================================================\n";
    cout << " SUB-DATASET 3: Main spatial campaign\n";
    cout << "=========================================================\n";
    cout << "  Points loaded:      " << positions.size() << "\n";
    cout << "  K (codebook):       " << cfg::K_ORIENTATIONS << "\n";
    cout << "  M_repeats:          " << cfg::M_REPEATS << "\n";
    cout << "  Cooperative {K+1}:  " << (cfg::S3_ENABLE_COOPERATIVE ? "ENABLED"
                                                                    : "DISABLED (STAGE 2 skipped)") << "\n";
    if (cfg::N_TILT_SCANS_PER_POINT > 0)
        cout << "  Tilt scans / point: " << cfg::N_TILT_SCANS_PER_POINT
             << " (uniform tilt 0-" << cfg::TILT_MAX_DEG << " deg)\n";
    else
        cout << "  Tilt scans / point: 0 (STAGE 3 DISABLED)\n";
    cout << "  Acquisition:        " << cfg::DAQ_ACQ_TIME_SEC << " s (" << cfg::DAQ_N_SAMPLES
         << " samples @ " << cfg::DAQ_FSAMPLE << " Hz)\n";
    if (cfg::S3_STUCK_CHECK_ENABLED)
        cout << "  Anti-stuck check:   ENABLED (pauses if two consecutive points give the\n"
             << "                      same vertical {K} vector within " << cfg::S3_STUCK_TOL_V << " V)\n";
    else
        cout << "  Anti-stuck check:   DISABLED (a silent UR5 protective stop will NOT be caught)\n";
    cout << "=========================================================\n\n";

    // -------------------------------------------------------------------------
    // Conexion al servidor + base del robot
    // -------------------------------------------------------------------------
    initializeWinsock();
    SOCKET sock = connectToServer(cfg::SERVER_IP, cfg::SERVER_PORT);

    cout << "Robot base position in the global frame.\n";
    int posIndex = promptUserForRobotPositionIndex();
    if (posIndex == -1) {
        cout << "User selected 'q' to quit. Exiting.\n";
        closesocket(sock); WSACleanup();
        return 0;
    }
    // Consume the trailing newline left by 'cin >>' so the later getline() prompts work.
    cin.ignore(numeric_limits<streamsize>::max(), '\n');

    // Logical (intended) base position selected by the operator.
    double baseXsel, baseYsel;
    if (posIndex == -2) {
        baseXsel = customPos.x; baseYsel = customPos.y;
        cout << "  Using custom position: (" << baseXsel << ", " << baseYsel << ")\n";
    } else {
        baseXsel = PREDEFINED_POSITIONS[posIndex - 1][0];
        baseYsel = PREDEFINED_POSITIONS[posIndex - 1][1];
        cout << "  Selected position " << posIndex << ": (" << baseXsel << ", " << baseYsel << ")\n";
    }

    // Commanded base = intended + calibration offset (what is actually sent to the UR5).
    double baseX = baseXsel + cfg::ROBOT_OFFSET_X;
    double baseY = baseYsel + cfg::ROBOT_OFFSET_Y;
    sendCoordinates(sock, baseX, baseY, cfg::ROBOT_BASE_Z);
    cout << "Base sent (with offsets): (" << baseX << ", " << baseY << ", " << cfg::ROBOT_BASE_Z << ")\n\n";

    // -------------------------------------------------------------------------
    // Gimbal del transmisor
    // -------------------------------------------------------------------------
    instrument gimbal;
    gimbal.setSerialNo_MotorX(cfg::MOTOR_AXIS_X);
    gimbal.setSerialNo_MotorY(cfg::MOTOR_AXIS_Y);
    gimbal.setTransmitterPosition(0, 0, cfg::TRANSMITTER_H);

    // -------------------------------------------------------------------------
    // Sesion + reanudacion
    // -------------------------------------------------------------------------
    const string baseDir   = "output/sub3_spatial";
    const string stateFile = baseDir + "/state.txt";
    datalog::ensureDir(baseDir);

    const string masterHeader =
        "point_id,x,y,z,scan_kind,tilt_cmd_deg,tilt_cmd_az,"
        "pose_px,pose_py,pose_pz,pose_qx,pose_qy,pose_qz,pose_qw,"
        "nr_incl,nr_az,orientation_id,nt_incl,nt_az,repeat_id,"
        "date,time,v_mean,v_median,v_std,n_samples,fs";
    const string kp1Header =
        "point_id,repeat_id,nt_incl_kp1,nt_az_kp1,"
        "pose_px,pose_py,pose_pz,pose_qx,pose_qy,pose_qz,pose_qw,"
        "nr_incl,nr_az,date,time,v_mean,v_median,v_std,n_samples,fs";

    datalog::ResumeState rs = datalog::loadState(stateFile);
    int startIndex = 0, pointCounter = 0;
    string sessionStamp, sessionDir;
    datalog::CsvWriter master, kp1;

    bool resuming = rs.valid;
    if (resuming) {
        cout << "Previous state found (point " << rs.nextIndex << "/" << positions.size()
             << "). C = continue, R = restart, Q = quit: ";
        char op; cin >> op; op = toupper(op);
        cin.ignore(numeric_limits<streamsize>::max(), '\n');
        if (op == 'Q') { receiverFinished(sock); closesocket(sock); WSACleanup(); return 0; }
        if (op == 'C') {
            startIndex = rs.nextIndex; pointCounter = rs.counter; sessionStamp = rs.sessionStamp;
            sessionDir = baseDir + "/" + sessionStamp;
            master.open(sessionDir + "/master.csv", masterHeader, /*append*/true);
            kp1.open(sessionDir + "/kp1.csv", kp1Header, /*append*/true);
        } else {
            resuming = false;
        }
    }

    if (!resuming) {
        cout << "\n--- Session metadata (press Enter to accept the [default]) ---\n";
        datalog::Metadata meta;
        meta.set("session_date", datalog::date());
        meta.set("session_time", datalog::clockTime());
        meta.set("subdataset", std::string("3_spatial_campaign"));
        meta.set("operator", promptLine("Operator", "Kevin"));
        meta.set("LED_serial", promptLine("LED serial", "SFH4725S"));
        meta.set("PD_serial", promptLine("PD serial", "BPX61"));
        meta.set("I_LED", promptLine("I_LED [mA] (manual)", "500"));
        meta.set("T_ambient", promptLine("T_ambient [C] (manual)", "26"));
        meta.set("T_LED", promptLine("T_LED [C] (manual)", "31"));
        meta.set("codebook_id", promptLine("Codebook ID", "Optimized LED Phi_1/2_36.7"));
        meta.set("comment", promptLine("Comment / reason", "NA"));
        meta.set("K_orientations", cfg::K_ORIENTATIONS);
        meta.set("M_repeats", cfg::M_REPEATS);
        meta.set("n_tilt_scans_per_point", cfg::N_TILT_SCANS_PER_POINT);
        meta.set("tilt_max_deg", cfg::TILT_MAX_DEG);
        meta.set("cooperative_kp1_enabled", cfg::S3_ENABLE_COOPERATIVE ? 1 : 0);
        meta.set("low_signal_warn_v", cfg::S3_LOW_SIGNAL_WARN_V);
        meta.set("transmitter_x", 0.0);
        meta.set("transmitter_y", 0.0);
        meta.set("transmitter_z", cfg::TRANSMITTER_H);
        meta.set("robot_position_index", posIndex);   // menu selection (-2 = custom)
        meta.set("robot_base_x", baseXsel);            // intended (menu-selected) position [m]
        meta.set("robot_base_y", baseYsel);
        meta.set("robot_base_x_cmd", baseX);           // commanded = intended + offset (sent to UR5) [m]
        meta.set("robot_base_y_cmd", baseY);
        meta.set("daq_channel", std::string(cfg::DAQ_CHANNEL));
        meta.set("daq_sample_rate_hz", cfg::DAQ_FSAMPLE);
        meta.set("n_samples", cfg::DAQ_N_SAMPLES);
        // V_dark is measured ONCE at the END of the campaign (LED off) and appended
        // to metadata.txt, so turning the LED off/on does not perturb the session.

        sessionStamp = datalog::stamp();
        sessionDir = baseDir + "/" + sessionStamp;
        datalog::ensureDir(sessionDir);
        meta.write(sessionDir + "/metadata.txt");
        master.open(sessionDir + "/master.csv", masterHeader);
        kp1.open(sessionDir + "/kp1.csv", kp1Header);

        cout << "Resetting gimbal to (0,0)...\n";
        gimbal.rotateMotorX(0.0); Sleep(2000);
        gimbal.rotateMotorY(0.0); Sleep(2000);
        startIndex = 0; pointCounter = 0;
    }

    if (!master.isOpen() || !kp1.isOpen()) {
        cerr << "Error: could not open the session CSV files.\n";
        receiverFinished(sock); closesocket(sock); WSACleanup(); return 1;
    }
    master.stream() << std::setprecision(8);
    kp1.stream() << std::setprecision(8);

    // Console logger: mirror everything printed to the console into <session>/run.log
    // so there is a record of the run in case it is interrupted. Restored on exit.
    datalog::ConsoleLogger logger;
    logger.start(sessionDir + "/run.log", resuming);

    cout << "\nSession folder: " << sessionDir << "\n";
    cout << "Console log:    " << sessionDir << "/run.log\n\n";

    // Uniform random tilt generator
    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> tiltDist(0.0, cfg::TILT_MAX_DEG);
    std::uniform_real_distribution<double> azDist(0.0, 360.0);

    // Human-readable voltage for console/log (mean of the acquisition window).
    auto vStr = [](const DaqStats& st) -> string {
        if (!st.ok) return std::string("ACQ_FAIL");
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6) << st.mean << " V";
        return oss.str();
    };

    // CSV writing helpers -----------------------------------------------------
    auto writeStats = [](datalog::CsvWriter& w, const DaqStats& st) {
        if (st.ok) w.stream() << st.mean << "," << st.median << "," << st.std << "," << st.n << "," << cfg::DAQ_FSAMPLE << "\n";
        else       w.stream() << "NA,NA,NA,0," << cfg::DAQ_FSAMPLE << "\n";
        w.flush();
    };
    // Formats ONE master.csv row (newline included) instead of writing it out.
    // Returning a string lets STAGE 1 buffer its rows in memory and commit them
    // only after the anti-stuck check has passed, so a frozen scan never reaches
    // master.csv (otherwise a retried point would leave a corrupt partial scan
    // glued in front of the good one under the same point_id).
    // setprecision(8) mirrors the one applied to master.stream() so the text is
    // byte-for-byte identical to a direct write.
    auto masterRow = [](const string& pid, const Position& p, const char* kind,
                        double tiltDeg, double tiltAz, const RobotPose& pose,
                        int oid, double ntIncl, double ntAz, int rep, const DaqStats& st) -> string {
        std::ostringstream oss;
        oss << std::setprecision(8);
        oss << pid << "," << p.x << "," << p.y << "," << p.z << ","
            << kind << "," << tiltDeg << "," << tiltAz << ","
            << pose.px << "," << pose.py << "," << pose.pz << ","
            << pose.qx << "," << pose.qy << "," << pose.qz << "," << pose.qw << ","
            << pose.nr_incl << "," << pose.nr_az << ","
            << oid << "," << ntIncl << "," << ntAz << "," << rep << ","
            << datalog::date() << "," << datalog::clockTime() << ",";
        if (st.ok) oss << st.mean << "," << st.median << "," << st.std << "," << st.n << "," << cfg::DAQ_FSAMPLE << "\n";
        else       oss << "NA,NA,NA,0," << cfg::DAQ_FSAMPLE << "\n";
        return oss.str();
    };
    auto writeMaster = [&](const string& pid, const Position& p, const char* kind,
                           double tiltDeg, double tiltAz, const RobotPose& pose,
                           int oid, double ntIncl, double ntAz, int rep, const DaqStats& st) {
        master.stream() << masterRow(pid, p, kind, tiltDeg, tiltAz, pose, oid, ntIncl, ntAz, rep, st);
        master.flush();
    };
    auto writeKp1 = [&](const string& pid, int rep, double ntIncl, double ntAz,
                        const RobotPose& pose, const DaqStats& st) {
        kp1.stream() << pid << "," << rep << "," << ntIncl << "," << ntAz << ","
                     << pose.px << "," << pose.py << "," << pose.pz << ","
                     << pose.qx << "," << pose.qy << "," << pose.qz << "," << pose.qw << ","
                     << pose.nr_incl << "," << pose.nr_az << ","
                     << datalog::date() << "," << datalog::clockTime() << ",";
        writeStats(kp1, st);
    };

    // Moves the gimbal to codebook orientation i. Returns false on motor error.
    auto setCodebook = [&](int i, double& ntIncl, double& ntAz) -> bool {
        ntIncl = cfg::CODEBOOK[i][0];
        ntAz   = cfg::CODEBOOK[i][1];
        int mr = gimbal.setTransmitterOrientation(ntIncl, fmod(ntAz + cfg::AZIMUTH_CMD_OFFSET, 360.0));
        return (mr == 0);
    };

    // Non-blocking sanity check: if EVERY reading of a {K} scan is below the
    // threshold (i.e. the maximum voltage is still low), the LED may have been left
    // OFF (it cannot be switched on by software). Only warns; never blocks the run.
    auto warnIfAllLow = [](double maxMean, int nOk, const string& ctx) {
        if (nOk <= 0) return;
        if (maxMean < cfg::S3_LOW_SIGNAL_WARN_V) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(4) << maxMean;
            cout << "  [WARNING] All " << nOk << " readings of the " << ctx
                 << " are below " << cfg::S3_LOW_SIGNAL_WARN_V << " V (max = " << oss.str()
                 << " V). The LED may be OFF (not switched back on after V_dark)."
                    " Non-blocking warning.\n";
        }
    };

    // ---- Anti-stuck check (silent UR5 protective stop) ----------------------
    // Max |dV| between two vertical {K} fingerprints. Returns -1 when they are
    // not comparable (different length, or one of them empty).
    auto maxAbsDiff = [](const vector<double>& a, const vector<double>& b) -> double {
        if (a.empty() || a.size() != b.size()) return -1.0;
        double m = 0.0;
        for (size_t i = 0; i < a.size(); i++) {
            double d = std::fabs(a[i] - b[i]);
            if (d > m) m = d;
        }
        return m;
    };

    // Blocking WARNING + operator decision. 'pp' is the target commanded for the
    // current point and 'prevP' is where the PD physically still is (the previous
    // point). See StuckAnswer for the meaning of the three possible answers.
    auto askStuckDecision = [&](int pointNo, int nPoints, const Position& pp,
                                const Position& prevP, const string& prevLabel,
                                double dev, size_t nDiscard) -> StuckAnswer {
        cout << "\n\n";
        cout << "#############################################################################\n";
        cout << "#                                                                           #\n";
        cout << "#      W A R N I N G  -  T H E   R O B O T   D I D   N O T   M O V E        #\n";
        cout << "#                                                                           #\n";
        cout << "#############################################################################\n";
        cout << "\n";
        cout << "  THE CAMPAIGN IS NOW PAUSED. NOTHING HAS BEEN WRITTEN TO master.csv YET:\n";
        cout << "  what happens to the rows just measured depends on your choice below.\n";
        cout << "\n";
        cout << "  WHAT WAS DETECTED\n";
        cout << "  -----------------\n";
        cout << "  The vertical {K} scan just measured at POINT " << pointNo << "/" << nPoints << "\n";
        cout << "  is IDENTICAL to the one measured at the previous point (" << prevLabel << ").\n";
        cout << "\n";
        cout << "  WHERE THE RECEIVER IS\n";
        cout << "  ---------------------\n";
        cout << "    commanded target for THIS point : (" << pp.x << ", " << pp.y << ", " << pp.z << ")\n";
        cout << "    >>>  PD IS STILL STUCK AT       : (" << prevP.x << ", " << prevP.y << ", "
             << prevP.z << ")  <<<\n";
        cout << "    i.e. the previous point's position: the receiver never left it.\n";
        cout << "\n";
        {   // Local formatting: never touch cout's own precision, otherwise every
            // double printed for the rest of the campaign would change format.
            std::ostringstream d, t;
            d << std::fixed << std::setprecision(5) << dev;
            t << std::fixed << std::setprecision(5) << cfg::S3_STUCK_TOL_V;
            cout << "    max |dV| between both scans : " << d.str() << " V\n";
            cout << "    tolerance                   : " << t.str() << " V\n";
        }
        cout << "\n";
        cout << "  If the photodiode had really travelled to a new position, the " << cfg::K_ORIENTATIONS << "\n";
        cout << "  voltages would have changed by tens of mV at least. Reading the same\n";
        cout << "  vector twice means the PD is STILL PHYSICALLY AT THE PREVIOUS POINT.\n";
        cout << "\n";
        cout << "  MOST LIKELY CAUSE\n";
        cout << "  -----------------\n";
        cout << "  The UR5 is in PROTECTIVE STOP (e.g. the arm touched itself). This is\n";
        cout << "  silent from here: the Python server keeps answering 'reachable' and\n";
        cout << "  keeps returning a pose, because the pose it reports is the COMMANDED\n";
        cout << "  one, not the measured one. So the campaign would happily keep\n";
        cout << "  re-measuring the same physical location for hours.\n";
        cout << "\n";
        cout << "  WHAT TO DO BEFORE CONTINUING\n";
        cout << "  ----------------------------\n";
        cout << "   1. Look at the UR5 teach pendant and clear the protective stop.\n";
        cout << "   2. Check that the arm is not colliding with itself or the structure.\n";
        cout << "   3. Make sure the Python server is STILL RUNNING (do not restart it:\n";
        cout << "      this program keeps the same socket open).\n";
        cout << "   4. Leave the LED ON and do not touch the gimbal.\n";
        cout << "\n";
        cout << "  OPTIONS\n";
        cout << "  -------\n";
        cout << "   [C] CONTINUE - I fixed the robot. Re-measure THIS point (" << pointNo
             << ") from the\n";
        cout << "                  start and then carry on with the rest of the campaign.\n";
        cout << "                  The " << nDiscard << " suspect row(s) are discarded, not saved.\n";
        cout << "   [A] ACCEPT ANYWAY - I checked the robot and there is NO error: it really\n";
        cout << "                  did move, so this is a FALSE POSITIVE. Keep the " << nDiscard << " row(s)\n";
        cout << "                  just measured, save them to master.csv and carry on with\n";
        cout << "                  the campaign. Use this ONLY after checking with your own\n";
        cout << "                  eyes that the PD is at the commanded target above.\n";
        cout << "   [N] DO NOT CONTINUE - stop here. The point is left NOT done, so the\n";
        cout << "                  next run resumes exactly at this point with 'C'.\n";
        cout << "\n";
        while (true) {
            cout << "  Your choice (C = re-measure / A = accept anyway / N = stop): ";
            string ans;
            // EOF (no console) -> stop, the only choice that cannot corrupt the dataset.
            if (!getline(cin, ans)) return StuckAnswer::Stop;
            size_t a = ans.find_first_not_of(" \t\r\n");
            if (a != string::npos) {
                char c = static_cast<char>(toupper(ans[a]));
                if (c == 'C') return StuckAnswer::Retry;
                if (c == 'A') return StuckAnswer::Accept;
                if (c == 'N' || c == 'Q') return StuckAnswer::Stop;
            }
            cout << "  [Error] Type C to re-measure, A to accept anyway, or N to stop.\n";
        }
    };

    // -------------------------------------------------------------------------
    // Main loop over points
    // -------------------------------------------------------------------------
    // Vertical {K} fingerprint of the last successfully validated point, used to
    // tell whether the robot actually moved. Empty on the first point of a run
    // (and right after a resume), so that point cannot be checked - a stuck
    // robot is then caught at the NEXT point instead.
    vector<double> prevVertical;
    string prevVerticalLabel;
    Position prevPos(0.0, 0.0, 0.0);   // where the PD physically is while stuck
    const int totalPoints = static_cast<int>(positions.size());
    const bool coopEnabled = cfg::S3_ENABLE_COOPERATIVE;         // STAGE 2 (cooperative {K+1}) is optional
    const bool tiltEnabled = (cfg::N_TILT_SCANS_PER_POINT > 0);  // STAGE 3 (random tilt {K}) is optional
    const int nStages = 1 + (coopEnabled ? 1 : 0) + (tiltEnabled ? 1 : 0);
    for (int idx = startIndex; idx < totalPoints; idx++) {
        Position& p = positions[idx];
        if (p.done) {
            cout << "[" << datalog::clockTime() << "] Point " << (idx + 1) << "/" << totalPoints
                 << " already marked done -> skip.\n";
            datalog::saveState(stateFile, { idx + 1, "", sessionStamp, pointCounter, true });
            continue;
        }

        cout << "\n#########################################################\n";
        cout << "# POINT " << (idx + 1) << "/" << totalPoints
             << "   target = (" << p.x << ", " << p.y << ", " << p.z << ")\n";
        cout << "#########################################################\n";

        cout << "[" << datalog::clockTime() << "] Moving receiver to target...\n";
        sendCoordinates(sock, p.x, p.y, p.z);
        string reach = receiveResponse(sock, 3);
        if (reach != "reachable") {
            cout << "  -> NOT REACHABLE (server: '" << reach << "'). Skipping point.\n";
            datalog::saveState(stateFile, { idx + 1, "", sessionStamp, pointCounter, true });
            continue;
        }
        cout << "  -> reachable.\n";

        pointCounter++;
        const string pointId = sessionStamp + "_" + std::to_string(pointCounter);
        cout << "  point_id = " << pointId << "\n";

        // ---- STAGE 1: Vertical baseline scan {K} (PD -> zenith) - always on ----
        // The {K} voltages of this scan are this point's "fingerprint": they are
        // compared against the previous point's to prove the robot really moved
        // (cfg::S3_STUCK_CHECK_ENABLED). The rows are buffered and committed to
        // master.csv ONLY after that check passes, so a frozen scan is never
        // stored and a retried point leaves no corrupt partial scan behind.
        int stageNo = 1;
        bool stuckStop = false;        // operator answered "do not continue"
        vector<double> curVertical;    // fingerprint, in (repeat, orientation) order

        for (int attempt = 1; ; attempt++) {
            vector<string> pending;         // STAGE 1 rows, not written yet
            bool fingerprintUsable = true;  // false if any acquisition failed
            bool accepted = false;          // operator overrode the check with [A]
            curVertical.clear();

            cout << "\n--- STAGE " << stageNo << "/" << nStages
                 << ": Vertical baseline scan {K} (PD -> zenith)";
            if (attempt > 1) cout << "   [RETRY " << (attempt - 1) << "]";
            cout << " ---\n";
            cout << "[" << datalog::clockTime() << "] Setting PD to vertical...\n";
            receiverPointingToCeil(sock);
            RobotPose poseV = receivePose(sock, 40);
            if (!poseV.valid) {
                cout << "  [WARN] Invalid vertical pose; skipping STAGE " << stageNo << ".\n";
                break;                      // nothing measured: nothing to check or commit
            }

            for (int r = 1; r <= cfg::M_REPEATS; r++) {
                double maxV = 0.0; int nOk = 0;
                for (int i = 0; i < cfg::K_ORIENTATIONS; i++) {
                    double ntIncl, ntAz;
                    if (!setCodebook(i, ntIncl, ntAz)) {
                        cerr << "[ABORT] Motor error at codebook orientation " << (i + 1) << ".\n";
                        master.close(); kp1.close();
                        datalog::saveState(stateFile, { idx, "", sessionStamp, pointCounter - 1, true });
                        receiverFinished(sock); closesocket(sock); WSACleanup(); return 1;
                    }
                    Sleep(cfg::STABILIZATION_TIME_MS);
                    DaqStats st = daqAcquireStats(cfg::DAQ_N_SAMPLES, cfg::DAQ_FSAMPLE);
                    if (st.ok) {
                        if (nOk == 0 || st.mean > maxV) maxV = st.mean;
                        nOk++;
                        curVertical.push_back(st.mean);
                    } else {
                        fingerprintUsable = false;   // a hole makes the vector non-comparable
                    }
                    pending.push_back(masterRow(pointId, p, "vertical", 0.0, 0.0, poseV,
                                                i + 1, ntIncl, ntAz, r, st));
                    cout << "    [vertical rep " << r << "/" << cfg::M_REPEATS << "] LED orient "
                         << (i + 1) << "/" << cfg::K_ORIENTATIONS
                         << " (nt_incl=" << ntIncl << ", nt_az=" << ntAz << ")  ->  V = " << vStr(st) << "\n";
                }
                warnIfAllLow(maxV, nOk, "vertical {K} scan (rep " + std::to_string(r) + ")");
            }

            // ---- Defensive check: did the PD really travel to a new position? ---
            double dev = -1.0;   // <0 = not comparable, so the check cannot run
            if (cfg::S3_STUCK_CHECK_ENABLED && fingerprintUsable && !prevVertical.empty())
                dev = maxAbsDiff(prevVertical, curVertical);

            if (dev >= 0.0 && dev <= cfg::S3_STUCK_TOL_V) {
                StuckAnswer answer = askStuckDecision(idx + 1, totalPoints, p, prevPos,
                                                      prevVerticalLabel, dev, pending.size());
                if (answer == StuckAnswer::Retry) {
                    cout << "\n  -> CONTINUE: discarding the " << pending.size()
                         << " suspect row(s) and re-measuring point " << (idx + 1) << ".\n";
                    cout << "[" << datalog::clockTime() << "] Re-sending the target to the robot...\n";
                    sendCoordinates(sock, p.x, p.y, p.z);
                    string reachRetry = receiveResponse(sock, 3);
                    if (reachRetry == "reachable") cout << "  -> reachable.\n";
                    else cout << "  [WARNING] server answered '" << reachRetry
                              << "' instead of 'reachable'. Watch the next readings closely.\n";
                    continue;                        // redo the whole vertical scan
                }
                if (answer == StuckAnswer::Stop) {
                    stuckStop = true;
                    break;
                }
                // [A] The operator inspected the robot and found no fault, so this
                // is a false positive: keep the rows and commit them like any other
                // point. Logged loudly so the override is traceable in run.log.
                accepted = true;
                cout << "\n  -> ACCEPT ANYWAY: operator confirmed the robot is healthy and that\n";
                cout << "     this is a FALSE POSITIVE. The " << pending.size()
                     << " row(s) WILL be saved to master.csv.\n";
            }

            // ---- Validated: commit the buffered rows to master.csv --------------
            for (size_t k = 0; k < pending.size(); k++) master.stream() << pending[k];
            master.flush();
            if (fingerprintUsable) {
                prevVertical      = curVertical;
                prevVerticalLabel = "point " + std::to_string(idx + 1) + ", " + pointId;
                prevPos           = p;   // last position the PD is known to have reached
            }
            if (dev >= 0.0) {
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(5) << dev;
                if (accepted)
                    cout << "  Anti-stuck check OVERRIDDEN BY THE OPERATOR: max |dV| vs the previous\n"
                         << "  point was only " << oss.str() << " V, accepted as a false positive.\n";
                else
                    cout << "  Anti-stuck check PASSED: max |dV| vs the previous point = "
                         << oss.str() << " V (> " << cfg::S3_STUCK_TOL_V << " V) -> the robot moved.\n";
            }
            cout << "  STAGE " << stageNo << " done (vertical {K}).\n";
            break;
        }

        // Operator chose to stop: leave this point NOT done and exit cleanly, so
        // the next run resumes exactly here when answering 'C'.
        if (stuckStop) {
            cout << "\n";
            cout << "  CAMPAIGN STOPPED BY THE OPERATOR (the robot was not moving).\n";
            cout << "  Point " << (idx + 1) << "/" << totalPoints
                 << " is left NOT done and none of its rows were written.\n";
            cout << "  Fix the robot, run this program again and answer 'C' to resume\n";
            cout << "  exactly at this point.\n";
            receiverFinished(sock);
            master.close(); kp1.close();
            datalog::saveState(stateFile, { idx, "", sessionStamp, pointCounter - 1, true });
            logger.stop();
            closesocket(sock); WSACleanup();
            return 2;
        }

        // ---- STAGE 2: Cooperative measurement {K+1} (PD -> LED, LED -> PD) - OPTIONAL ----
        // Skipped entirely when cfg::S3_ENABLE_COOPERATIVE == false.
        if (coopEnabled) {
            stageNo++;
            cout << "\n--- STAGE " << stageNo << "/" << nStages << ": Cooperative measurement {K+1} ---\n";
            cout << "[" << datalog::clockTime() << "] Setting PD pointed to LED...\n";
            receiverPointingToTransmitter(sock);
            RobotPose poseP = receivePose(sock, 40);
            if (poseP.valid) {
                double ntInclKp1, ntAzKp1;
                ledToReceiver(p.x, p.y, p.z, ntInclKp1, ntAzKp1);
                cout << "  Aiming LED to receiver (nt_incl=" << ntInclKp1 << ", nt_az=" << ntAzKp1 << ")...\n";
                int mr = gimbal.setTransmitterOrientation(ntInclKp1, fmod(ntAzKp1 + cfg::AZIMUTH_CMD_OFFSET, 360.0));
                if (mr == 0) {
                    Sleep(cfg::STABILIZATION_TIME_MS);
                    for (int r = 1; r <= cfg::M_REPEATS; r++) {
                        DaqStats st = daqAcquireStats(cfg::DAQ_N_SAMPLES, cfg::DAQ_FSAMPLE);
                        writeKp1(pointId, r, ntInclKp1, ntAzKp1, poseP, st);
                        cout << "    [K+1 rep " << r << "/" << cfg::M_REPEATS << "]  ->  V = " << vStr(st) << "\n";
                    }
                    cout << "  STAGE " << stageNo << " done (K+1).\n";
                } else {
                    cout << "  [WARN] Motor error at (K+1); skipping.\n";
                }
            } else {
                cout << "  [WARN] Invalid 'pointed' pose; skipping (K+1).\n";
            }
        }

        // ---- STAGE 3: Random-tilt scans {K} (PD tilted) - OPTIONAL ----
        // Skipped entirely when cfg::N_TILT_SCANS_PER_POINT == 0 (time-limited runs).
        if (tiltEnabled) {
            stageNo++;
            cout << "\n--- STAGE " << stageNo << "/" << nStages << ": Random-tilt scans {K} (PD tilted) ---\n";
            for (int t = 0; t < cfg::N_TILT_SCANS_PER_POINT; t++) {
                double theta = tiltDist(rng);
                double az    = azDist(rng);
                cout << "[" << datalog::clockTime() << "] Tilt scan " << (t + 1) << "/"
                     << cfg::N_TILT_SCANS_PER_POINT << ": PD theta=" << theta << " deg, az=" << az << " deg\n";
                receiverTilt(sock, theta, az);
                RobotPose poseT = receivePose(sock, 40);
                if (!poseT.valid) { cout << "  [WARN] Invalid tilt pose; skipping this tilt.\n"; continue; }
                for (int r = 1; r <= cfg::M_REPEATS; r++) {
                    double maxV = 0.0; int nOk = 0;
                    for (int i = 0; i < cfg::K_ORIENTATIONS; i++) {
                        double ntIncl, ntAz;
                        if (!setCodebook(i, ntIncl, ntAz)) {
                            cerr << "[ABORT] Motor error at codebook orientation " << (i + 1) << " (tilt scan).\n";
                            master.close(); kp1.close();
                            datalog::saveState(stateFile, { idx, "", sessionStamp, pointCounter - 1, true });
                            receiverFinished(sock); closesocket(sock); WSACleanup(); return 1;
                        }
                        Sleep(cfg::STABILIZATION_TIME_MS);
                        DaqStats st = daqAcquireStats(cfg::DAQ_N_SAMPLES, cfg::DAQ_FSAMPLE);
                        if (st.ok) { if (nOk == 0 || st.mean > maxV) maxV = st.mean; nOk++; }
                        writeMaster(pointId, p, "tilt", theta, az, poseT, i + 1, ntIncl, ntAz, r, st);
                        cout << "    [tilt#" << (t + 1) << " rep " << r << "/" << cfg::M_REPEATS << "] LED orient "
                             << (i + 1) << "/" << cfg::K_ORIENTATIONS
                             << " (nt_incl=" << ntIncl << ", nt_az=" << ntAz << ")  ->  V = " << vStr(st) << "\n";
                    }
                    warnIfAllLow(maxV, nOk, "tilt {K} scan #" + std::to_string(t + 1)
                                            + " (rep " + std::to_string(r) + ")");
                }
            }
            cout << "  STAGE " << stageNo << " done (tilt {K}).\n";
        }

        // End of point: release the server, mark the point as recorded, persist state.
        receiverFinished(sock);
        p.done = true;
        savePositions(cfg::S3_POSITIONS_FILE, positions);   // write '1' in the done column
        datalog::saveState(stateFile, { idx + 1, "", sessionStamp, pointCounter, true });
        cout << "[" << datalog::clockTime() << "] POINT " << (idx + 1) << "/" << totalPoints
             << " COMPLETED and marked done in the positions file.\n";
    }

    master.close();
    kp1.close();

    // V_dark: measured ONCE at the END (LED off) so turning the LED off/on does
    // not perturb the campaign. Appended to <session>/metadata.txt.
    measureVdarkToMetadata(sessionDir + "/metadata.txt");

    // Big reminder: the LED was just turned OFF to measure V_dark, so make sure it
    // is turned back ON before starting the next measurement session.
    cout << "\n";
    cout << "#############################################################\n";
    cout << "#                                                           #\n";
    cout << "#   DO NOT FORGET TO TURN THE LED ON FOR A NEW              #\n";
    cout << "#   MEASUREMENT SESSION!                                    #\n";
    cout << "#                                                           #\n";
    cout << "#############################################################\n";

    datalog::deleteState(stateFile);

    cout << "\n=========================================================\n";
    cout << " SPATIAL CAMPAIGN COMPLETED\n";
    cout << "=========================================================\n";
    cout << "  Points recorded: " << pointCounter << "\n";
    cout << "  Data folder:     " << sessionDir << "\n";
    cout << "  Console log:     " << sessionDir << "/run.log\n";

    logger.stop();
    closesocket(sock);
    WSACleanup();
    cout << "Press Enter to exit...";
    { string tmp; getline(cin, tmp); }
    return 0;
}
