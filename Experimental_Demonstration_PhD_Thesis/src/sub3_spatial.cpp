// =============================================================================
// sub3_spatial.cpp  -  Sub-dataset 3: Campana espacial principal
//
// Por cada punto (posicion del receptor):
//   1) Scan vertical {K}: PD al cenit, el LED barre el codebook de K orientaciones.
//   2) Medicion cooperativa {K+1}: PD apuntando al LED y LED apuntando al receptor.
//   3) Scan(s) con tilt aleatorio {K}: PD inclinado con tilt UNIFORME (theta en
//      [0,TILT_MAX_DEG], azimut en [0,360)), generado por este orquestador C++.
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

#include "stdafx.h"
#include "experiment_config.h"
#include "instrument.h"
#include "network_utils.h"
#include "positions.h"
#include "daq.h"
#include "datalog.h"

using namespace std;

static constexpr double kPi = 3.14159265358979323846;

static string promptLine(const string& label) {
    cout << label << ": ";
    string line;
    getline(cin, line);
    size_t a = line.find_first_not_of(" \t\r\n");
    if (a == string::npos) return "NA";
    size_t b = line.find_last_not_of(" \t\r\n");
    return line.substr(a, b - a + 1);
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
    cout << "  Tilt scans / point: " << cfg::N_TILT_SCANS_PER_POINT
         << " (uniform tilt 0-" << cfg::TILT_MAX_DEG << " deg)\n";
    cout << "  Acquisition:        " << cfg::DAQ_ACQ_TIME_SEC << " s (" << cfg::DAQ_N_SAMPLES
         << " samples @ " << cfg::DAQ_FSAMPLE << " Hz)\n";
    cout << "=========================================================\n\n";

    // -------------------------------------------------------------------------
    // Conexion al servidor + base del robot
    // -------------------------------------------------------------------------
    initializeWinsock();
    SOCKET sock = connectToServer(cfg::SERVER_IP, cfg::SERVER_PORT);

    cout << "Robot base position in the global frame.\n";
    double baseX = 0.0, baseY = 0.0;
    { string s = promptLine("  Base X [m] (Enter=0)"); if (s != "NA") baseX = std::stod(s); }
    { string s = promptLine("  Base Y [m] (Enter=0)"); if (s != "NA") baseY = std::stod(s); }
    baseX += cfg::ROBOT_OFFSET_X;
    baseY += cfg::ROBOT_OFFSET_Y;
    sendCoordinates(sock, baseX, baseY, cfg::ROBOT_BASE_Z);
    cout << "Base sent: (" << baseX << ", " << baseY << ", " << cfg::ROBOT_BASE_Z << ")\n\n";

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
        cout << "\n--- Session metadata (press Enter to skip a field) ---\n";
        datalog::Metadata meta;
        meta.set("session_date", datalog::date());
        meta.set("session_time", datalog::clockTime());
        meta.set("subdataset", std::string("3_spatial_campaign"));
        meta.set("operator", promptLine("Operator"));
        meta.set("LED_serial", promptLine("LED serial"));
        meta.set("PD_serial", promptLine("PD serial"));
        meta.set("amp_gain", promptLine("Amplifier gain (TIA+OPAM)"));
        meta.set("ambient_light_state", promptLine("Ambient light (on/off/level)"));
        meta.set("I_LED", promptLine("I_LED [mA] (manual)"));
        meta.set("T_ambient", promptLine("T_ambient [C] (manual)"));
        meta.set("robot_repeatability_mm", promptLine("UR5 repeatability [mm] (datasheet)"));
        meta.set("codebook_id", promptLine("Codebook ID (Enter=TCOM_K9)"));
        meta.set("K_orientations", cfg::K_ORIENTATIONS);
        meta.set("M_repeats", cfg::M_REPEATS);
        meta.set("n_tilt_scans_per_point", cfg::N_TILT_SCANS_PER_POINT);
        meta.set("tilt_max_deg", cfg::TILT_MAX_DEG);
        meta.set("transmitter_x", 0.0);
        meta.set("transmitter_y", 0.0);
        meta.set("transmitter_z", cfg::TRANSMITTER_H);
        meta.set("robot_base_x", baseX);
        meta.set("robot_base_y", baseY);
        meta.set("daq_channel", std::string(cfg::DAQ_CHANNEL));
        meta.set("daq_sample_rate_hz", cfg::DAQ_FSAMPLE);
        meta.set("n_samples", cfg::DAQ_N_SAMPLES);

        // V_dark: measured once per session (LED off). Stored in metadata, not per row.
        cout << "\n[V_dark] Turn the LED OFF and press Enter to measure the baseline (s = skip)...";
        { string tmp; getline(cin, tmp);
          if (tmp != "s" && tmp != "S") {
              DaqStats dark = daqAcquireStats(cfg::DAQ_N_SAMPLES, cfg::DAQ_FSAMPLE);
              if (dark.ok) { meta.set("v_dark_mean", dark.mean); meta.set("v_dark_median", dark.median); meta.set("v_dark_std", dark.std); }
              else meta.set("v_dark_mean", std::string("NA"));
              cout << "  V_dark mean = " << (dark.ok ? dark.mean : 0.0) << " V\n";
              cout << "[V_dark] Turn the LED ON and press Enter to start...";
              { string t2; getline(cin, t2); }
          } else {
              meta.set("v_dark_mean", std::string("NA"));
          }
        }

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
    auto writeMaster = [&](const string& pid, const Position& p, const char* kind,
                           double tiltDeg, double tiltAz, const RobotPose& pose,
                           int oid, double ntIncl, double ntAz, int rep, const DaqStats& st) {
        master.stream() << pid << "," << p.x << "," << p.y << "," << p.z << ","
                        << kind << "," << tiltDeg << "," << tiltAz << ","
                        << pose.px << "," << pose.py << "," << pose.pz << ","
                        << pose.qx << "," << pose.qy << "," << pose.qz << "," << pose.qw << ","
                        << pose.nr_incl << "," << pose.nr_az << ","
                        << oid << "," << ntIncl << "," << ntAz << "," << rep << ","
                        << datalog::date() << "," << datalog::clockTime() << ",";
        writeStats(master, st);
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

    // -------------------------------------------------------------------------
    // Main loop over points
    // -------------------------------------------------------------------------
    const int totalPoints = static_cast<int>(positions.size());
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

        // ---- STAGE 1/3: Vertical baseline scan {K} (PD -> zenith) ----
        cout << "\n--- STAGE 1/3: Vertical baseline scan {K} (PD -> zenith) ---\n";
        cout << "[" << datalog::clockTime() << "] Setting PD to vertical...\n";
        receiverPointingToCeil(sock);
        RobotPose poseV = receivePose(sock, 40);
        if (poseV.valid) {
            for (int r = 1; r <= cfg::M_REPEATS; r++) {
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
                    writeMaster(pointId, p, "vertical", 0.0, 0.0, poseV, i + 1, ntIncl, ntAz, r, st);
                    cout << "    [vertical rep " << r << "/" << cfg::M_REPEATS << "] LED orient "
                         << (i + 1) << "/" << cfg::K_ORIENTATIONS
                         << " (nt_incl=" << ntIncl << ", nt_az=" << ntAz << ")  ->  V = " << vStr(st) << "\n";
                }
            }
            cout << "  STAGE 1 done (vertical {K}).\n";
        } else {
            cout << "  [WARN] Invalid vertical pose; skipping STAGE 1.\n";
        }

        // ---- STAGE 2/3: Cooperative measurement {K+1} (PD -> LED, LED -> PD) ----
        cout << "\n--- STAGE 2/3: Cooperative measurement {K+1} ---\n";
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
                cout << "  STAGE 2 done (K+1).\n";
            } else {
                cout << "  [WARN] Motor error at (K+1); skipping.\n";
            }
        } else {
            cout << "  [WARN] Invalid 'pointed' pose; skipping (K+1).\n";
        }

        // ---- STAGE 3/3: Random-tilt scans {K} (PD tilted) ----
        cout << "\n--- STAGE 3/3: Random-tilt scans {K} (PD tilted) ---\n";
        for (int t = 0; t < cfg::N_TILT_SCANS_PER_POINT; t++) {
            double theta = tiltDist(rng);
            double az    = azDist(rng);
            cout << "[" << datalog::clockTime() << "] Tilt scan " << (t + 1) << "/"
                 << cfg::N_TILT_SCANS_PER_POINT << ": PD theta=" << theta << " deg, az=" << az << " deg\n";
            receiverTilt(sock, theta, az);
            RobotPose poseT = receivePose(sock, 40);
            if (!poseT.valid) { cout << "  [WARN] Invalid tilt pose; skipping this tilt.\n"; continue; }
            for (int r = 1; r <= cfg::M_REPEATS; r++) {
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
                    writeMaster(pointId, p, "tilt", theta, az, poseT, i + 1, ntIncl, ntAz, r, st);
                    cout << "    [tilt#" << (t + 1) << " rep " << r << "/" << cfg::M_REPEATS << "] LED orient "
                         << (i + 1) << "/" << cfg::K_ORIENTATIONS
                         << " (nt_incl=" << ntIncl << ", nt_az=" << ntAz << ")  ->  V = " << vStr(st) << "\n";
                }
            }
        }
        cout << "  STAGE 3 done (tilt {K}).\n";

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
