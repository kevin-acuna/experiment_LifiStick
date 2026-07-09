#include "network_utils.h"
#include <iostream>
#include <cstdlib>  // para exit(1)
#include <sstream>
#include <string>

void initializeWinsock() {
    WSADATA wsaData;
    int result = WSAStartup(MAKEWORD(2,2), &wsaData);
    if (result != 0) {
        std::cerr << "WSAStartup failed: " << result << std::endl;
        exit(1);
    }
}

SOCKET connectToServer(const char* ip, int port) {
    SOCKET sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == INVALID_SOCKET) {
        std::cerr << "Error at socket(): " << WSAGetLastError() << std::endl;
        WSACleanup();
        exit(1);
    }

    sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    inet_pton(AF_INET, ip, &serv_addr.sin_addr);

    if (connect(sock, (sockaddr*)&serv_addr, sizeof(serv_addr)) == SOCKET_ERROR) {
        closesocket(sock);
        WSACleanup();
        std::cerr << "[Error] Unable to connect to server!" << std::endl;
        exit(1);
    }
    std::cout << "[Info] Connection with the server was successful" << std::endl;
    return sock;
}

void sendCoordinates(SOCKET sock, double x, double y, double z) {
    double coordinates[3] = { x, y, z };
    int bytesSent = send(sock, reinterpret_cast<char*>(coordinates), sizeof(coordinates), 0);
    if (bytesSent == SOCKET_ERROR) {
        std::cerr << "Send failed: " << WSAGetLastError() << std::endl;
        closesocket(sock);
        WSACleanup();
        exit(1);
    }
}

std::string receiveResponse(SOCKET sock, int timeout_sec) {
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(sock, &readfds);

    timeval timeout;
    timeout.tv_sec = timeout_sec;
    timeout.tv_usec = 0;

    int result = select(sock + 1, &readfds, NULL, NULL, &timeout);
    if (result > 0) {
        char buffer[1024];
        int bytesReceived = recv(sock, buffer, sizeof(buffer), 0);
        if (bytesReceived > 0) {
            return std::string(buffer, bytesReceived);
        }
    } else if (result == 0) {
        return "No response from robot";
    } else {
        std::cerr << "recv failed: " << WSAGetLastError() << std::endl;
        closesocket(sock);
        WSACleanup();
        exit(1);
    }
    return "";
}

void sendMessage(SOCKET sock, const std::string& message) {
    int bytesSent = send(sock, message.c_str(), message.length(), 0);
    if (bytesSent == SOCKET_ERROR) {
        std::cerr << "Send failed: " << WSAGetLastError() << std::endl;
        closesocket(sock);
        WSACleanup();
        exit(1);
    }
}

void receiverPointingToCeil(SOCKET sock) {
    std::cout << "[Info] Sending command for receiver to face upward (vertical)...\n";
    sendMessage(sock, "vertical");  
}

void receiverRandomOrientation(SOCKET sock) {
    std::cout << "[Info] Sending command for receiver to face random orientation...\n";
    sendMessage(sock, "random_n_r");  
}

void receiverPointingToTransmitter(SOCKET sock) {
    std::cout << "[Info] Sending command for receiver to face the transmitter (pointed)...\n";
    sendMessage(sock, "pointed");  
}

void receiverFinished(SOCKET sock) {
    std::cout << "[Info] Sending command for receiver to face the transmitter (finish)...\n";
    sendMessage(sock, "finished");  
}

void receiverTilt(SOCKET sock, double theta_deg, double az_deg) {
    std::ostringstream oss;
    oss << "tilt " << theta_deg << " " << az_deg;
    std::cout << "[Info] Sending tilt command: theta=" << theta_deg
              << " deg, az=" << az_deg << " deg...\n";
    sendMessage(sock, oss.str());
}

RobotPose receivePose(SOCKET sock, int timeout_sec) {
    RobotPose pose;
    std::string resp = receiveResponse(sock, timeout_sec);
    if (resp.empty()) {
        return pose;  // valid = false
    }

    std::istringstream iss(resp);
    std::string tag;
    iss >> tag;
    if (tag != "reached") {
        std::cerr << "[Error] receivePose: respuesta inesperada: '" << resp << "'\n";
        return pose;  // valid = false
    }

    if (iss >> pose.px >> pose.py >> pose.pz
            >> pose.qx >> pose.qy >> pose.qz >> pose.qw
            >> pose.nr_incl >> pose.nr_az) {
        pose.valid = true;
    } else {
        std::cerr << "[Error] receivePose: no se pudo parsear la pose de: '" << resp << "'\n";
    }
    return pose;
}