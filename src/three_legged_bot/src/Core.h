#ifndef CORE_H
#define CORE_H

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <cstdint>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <boost/asio.hpp>
#include "rclcpp/rclcpp.hpp"

using namespace std;
using namespace boost::asio;
using boost::system::error_code;
using namespace std::chrono;

// Command info data
constexpr size_t SIZEOF64 = 8;
constexpr size_t SIZEOF32 = 4;
constexpr size_t SIZEOF16 = 2;

vector<vector<uint8_t>> commandInfoTX = {
    {1, 0x9A, 5, 0, 1, 4, 1, 0, 0, 0, 0},
    {2, 0x9B, 5, 0, 1, 4, 1, 0, 0, 0, 0},
    {3, 0x9C, 5, 0, 1, 4, 2, 0, 0, 0, 0},
    {10, 0xA1, 8, 2, 2, 4, 7, 2, 0, 0, 0, 1},
    {12, 0xA3, 14, 8, 2, 4, 13, 2, 1, 0, 0, 0},
    {13, 0xA4, 18, 12, 2, 4, 17, 2, 1, 1, 0, 0}
};

std::vector<std::vector<std::tuple<std::string, string, int, int>>> commandInfoRX = {
    {
        std::make_tuple("Temperature", "C", 5, 5),
        std::make_tuple("Torque", "N/m", 6, 7),
        std::make_tuple("Speed", "dps", 8, 9),
        std::make_tuple("Position", "Encoder", 10, 11)
    }
};

std::chrono::steady_clock::time_point start;
std::vector<double> elapsedTimes;

inline void writeVectorToCSV(const std::vector<double>& data, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file for writing." << std::endl;
        return;
    }
    for (const auto& value : data) {
        file << value << std::endl;
    }
    file.close();
}

inline void startTimer() {
    start = std::chrono::steady_clock::now();
}

inline void trackElapsedTime() {
    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    elapsedTimes.push_back(elapsed.count());
}

inline void clearScreen() {
#ifdef _WIN32
    std::system("cls");
#else
    std::system("clear");
#endif
}

inline void multiloopControl(uint8_t motorID, int degreePosition, boost::asio::serial_port& serial) {
    constexpr int CMDSize = 14 - 1;
    uint8_t store = 0;
    degreePosition *= 1000;

    stringstream ss;
    ss << hex << degreePosition;
    string hexString = ss.str();
    if (hexString.length() % 2 != 0) {
        hexString = "0" + hexString;
    }

    vector<uint8_t> bytes;
    for (size_t i = 0; i < hexString.length(); i += 2) {
        string byteString = hexString.substr(i, 2);
        uint8_t byte = static_cast<uint8_t>(stoi(byteString, nullptr, 16));
        bytes.push_back(byte);
    }
    reverse(bytes.begin(), bytes.end());

    vector<uint8_t> command = {0x3E, 0xA3, motorID, 0x08, 0x00};
    command.insert(command.end(), bytes.begin(), bytes.end());

    while (command.size() <= CMDSize) {
        command.push_back(0x00);
    }

    for (int i = 0; i < 4; i++) {
        store += command[i];
    }
    command[4] = store;

    store = 0;
    for (int i = 5; i < CMDSize; i++) {
        store += command[i];
    }
    command[13] = store;

    write(serial, buffer(command.data(), command.size()));
}

inline void universialControl(int commandID, uint8_t motorID, int degreePosition, int dps, int torque, uint8_t dir, boost::asio::serial_port& serial) {
    (void)torque;  // Suppress unused warning
    vector<int> checksumLoc;
    int location, index = 0;

    for (location = 0; commandInfoTX[location][index] != commandID; location++) {}

    uint8_t hexCMD = commandInfoTX[location][++index];
    int CMDSize = commandInfoTX[location][++index] - 1;
    uint8_t dataSize = commandInfoTX[location][++index];
    int numCheck = commandInfoTX[location][++index];
    for (int i = 0; i < numCheck; i++) {
        checksumLoc.push_back(commandInfoTX[location][++index]);
    }

    int useAngle = commandInfoTX[location][++index];
    int useDps = commandInfoTX[location][++index];
    (void)useDps;  // Suppress unused warning
    int useDir = commandInfoTX[location][++index];

    degreePosition *= 1000;
    dps *= 1000;

    vector<uint8_t> command = {0x3E, hexCMD, motorID, dataSize};
    uint8_t store = 0;

    for (int i = 0; i < checksumLoc[0]; i++) {
        store += command[i];
    }

    command.push_back(store);

    if (useDir == 1) {
        command.push_back(dir);
    }

    if (useAngle == 1) {
        stringstream ss;
        ss << hex << degreePosition;
        string hexString = ss.str();
        if (hexString.length() % 2 != 0) {
            hexString = "0" + hexString;
        }

        vector<uint8_t> bytes;
        for (size_t i = 0; i < hexString.length(); i += 2) {
            string byteString = hexString.substr(i, 2);
            uint8_t byte = static_cast<uint8_t>(stoi(byteString, nullptr, 16));
            bytes.push_back(byte);
        }

        reverse(bytes.begin(), bytes.end());
        while (bytes.size() < SIZEOF32) {
            bytes.push_back(0x00);
        }

        command.insert(command.end(), bytes.begin(), bytes.end());
    }

    command.push_back(0x00);
    store = 0;

    for (int i = 5; i < CMDSize; i++) {
        store += command[i];
    }

    if (numCheck > 1) {
        command[checksumLoc[1]] = store;
    }

    write(serial, buffer(command.data(), command.size()));
}

inline std::vector<double> readResponse(boost::asio::serial_port& serial, std::vector<uint8_t> buf, boost::system::error_code ec) {
    stringstream ss;
    size_t len = serial.read_some(buffer(buf.data(), buf.size()), ec);
    vector<uint8_t> data;

    if (ec) {
        cerr << "Error on receive: " << ec.message() << endl;
    } else {
        for (size_t i = 0; i < len; ++i) {
            uint8_t byte = static_cast<uint8_t>(buf[i]);
            data.push_back(byte);
        }
    }

    vector<double> rx(4, 0);
    for (size_t i = 0; i < commandInfoRX[0].size(); ++i) {
        ss.str("");
        int startIndexValue = std::get<2>(commandInfoRX[0][i]);
        int maxIndex = std::get<3>(commandInfoRX[0][i]);

        for (size_t j = maxIndex; j >= static_cast<size_t>(startIndexValue) && j < data.size(); --j) {
            ss << setw(2) << setfill('0') << hex << static_cast<int>(data[j]);
        }

        rx[i] = stoi(ss.str(), nullptr, 16);
    }

    return rx;
}

#endif  // CORE_H
