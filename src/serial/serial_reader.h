#pragma once

#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>

class SerialImpl;

class SerialReader {
public:
    SerialReader(const std::string& port = "", int baudrate = 115200);
    ~SerialReader();

    bool startReading();
    void stopReading();
    bool isReading() const;
    void enableEmulation();

    bool writeData(const std::vector<uint8_t>& data);
    bool sendMSPRequest(uint8_t command);

    std::string getPortName() const { return port_; }
    int getBaudrate() const { return baudrate_; }
    bool isConnected() const;

    std::function<void(const std::vector<uint8_t>&)> dataReceived;

private:
    void readLoop();
    void requestLoop();
    void emulateData();

    std::vector<uint8_t> createMSPRequest(uint8_t command);

    std::string port_;
    int baudrate_;
    std::unique_ptr<SerialImpl> serialImpl_;
    std::thread readThread_;
    std::thread requestThread_;
    std::atomic<bool> stopRequested_;
    std::mutex serialMutex_;
};