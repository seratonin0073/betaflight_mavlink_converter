#pragma once

#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <functional>
#include <memory>

class SerialImpl;

class SerialReader {
public:
    SerialReader(const std::string& port = "", int baudrate = 115200);
    ~SerialReader();

    bool startReading();
    void stopReading();
    bool isReading() const;
    void enableEmulation();
    void emulateData();

    std::function<void(const std::vector<uint8_t>&)> dataReceived;

private:
    void readLoop();

    std::string port_;
    int baudrate_;
    std::unique_ptr<SerialImpl> serialImpl_;
    std::thread readThread_;
    std::atomic<bool> stopRequested_;
};