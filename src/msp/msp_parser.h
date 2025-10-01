#pragma once

#include <vector>
#include <cstdint>
#include <functional>
#include <array>
#include "msp_protocol.h"

class MSPParser {
public:
    MSPParser();
    void processData(const std::vector<uint8_t>& data);
    void reset();

    std::function<void(const AttitudeData&)> attitudeReceived;
    std::function<void(const RCChannelsData&)> rcChannelsReceived;
    std::function<void(const BatteryData&)> batteryStateReceived;

private:
    bool parseMSPMessage(const std::vector<uint8_t>& message);
    bool validateCRC(const std::vector<uint8_t>& message);

    std::vector<uint8_t> rxBuffer_;
    static const size_t BUFFER_SIZE = 512;
};