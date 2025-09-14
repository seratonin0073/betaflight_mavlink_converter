#pragma once

#include <cstdint>
#include <array>

struct AttitudeData {
    float roll;     // degrees
    float pitch;    // degrees
    float yaw;      // degrees
};

struct RCChannelsData {
    std::array<uint16_t, 16> channels; // 16 RC channels
};

struct BatteryData {
    float voltage;      // volts
    float current;      // amperes
    uint32_t capacity;  // mAh
    uint8_t percentage; // %
};