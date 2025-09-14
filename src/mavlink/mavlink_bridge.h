#pragma once

#include <vector>
#include <cstdint>
#include "../msp/msp_protocol.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct MAVLinkMessage {
    std::vector<uint8_t> data;
    uint8_t msgid;
    uint8_t len;
};

class MAVLinkBridge {
public:
    MAVLinkBridge();

    MAVLinkMessage convertAttitude(const AttitudeData& attitude);
    MAVLinkMessage convertRCChannels(const RCChannelsData& channels);
    MAVLinkMessage convertSystemStatus(const BatteryData& battery);
    MAVLinkMessage convertBatteryStatus(const BatteryData& battery);

private:
    uint8_t sequenceNumber_;
    uint8_t systemId_;
    uint8_t componentId_;
};