#pragma once

#include <vector>
#include <cstdint>
#include "../msp/msp_protocol.h"

// Включаємо MAVLink заголовки
#include "../lib/mavlink/common/mavlink.h"

struct MAVLinkMessage {
    std::vector<uint8_t> data;
    uint16_t msgid;
    uint8_t len;

    MAVLinkMessage() : msgid(0), len(0) {}
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
    uint64_t bootTimeMs_;

    uint64_t getBootTimeMs() const;
    void packMAVLinkMessage(const mavlink_message_t& mavMsg, MAVLinkMessage& msg);
};