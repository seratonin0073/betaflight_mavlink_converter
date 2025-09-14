#include "mavlink_bridge.h"
#include <iostream>
#include <cstring>

#define MAVLINK_MSG_ID_ATTITUDE 30
#define MAVLINK_MSG_ID_RC_CHANNELS 65
#define MAVLINK_MSG_ID_SYS_STATUS 1
#define MAVLINK_MSG_ID_BATTERY_STATUS 147

MAVLinkBridge::MAVLinkBridge()
    : sequenceNumber_(0), systemId_(1), componentId_(1) {}

MAVLinkMessage MAVLinkBridge::convertAttitude(const AttitudeData& attitude) {
    MAVLinkMessage msg;

    msg.msgid = MAVLINK_MSG_ID_ATTITUDE;
    msg.len = 28;

    std::cout << "Конвертація ATTITUDE в MAVLink" << std::endl;

    sequenceNumber_++;
    return msg;
}

MAVLinkMessage MAVLinkBridge::convertRCChannels(const RCChannelsData& channels) {
    MAVLinkMessage msg;

    msg.msgid = MAVLINK_MSG_ID_RC_CHANNELS;
    msg.len = 42;

    std::cout << "Конвертація RC_CHANNELS в MAVLink" << std::endl;

    sequenceNumber_++;
    return msg;
}

MAVLinkMessage MAVLinkBridge::convertSystemStatus(const BatteryData& battery) {
    MAVLinkMessage msg;

    msg.msgid = MAVLINK_MSG_ID_SYS_STATUS;
    msg.len = 31;

    std::cout << "Конвертація SYS_STATUS в MAVLink" << std::endl;

    sequenceNumber_++;
    return msg;
}

MAVLinkMessage MAVLinkBridge::convertBatteryStatus(const BatteryData& battery) {
    MAVLinkMessage msg;

    msg.msgid = MAVLINK_MSG_ID_BATTERY_STATUS;
    msg.len = 36;

    std::cout << "Конвертація BATTERY_STATUS в MAVLink" << std::endl;

    sequenceNumber_++;
    return msg;
}