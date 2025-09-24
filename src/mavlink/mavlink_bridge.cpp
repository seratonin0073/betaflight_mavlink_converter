#include "mavlink_bridge.h"
#include <iostream>
#include <chrono>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

MAVLinkBridge::MAVLinkBridge()
    : sequenceNumber_(0),
      systemId_(1),
      componentId_(MAV_COMP_ID_PERIPHERAL) {
    bootTimeMs_ = getBootTimeMs();
}

uint64_t MAVLinkBridge::getBootTimeMs() const {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count();
}

void MAVLinkBridge::packMAVLinkMessage(const mavlink_message_t& mavMsg, MAVLinkMessage& msg) {
    msg.msgid = mavMsg.msgid;
    msg.len = mavMsg.len;

    // Конвертація MAVLink повідомлення у вектор байтів
    msg.data.resize(MAVLINK_MAX_PACKET_LEN);
    uint16_t len = mavlink_msg_to_send_buffer(msg.data.data(), &mavMsg);
    msg.data.resize(len);
}

MAVLinkMessage MAVLinkBridge::convertAttitude(const AttitudeData& attitude) {
    MAVLinkMessage msg;
    mavlink_message_t mavMsg;
    mavlink_attitude_t attitudeMsg;

    // Заповнення даних ATTITUDE
    attitudeMsg.time_boot_ms = getBootTimeMs() - bootTimeMs_;
    attitudeMsg.roll = attitude.roll * M_PI / 180.0f;  // градуси -> радіани
    attitudeMsg.pitch = attitude.pitch * M_PI / 180.0f;
    attitudeMsg.yaw = attitude.yaw * M_PI / 180.0f;
    attitudeMsg.rollspeed = 0.0f;  // Не надається MSP
    attitudeMsg.pitchspeed = 0.0f;
    attitudeMsg.yawspeed = 0.0f;

    // Кодування MAVLink повідомлення
    mavlink_msg_attitude_encode(systemId_, componentId_, &mavMsg, &attitudeMsg);
    packMAVLinkMessage(mavMsg, msg);

    std::cout << "✅ Конвертовано ATTITUDE: "
              << "roll=" << attitude.roll << "° -> " << attitudeMsg.roll << "rad" << std::endl;

    sequenceNumber_++;
    return msg;
}

MAVLinkMessage MAVLinkBridge::convertRCChannels(const RCChannelsData& channels) {
    MAVLinkMessage msg;
    mavlink_message_t mavMsg;
    mavlink_rc_channels_t rcMsg;

    // Заповнення даних RC_CHANNELS - ВИПРАВЛЕНА ВЕРСІЯ
    rcMsg.time_boot_ms = getBootTimeMs() - bootTimeMs_;
    rcMsg.chancount = 16;
    rcMsg.rssi = 255;  // Максимальний RSSI


    rcMsg.chan1_raw = channels.channels[0];
    rcMsg.chan2_raw = channels.channels[1];
    rcMsg.chan3_raw = channels.channels[2];
    rcMsg.chan4_raw = channels.channels[3];
    rcMsg.chan5_raw = channels.channels[4];
    rcMsg.chan6_raw = channels.channels[5];
    rcMsg.chan7_raw = channels.channels[6];
    rcMsg.chan8_raw = channels.channels[7];
    rcMsg.chan9_raw = channels.channels[8];
    rcMsg.chan10_raw = channels.channels[9];
    rcMsg.chan11_raw = channels.channels[10];
    rcMsg.chan12_raw = channels.channels[11];
    rcMsg.chan13_raw = channels.channels[12];
    rcMsg.chan14_raw = channels.channels[13];
    rcMsg.chan15_raw = channels.channels[14];
    rcMsg.chan16_raw = channels.channels[15];
    rcMsg.chan17_raw = UINT16_MAX;
    rcMsg.chan18_raw = UINT16_MAX;
    // Кодування MAVLink повідомлення
    mavlink_msg_rc_channels_encode(systemId_, componentId_, &mavMsg, &rcMsg);
    packMAVLinkMessage(mavMsg, msg);

    std::cout << "✅ Конвертовано RC_CHANNELS: "
              << channels.channels[0] << ", " << channels.channels[1] << ", ..."
              << " (RSSI: " << static_cast<int>(rcMsg.rssi) << ")" << std::endl;

    sequenceNumber_++;
    return msg;
}

MAVLinkMessage MAVLinkBridge::convertSystemStatus(const BatteryData& battery) {
    MAVLinkMessage msg;
    mavlink_message_t mavMsg;
    mavlink_sys_status_t sysMsg;

    // Заповнення даних SYS_STATUS
    sysMsg.onboard_control_sensors_present =
        MAV_SYS_STATUS_SENSOR_3D_GYRO |
        MAV_SYS_STATUS_SENSOR_3D_ACCEL |
        MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;

    sysMsg.onboard_control_sensors_enabled = sysMsg.onboard_control_sensors_present;
    sysMsg.onboard_control_sensors_health = sysMsg.onboard_control_sensors_present;

    // Батарея
    sysMsg.voltage_battery = battery.voltage * 1000;  // V -> mV
    sysMsg.current_battery = battery.current * 100;   // A -> 10mA
    sysMsg.battery_remaining = battery.percentage;

    // Інші поля
    sysMsg.load = 0;
    sysMsg.voltage_battery = 0;
    sysMsg.drop_rate_comm = 0;
    sysMsg.errors_comm = 0;
    sysMsg.errors_count1 = 0;
    sysMsg.errors_count2 = 0;
    sysMsg.errors_count3 = 0;
    sysMsg.errors_count4 = 0;

    // Кодування MAVLink повідомлення
    mavlink_msg_sys_status_encode(systemId_, componentId_, &mavMsg, &sysMsg);
    packMAVLinkMessage(mavMsg, msg);

    std::cout << "✅ Конвертовано SYS_STATUS: "
              << "voltage=" << battery.voltage << "V, "
              << "current=" << battery.current << "A, "
              << "remaining=" << static_cast<int>(battery.percentage) << "%" << std::endl;

    sequenceNumber_++;
    return msg;
}

MAVLinkMessage MAVLinkBridge::convertBatteryStatus(const BatteryData& battery) {
    MAVLinkMessage msg;
    mavlink_message_t mavMsg;
    mavlink_battery_status_t batMsg;

    // Заповнення даних BATTERY_STATUS
    batMsg.id = 0;  // Батарея 0
    batMsg.battery_function = MAV_BATTERY_FUNCTION_ALL;
    batMsg.type = MAV_BATTERY_TYPE_LION;
    batMsg.temperature = INT16_MAX;  // Не надається
    batMsg.voltages[0] = battery.voltage * 1000;  // V -> mV
    for (int i = 1; i < 10; i++) {
        batMsg.voltages[i] = UINT16_MAX;  // Невикористані
    }
    batMsg.current_battery = battery.current * 100;  // A -> 10mA
    batMsg.current_consumed = battery.capacity;      // mAh
    batMsg.energy_consumed = -1;                     // Не надається
    batMsg.battery_remaining = battery.percentage;
    batMsg.time_remaining = -1;                      // Не надається
    batMsg.charge_state = MAV_BATTERY_CHARGE_STATE_UNDEFINED;

    // Кодування MAVLink повідомлення
    mavlink_msg_battery_status_encode(systemId_, componentId_, &mavMsg, &batMsg);
    packMAVLinkMessage(mavMsg, msg);

    std::cout << "✅ Конвертовано BATTERY_STATUS: "
              << "voltage=" << battery.voltage << "V, "
              << "current=" << battery.current << "A, "
              << "capacity=" << battery.capacity << "mAh" << std::endl;

    sequenceNumber_++;
    return msg;
}