#include "msp_parser.h"
#include <iostream>
#include <cstring>
#include <algorithm>

MSPParser::MSPParser() {
    rxBuffer_.reserve(BUFFER_SIZE);
}

void MSPParser::processData(const std::vector<uint8_t>& data) {
    // Лог усіх байтів що приходять
    std::cout << "[RX] ";
    for (auto b : data) {
        printf("%02X ", b);
    }
    std::cout << std::endl;

    rxBuffer_.insert(rxBuffer_.end(), data.begin(), data.end());

    while (true) {
        auto it = std::find(rxBuffer_.begin(), rxBuffer_.end(), '$');
        if (it == rxBuffer_.end()) break;

        // мінімальний розмір пакета "$M><len><cmd><crc>"
        if (rxBuffer_.end() - it < 6) break;

        if (*(it + 1) != 'M' || (*(it + 2) != '>')) {
            std::cout << "Bad header after '$': "
                      << std::hex << (int)*(it+1) << " " << (int)*(it+2) << std::dec
                      << std::endl;
            rxBuffer_.erase(rxBuffer_.begin(), it + 1);
            continue;
        }

        uint8_t dataLength = *(it + 3);
        size_t packetLength = 5 + dataLength + 1;

        if (rxBuffer_.end() - it < packetLength) {
            break; // чекаємо ще байти
        }

        std::vector<uint8_t> packet(it, it + packetLength);

        if (validateCRC(packet)) {
            parseMSPMessage(packet);
        } else {
            uint8_t calc = 0;
            for (size_t i = 3; i < packet.size() - 1; i++) calc ^= packet[i];
            std::cout << "MSP CRC помилка! Отримано="
                      << (int)packet.back()
                      << " Очікувалось=" << (int)calc << std::endl;
        }

        rxBuffer_.erase(rxBuffer_.begin(), it + packetLength);
    }

    if (rxBuffer_.size() > BUFFER_SIZE) {
        rxBuffer_.erase(rxBuffer_.begin(), rxBuffer_.end() - BUFFER_SIZE/2);
    }
}


bool MSPParser::validateCRC(const std::vector<uint8_t>& message) {
    if (message.size() < 6) return false;

    uint8_t calculatedCrc = 0;
    for (size_t i = 3; i < message.size() - 1; i++) {
        calculatedCrc ^= message[i];
    }

    return calculatedCrc == message.back();
}

bool MSPParser::parseMSPMessage(const std::vector<uint8_t>& message) {
    uint8_t dataLength = message[3];
    uint8_t cmd = message[4];

    std::cout << "MSP команда: " << static_cast<int>(cmd) << std::endl;

    switch (cmd) {
        case 108: {
            if (dataLength >= 6) {
                AttitudeData attitude;
                attitude.roll = static_cast<int16_t>(message[5] | (message[6] << 8)) / 10.0f;
                attitude.pitch = static_cast<int16_t>(message[7] | (message[8] << 8)) / 10.0f;
                attitude.yaw = static_cast<int16_t>(message[9] | (message[10] << 8));

                std::cout << "ATTITUDE: roll=" << attitude.roll
                          << "°, pitch=" << attitude.pitch << "°, yaw=" << attitude.yaw << "°" << std::endl;

                if (attitudeReceived) {
                    attitudeReceived(attitude);
                }
            }
            break;
        }

        case 105: {
            if (dataLength >= 32) {
                RCChannelsData channels;
                for (int i = 0; i < 16; i++) {
                    channels.channels[i] = message[5 + i * 2] | (message[6 + i * 2] << 8);
                }

                std::cout << "RC_CHANNELS: ";
                for (int i = 0; i < 4; i++) {
                    std::cout << channels.channels[i];
                    if (i < 3) std::cout << ", ";
                }
                std::cout << std::endl;

                if (rcChannelsReceived) {
                    rcChannelsReceived(channels);
                }
            }
            break;
        }

        case 130: {
            if (dataLength >= 7) {
                BatteryData battery;
                battery.voltage = static_cast<int16_t>(message[5] | (message[6] << 8)) / 100.0f;
                battery.current = static_cast<int16_t>(message[7] | (message[8] << 8)) / 100.0f;
                battery.capacity = message[9] | (message[10] << 8) | (message[11] << 16);
                battery.percentage = message[12];

                std::cout << "BATTERY: voltage=" << battery.voltage
                          << "V, current=" << battery.current << "A, capacity="
                          << battery.capacity << "mAh (" << static_cast<int>(battery.percentage) << "%)" << std::endl;

                if (batteryStateReceived) {
                    batteryStateReceived(battery);
                }
            }
            break;
        }

        default:
            std::cout << "Невідома MSP команда: " << static_cast<int>(cmd) << std::endl;
            break;
    }

    return true;
}

void MSPParser::reset() {
    rxBuffer_.clear();
}