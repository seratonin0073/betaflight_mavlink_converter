#include "msp_parser.h"
#include <iostream>
#include <cstring>

MSPParser::MSPParser()
    : inMessage_(false), expectedLength_(0) {
    buffer_.reserve(256);
}

void MSPParser::processData(const std::vector<uint8_t>& data) {
    for (uint8_t byte : data) {
        if (!inMessage_) {
            if (byte == '$') {
                inMessage_ = true;
                buffer_.clear();
                buffer_.push_back(byte);
            }
            continue;
        }

        buffer_.push_back(byte);

        if (buffer_.size() == 5) {
            if (buffer_[1] != 'M') {
                reset();
                continue;
            }

            uint8_t direction = buffer_[2];
            expectedLength_ = buffer_[3];
            uint8_t cmd = buffer_[4];

            expectedLength_ += 1;
        }

        if (buffer_.size() >= 5 && buffer_.size() == 5 + expectedLength_) {
            parseMSPMessage(buffer_);
            reset();
        }
    }
}

void MSPParser::reset() {
    inMessage_ = false;
    expectedLength_ = 0;
    buffer_.clear();
}

bool MSPParser::parseMSPMessage(const std::vector<uint8_t>& message) {
    if (message.size() < 6) {
        std::cerr << "Повідомлення занадто коротке: " << message.size() << " байт" << std::endl;
        return false;
    }

    // Перевіряємо заголовок
    if (message[0] != '$' || message[1] != 'M' || (message[2] != '<' && message[2] != '>')) {
        std::cerr << "Невірний заголовок MSP" << std::endl;
        return false;
    }

    uint8_t dataLength = message[3];
    uint8_t cmd = message[4];

    size_t expectedLength = 5 + dataLength + 1;

    if (message.size() != expectedLength) {
        std::cerr << "Невірна довжина повідомлення. Очікувалось: "
                  << expectedLength << ", отримано: " << message.size()
                  << " (dataLength=" << static_cast<int>(dataLength) << ")" << std::endl;
        return false;
    }

    uint8_t calculatedCrc = 0;
    for (size_t i = 3; i < message.size() - 1; i++) {
        calculatedCrc ^= message[i];
    }

    uint8_t receivedCrc = message.back();
    if (calculatedCrc != receivedCrc) {
        std::cerr << "MSP CRC помилка. Очікувалось: "
                  << static_cast<int>(calculatedCrc) << ", отримано: "
                  << static_cast<int>(receivedCrc) << std::endl;
        return false;
    }

    std::cout << "Успішно розпарсено MSP команду: " << static_cast<int>(cmd) << std::endl;

    switch (cmd) {
        case 108: {
            if (dataLength >= 6) {
                AttitudeData attitude;
                attitude.roll = static_cast<int16_t>(message[5] | (message[6] << 8)) / 10.0f;
                attitude.pitch = static_cast<int16_t>(message[7] | (message[8] << 8)) / 10.0f;
                attitude.yaw = static_cast<int16_t>(message[9] | (message[10] << 8));

                std::cout << "Розпарсено ATTITUDE: roll=" << attitude.roll
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

                std::cout << "Розпарсено RC_CHANNELS: " << channels.channels[0]
                          << ", " << channels.channels[1] << ", " << channels.channels[2] << ", ..." << std::endl;

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

                std::cout << "Розпарсено BATTERY: voltage=" << battery.voltage
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