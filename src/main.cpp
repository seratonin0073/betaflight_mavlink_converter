#include <iostream>
#include <string>
#include <memory>
#include <csignal>
#include <thread>
#include <chrono>
#include <functional>

#include "serial/serial_reader.h"
#include "msp/msp_parser.h"
#include "mavlink/mavlink_bridge.h"
#include "udp/udp_sender.h"

volatile sig_atomic_t stop = 0;

void signalHandler(int signum) {
    std::cout << "Отримано сигнал зупинки (" << signum << "), завершую роботу..." << std::endl;
    stop = 1;
}

void printHelp() {
    std::cout << "Використання: betaflight_mavlink_converter [options]" << std::endl;
    std::cout << "Опції:" << std::endl;
    std::cout << "  --device DEVICE    Послідовний порт" << std::endl;
    std::cout << "  --baud BAUDRATE    Швидкість" << std::endl;
    std::cout << "  --udp              Увімкнути відправку MAVLink через UDP" << std::endl;
    std::cout << "  --emulate          Увімкнути режим емуляції" << std::endl;
    std::cout << "  --help             Показати довідку" << std::endl;
    std::cout << std::endl;
    std::cout << "Приклади:" << std::endl;
    std::cout << "  ./betaflight_mavlink_converter --device /dev/ttyAMA0 --baud 115200 --udp" << std::endl;
    std::cout << "  ./betaflight_mavlink_converter --emulate --udp" << std::endl;
}

int main(int argc, char *argv[]) {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    std::string device = "";
    int baudrate = 115200;
    bool enableUdp = false;
    bool enableEmulation = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--device" && i + 1 < argc) {
            device = argv[++i];
            std::cout << "Вказано порт: " << device << std::endl;
        } else if (arg == "--baud" && i + 1 < argc) {
            try {
                baudrate = std::stoi(argv[++i]);
                std::cout << "Вказано швидкість: " << baudrate << " бод" << std::endl;
            } catch (const std::exception& e) {
                std::cerr << "Помилка: Невірна швидкість " << argv[i] << std::endl;
                return 1;
            }
        } else if (arg == "--udp") {
            enableUdp = true;
            std::cout << "UDP відправка увімкнена" << std::endl;
        } else if (arg == "--emulate") {
            enableEmulation = true;
            std::cout << "Режим емуляції увімкнено" << std::endl;
        } else if (arg == "--help") {
            printHelp();
            return 0;
        } else {
            std::cerr << "Невідомий аргумент: " << arg << std::endl;
            return 1;
        }
    }

    std::cout << "================================================" << std::endl;
    std::cout << "    Betaflight to MAVLink Converter" << std::endl;
    std::cout << "================================================" << std::endl;

    try {
        auto serialReader = std::make_shared<SerialReader>(device, baudrate);
        auto mspParser = std::make_shared<MSPParser>();
        auto mavlinkBridge = std::make_shared<MAVLinkBridge>();
        std::shared_ptr<UDPSender> udpSender = nullptr;

        if (enableUdp) {
            try {
                udpSender = std::make_shared<UDPSender>("127.0.0.1", 14550);
                if (udpSender->isConnected()) {
                    std::cout << "UDP відправка налаштована на 127.0.0.1:14550" << std::endl;
                } else {
                    std::cout << "UDP не підключено" << std::endl;
                    udpSender = nullptr;
                }
            } catch (const std::exception& e) {
                std::cerr << "Помилка ініціалізації UDP: " << e.what() << std::endl;
            }
        }

        serialReader->dataReceived = [&](const std::vector<uint8_t>& data) {
            std::cout << "[СИРІ ДАНІ] " << data.size() << " байт отримано для парсингу" << std::endl;
            mspParser->processData(data);
        };

        mspParser->attitudeReceived = [&](const AttitudeData& attitude) {
            std::cout << "[ATTITUDE] roll=" << attitude.roll << " pitch=" << attitude.pitch << " yaw=" << attitude.yaw << std::endl;

            auto mavMsg = mavlinkBridge->convertAttitude(attitude);
            std::cout << "[MAVLINK] Створено ATTITUDE пакет" << std::endl;

            if (udpSender && udpSender->isConnected()) {
                udpSender->sendMAVLinkMessage(mavMsg);
            }
        };

        mspParser->rcChannelsReceived = [&](const RCChannelsData& channels) {
            std::cout << "[RC_CHANNELS] ";
            for (int i = 0; i < 4; i++) {
                std::cout << channels.channels[i] << " ";
            }
            std::cout << std::endl;

            auto mavMsg = mavlinkBridge->convertRCChannels(channels);
            std::cout << "[MAVLINK] Створено RC_CHANNELS пакет" << std::endl;

            if (udpSender && udpSender->isConnected()) {
                udpSender->sendMAVLinkMessage(mavMsg);
            }
        };

        mspParser->batteryStateReceived = [&](const BatteryData& battery) {
            std::cout << "[BATTERY] voltage=" << battery.voltage << "V current=" << battery.current << "A capacity=" << battery.capacity << "mAh" << std::endl;

            auto statusMsg = mavlinkBridge->convertSystemStatus(battery);
            auto batteryMsg = mavlinkBridge->convertBatteryStatus(battery);

            std::cout << "[MAVLINK] Створено SYS_STATUS та BATTERY_STATUS пакети" << std::endl;

            if (udpSender && udpSender->isConnected()) {
                udpSender->sendMAVLinkMessage(statusMsg);
                udpSender->sendMAVLinkMessage(batteryMsg);
            }
        };

        if (enableEmulation) {
            std::cout << "Запуск емуляції даних..." << std::endl;
            serialReader->enableEmulation();
        } else {
            std::cout << "Підключення до порту " << device << "..." << std::endl;
            if (serialReader->startReading()) {
                std::cout << "Успішно підключено. Очікування даних..." << std::endl;
            } else {
                std::cerr << "Не вдалося відкрити порт" << std::endl;
                return 1;
            }
        }

        std::cout << "================================================" << std::endl;
        std::cout << "Робота запущена. Натисніть Ctrl+C для зупинки." << std::endl;
        std::cout << "================================================" << std::endl;

        while (!stop) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            static int counter = 0;
            counter++;
            if (counter % 20 == 0) {
                std::cout << "Програма активна" << std::endl;
            }
        }

        std::cout << "Завершення роботи..." << std::endl;
        serialReader->stopReading();

        std::cout << "Роботу завершено." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Помилка: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}