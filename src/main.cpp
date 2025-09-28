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
    std::cout << "Використання: bfmavconverter [options]" << std::endl;
    std::cout << "Опції:" << std::endl;
    std::cout << "  --device DEVICE    Послідовний порт (за замовчуванням: авто-визначення)" << std::endl;
    std::cout << "  --baud BAUDRATE    Швидкість (за замовчуванням: 115200)" << std::endl;
    std::cout << "  --udp              Увімкнути відправку MAVLink через UDP" << std::endl;
    std::cout << "  --emulate          Увімкнути режим емуляції (без підключення до порту)" << std::endl;
    std::cout << "  --help             Показати цю довідку" << std::endl;
    std::cout << std::endl;
    std::cout << "Приклади:" << std::endl;
    std::cout << "  ./betaflight_mavlink_converter --device /dev/ttyAMA0 --baud 115200 --udp" << std::endl;
    std::cout << "  ./bfmavconverter --emulate --udp  (режим емуляції)" << std::endl;
    std::cout << "  ./bfmavconverter --help" << std::endl;
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
            std::cerr << "Використовуйте --help для довідки" << std::endl;
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
                std::cout << "UDP відправка налаштована на 127.0.0.1:14550" << std::endl;
            } catch (const std::exception& e) {
                std::cerr << "Помилка ініціалізації UDP: " << e.what() << std::endl;
                std::cout << "Продовжую без UDP..." << std::endl;
            }
        }

        serialReader->dataReceived = [&](const std::vector<uint8_t>& data) {
            mspParser->processData(data);
        };

        mspParser->attitudeReceived = [&](const AttitudeData& attitude) {
            std::cout << "[MSP] Отримано ATTITUDE: roll=" << attitude.roll
                      << "°, pitch=" << attitude.pitch
                      << "°, yaw=" << attitude.yaw << "°" << std::endl;

            auto mavMsg = mavlinkBridge->convertAttitude(attitude);
            std::cout << "[MAVLink] Згенеровано ATTITUDE (ID=" << static_cast<int>(mavMsg.msgid)
                      << ", len=" << static_cast<int>(mavMsg.len) << " байт)" << std::endl;

            if (udpSender && udpSender->isConnected()) {
                udpSender->sendMAVLinkMessage(mavMsg);
            }
        };

        mspParser->rcChannelsReceived = [&](const RCChannelsData& channels) {
            std::cout << "[MSP] Отримано RC_CHANNELS: ";
            for (int i = 0; i < 4; i++) {
                std::cout << channels.channels[i];
                if (i < 3) std::cout << ", ";
            }
            std::cout << ", ..." << std::endl;

            auto mavMsg = mavlinkBridge->convertRCChannels(channels);
            std::cout << "[MAVLink] Згенеровано RC_CHANNELS (ID=" << static_cast<int>(mavMsg.msgid)
                      << ", len=" << static_cast<int>(mavMsg.len) << " байт)" << std::endl;

            if (udpSender && udpSender->isConnected()) {
                udpSender->sendMAVLinkMessage(mavMsg);
            }
        };

        mspParser->batteryStateReceived = [&](const BatteryData& battery) {
            std::cout << "[MSP] Отримано BATTERY: voltage=" << battery.voltage
                      << "V, current=" << battery.current
                      << "A, capacity=" << battery.capacity
                      << "mAh (" << static_cast<int>(battery.percentage) << "%)" << std::endl;

            auto statusMsg = mavlinkBridge->convertSystemStatus(battery);
            auto batteryMsg = mavlinkBridge->convertBatteryStatus(battery);

            std::cout << "[MAVLink] Згенеровано SYS_STATUS (ID=" << static_cast<int>(statusMsg.msgid)
                      << ", len=" << static_cast<int>(statusMsg.len) << " байт)" << std::endl;
            std::cout << "[MAVLink] Згенеровано BATTERY_STATUS (ID=" << static_cast<int>(batteryMsg.msgid)
                      << ", len=" << static_cast<int>(batteryMsg.len) << " байт)" << std::endl;

            if (udpSender && udpSender->isConnected()) {
                udpSender->sendMAVLinkMessage(statusMsg);
                udpSender->sendMAVLinkMessage(batteryMsg);
            }
        };

        if (enableEmulation) {
            std::cout << "Запуск емуляції даних..." << std::endl;
            serialReader->enableEmulation();
        } else {
            std::cout << "Спроба підключення до порту " << device << "..." << std::endl;
            if (serialReader->startReading()) {
                std::cout << "Успішно підключено до порту. Очікування даних..." << std::endl;
            } else {
                std::cerr << "Не вдалося відкрити порт. Перевірте:" << std::endl;
                std::cerr << "   - Чи підключено політний контролер" << std::endl;
                std::cerr << "   - Чи правильний порт (" << device << ")" << std::endl;
                std::cerr << "   - Чи ввімкнено MSP в Betaflight Configurator" << std::endl;
                std::cerr << "   - Чи є права доступу (sudo usermod -a -G dialout $USER)" << std::endl;
                std::cerr << "   - Чи не зайнятий порт іншою програмою" << std::endl;
                std::cerr << std::endl;
                std::cerr << "Для тестування використовуйте: ./bfmavconverter --emulate" << std::endl;
                return 1;
            }
        }

        std::cout << "================================================" << std::endl;
        std::cout << "Робота запущена. Натисніть Ctrl+C для зупинки." << std::endl;
        std::cout << "================================================" << std::endl;

        while (!stop) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::cout << "Завершення роботи..." << std::endl;
        serialReader->stopReading();

        std::cout << "Роботу завершено." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Критична помилка: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}