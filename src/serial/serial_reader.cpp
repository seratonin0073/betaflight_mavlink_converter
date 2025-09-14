#include "serial_reader.h"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <system_error>

class SerialImpl {
public:
    SerialImpl(const std::string& port, int baudrate)
        : fd_(-1), port_(port), baudrate_(baudrate) {}

    bool open() {
        fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) {
            return false;
        }

        struct termios tty;
        if (tcgetattr(fd_, &tty) != 0) {
            close();
            return false;
        }

        // Встановлення швидкості
        speed_t speed;
        switch (baudrate_) {
            case 9600: speed = B9600; break;
            case 19200: speed = B19200; break;
            case 38400: speed = B38400; break;
            case 57600: speed = B57600; break;
            case 115200: speed = B115200; break;
            case 230400: speed = B230400; break;
            case 460800: speed = B460800; break;
            case 500000: speed = B500000; break;
            case 921600: speed = B921600; break;
            case 1000000: speed = B1000000; break;
            case 1152000: speed = B1152000; break;
            case 1500000: speed = B1500000; break;
            case 2000000: speed = B2000000; break;
            case 2500000: speed = B2500000; break;
            case 3000000: speed = B3000000; break;
            case 3500000: speed = B3500000; break;
            case 4000000: speed = B4000000; break;
            default: speed = B115200; break;
        }

        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 5;

        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            close();
            return false;
        }

        return true;
    }

    void close() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    int read(uint8_t* buffer, size_t size) {
        return ::read(fd_, buffer, size);
    }

    bool isOpen() const {
        return fd_ >= 0;
    }

private:
    int fd_;
    std::string port_;
    int baudrate_;
};

SerialReader::SerialReader(const std::string& port, int baudrate)
    : port_(port), baudrate_(baudrate), stopRequested_(false) {
    if (port_.empty()) {
        // Автовизначення порту
        const char* possible_ports[] = {
            "/dev/ttyAMA0", "/dev/ttyS0", "/dev/serial0",
            "/dev/ttyUSB0", "/dev/ttyACM0", nullptr
        };

        for (int i = 0; possible_ports[i] != nullptr; i++) {
            if (access(possible_ports[i], F_OK) != -1) {
                port_ = possible_ports[i];
                break;
            }
        }

        if (port_.empty()) {
            port_ = "/dev/ttyUSB0";
        }
    }
}

SerialReader::~SerialReader() {
    stopReading();
}

bool SerialReader::startReading() {
    serialImpl_ = std::make_unique<SerialImpl>(port_, baudrate_);
    if (!serialImpl_->open()) {
        std::cerr << "Не вдалося відкрити порт: " << port_ << std::endl;
        serialImpl_.reset();
        return false;
    }

    stopRequested_ = false;

    readThread_ = std::thread([this]() {
        readLoop();
    });

    std::cout << "Успішно підключено до порту: " << port_ << " на " << baudrate_ << " бод" << std::endl;
    return true;
}

void SerialReader::stopReading() {
    stopRequested_ = true;
    if (readThread_.joinable()) {
        readThread_.join();
    }

    if (serialImpl_) {
        serialImpl_->close();
        serialImpl_.reset();
    }
}

bool SerialReader::isReading() const {
    return !stopRequested_;
}

void SerialReader::enableEmulation() {
    stopReading();

    stopRequested_ = false;
    readThread_ = std::thread([this]() {
        emulateData();
    });

    std::cout << "=== РЕЖИМ ЕМУЛЯЦІЇ АКТИВОВАНО ===" << std::endl;
}

void SerialReader::readLoop() {
    uint8_t buffer[256];

    while (!stopRequested_ && serialImpl_ && serialImpl_->isOpen()) {
        int bytesRead = serialImpl_->read(buffer, sizeof(buffer));
        if (bytesRead > 0) {
            std::vector<uint8_t> data(buffer, buffer + bytesRead);
            if (dataReceived) {
                dataReceived(data);
            }
        } else if (bytesRead < 0) {
            std::cerr << "Помилка читання з порту" << std::endl;
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void SerialReader::emulateData() {
    std::cout << "=== РЕЖИМ ЕМУЛЯЦІЇ ===" << std::endl;
    std::cout << "Генерація тестових даних Betaflight MSP..." << std::endl;

    int counter = 0;
    while (!stopRequested_) {
        std::vector<uint8_t> data;
        std::string messageType;

        if (counter % 3 == 0) {
            messageType = "ATTITUDE";
            data = {'$', 'M', '<'};
            data.push_back(6);
            data.push_back(108);

            data.push_back(0x64); data.push_back(0x00); // roll = 100 (10.0°)
            data.push_back(0x32); data.push_back(0x00); // pitch = 50 (5.0°)
            data.push_back(0xC8); data.push_back(0x00); // yaw = 200 (200°)

            uint8_t crc = 0;
            for (size_t i = 3; i < data.size(); i++) {
                crc ^= data[i];
            }
            data.push_back(crc);
        }
        else if (counter % 3 == 1) {
            messageType = "RC_CHANNELS";
            data = {'$', 'M', '<'};
            data.push_back(32);
            data.push_back(105);

            // Дані (32 байти)
            for (int i = 0; i < 16; i++) {
                data.push_back(0xDC);
                data.push_back(0x05);
            }

            uint8_t crc = 0;
            for (size_t i = 3; i < data.size(); i++) {
                crc ^= data[i];
            }
            data.push_back(crc);
        }
        else {
            messageType = "BATTERY";
            data = {'$', 'M', '<'};
            data.push_back(8);
            data.push_back(130);

            data.push_back(0x90); data.push_back(0x41); // voltage = 16.8V
            data.push_back(0xFA); data.push_back(0x00); // current = 25.0A
            data.push_back(0xB0); data.push_back(0x04); data.push_back(0x00); // capacity = 1200mAh
            data.push_back(0x50);                       // percentage = 80%

            uint8_t crc = 0;
            for (size_t i = 3; i < data.size(); i++) {
                crc ^= data[i];
            }
            data.push_back(crc);
        }

        std::cout << "\n--- " << messageType << " повідомлення ---" << std::endl;
        std::cout << "Розмір: " << data.size() << " байт" << std::endl;
        std::cout << "HEX: ";
        for (size_t i = 0; i < data.size(); i++) {
            printf("%02X ", data[i]);
            //if (i == 5) std::cout << "| ";
        }
        std::cout << std::endl;

        if (dataReceived && !data.empty()) {
            std::cout << "Відправка емульованих даних..." << std::endl;
            dataReceived(data);
        }

        counter++;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}