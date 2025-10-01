#include "serial_reader.h"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <system_error>
#include <chrono>
#include <cstring>
#include <algorithm>

class SerialImpl {
public:
    SerialImpl(const std::string& port, int baudrate)
        : fd_(-1), port_(port), baudrate_(baudrate) {}

    ~SerialImpl() {
        close();
    }

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

        speed_t speed = B115200;
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
            default: speed = B115200; break;
        }

        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1;
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            close();
            return false;
        }

        tcflush(fd_, TCIOFLUSH);
        return true;
    }

    void close() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    int read(uint8_t* buffer, size_t size) {
        if (fd_ < 0) return -1;

        ssize_t bytesRead = ::read(fd_, buffer, size);
        if (bytesRead < 0) {
            return -1;
        }

        return bytesRead;
    }

    int writeData(const uint8_t* buffer, size_t size) {
        if (fd_ < 0) {
            return -1;
        }

        ssize_t bytesWritten = ::write(fd_, buffer, size);
        if (bytesWritten < 0) {
            return -1;
        }

        tcdrain(fd_);
        return bytesWritten;
    }

    bool isOpen() const {
        return fd_ >= 0;
    }

    int getFd() const { return fd_; }

private:
    int fd_;
    std::string port_;
    int baudrate_;
};

SerialReader::SerialReader(const std::string& port, int baudrate)
    : port_(port), baudrate_(baudrate), stopRequested_(false) {

    if (port_.empty()) {
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
            port_ = "/dev/ttyAMA0";
        }
    }
}

SerialReader::~SerialReader() {
    stopReading();
}

bool SerialReader::startReading() {
    if (isReading()) {
        return true;
    }

    serialImpl_ = std::make_unique<SerialImpl>(port_, baudrate_);
    if (!serialImpl_->open()) {
        serialImpl_.reset();
        return false;
    }

    stopRequested_ = false;

    readThread_ = std::thread([this]() {
        readLoop();
    });

    requestThread_ = std::thread([this]() {
        requestLoop();
    });

    std::cout << "Запущено читання з порту: " << port_ << std::endl;
    return true;
}

void SerialReader::stopReading() {
    stopRequested_ = true;

    if (requestThread_.joinable()) {
        requestThread_.join();
    }

    if (readThread_.joinable()) {
        readThread_.join();
    }

    if (serialImpl_) {
        serialImpl_->close();
        serialImpl_.reset();
    }
}

bool SerialReader::isReading() const {
    return !stopRequested_ && serialImpl_ && serialImpl_->isOpen();
}

bool SerialReader::isConnected() const {
    return serialImpl_ && serialImpl_->isOpen();
}

bool SerialReader::writeData(const std::vector<uint8_t>& data) {
    if (!serialImpl_ || !serialImpl_->isOpen()) {
        return false;
    }

    if (data.empty()) {
        return false;
    }

    std::lock_guard<std::mutex> lock(serialMutex_);

    int bytesWritten = serialImpl_->writeData(data.data(), data.size());
    if (bytesWritten != static_cast<int>(data.size())) {
        return false;
    }

    return true;
}

std::vector<uint8_t> SerialReader::createMSPRequest(uint8_t command) {
    std::vector<uint8_t> request;
    request.push_back('$');
    request.push_back('M');
    request.push_back('<');
    request.push_back(0);
    request.push_back(command);

    uint8_t crc = 0;
    for (size_t i = 3; i < request.size(); i++) {
        crc ^= request[i];
    }
    request.push_back(crc);

    return request;
}

bool SerialReader::sendMSPRequest(uint8_t command) {
    auto request = createMSPRequest(command);
    return writeData(request);
}

bool SerialReader::waitForResponse(int timeoutMs) {
    auto startTime = std::chrono::steady_clock::now();
    uint8_t buffer[256];

    while (!stopRequested_) {
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime);

        if (elapsed.count() > timeoutMs) {
            return false;
        }

        int bytesRead = serialImpl_->read(buffer, sizeof(buffer));
        if (bytesRead > 0) {
            return true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return false;
}

void SerialReader::readLoop() {
    uint8_t buffer[256];
    std::cout << "Потік читання запущено" << std::endl;

    while (!stopRequested_ && serialImpl_ && serialImpl_->isOpen()) {
        int bytesRead = serialImpl_->read(buffer, sizeof(buffer));

        if (bytesRead > 0) {
            std::vector<uint8_t> data(buffer, buffer + bytesRead);

            std::cout << "ОТРИМАНО ДАНІ: " << bytesRead << " байт" << std::endl;
            std::cout << "HEX: ";
            for (int i = 0; i < bytesRead; i++) {
                printf("%02X ", buffer[i]);
            }
            std::cout << std::endl;

            if (dataReceived && !data.empty()) {
                dataReceived(data);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "Потік читання зупинено" << std::endl;
}

void SerialReader::requestLoop() {
    std::cout << "Потік запитів MSP запущено" << std::endl;

    const uint8_t MSP_ATTITUDE = 108;
    const uint8_t MSP_RC = 105;
    const uint8_t MSP_BATTERY_STATE = 130;

    while (!stopRequested_) {
        if (!serialImpl_ || !serialImpl_->isOpen()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        uint8_t command;
        std::string commandName;

        switch (requestCounter % 3) {
            case 0:
                command = MSP_ATTITUDE;
                commandName = "ATTITUDE";
                break;
            case 1:
                command = MSP_RC;
                commandName = "RC_CHANNELS";
                break;
            case 2:
                command = MSP_BATTERY_STATE;
                commandName = "BATTERY_STATE";
                break;
        }

        std::cout << "========================================" << std::endl;
        std::cout << "ВІДПРАВКА ЗАПИТУ: " << commandName << " (CMD=" << static_cast<int>(command) << ")" << std::endl;

        if (sendMSPRequest(command)) {
            std::cout << "Запит відправлено успішно" << std::endl;
        }

        requestCounter++;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    std::cout << "Потік запитів MSP зупинено" << std::endl;
}

void SerialReader::enableEmulation() {
    stopReading();
    stopRequested_ = false;

    std::cout << "Запуск емуляції даних..." << std::endl;

    readThread_ = std::thread([this]() {
        emulateData();
    });
}

void SerialReader::emulateData() {
    std::cout << "Потік емуляції даних запущено" << std::endl;

    int counter = 0;
    while (!stopRequested_) {
        std::vector<uint8_t> data;
        std::string messageType;

        if (counter % 3 == 0) {
            messageType = "ATTITUDE";
            data = {'$', 'M', '>', 6, 108};
            data.push_back(0x64); data.push_back(0x00);
            data.push_back(0x32); data.push_back(0x00);
            data.push_back(0xC8); data.push_back(0x00);
        }
        else if (counter % 3 == 1) {
            messageType = "RC_CHANNELS";
            data = {'$', 'M', '>', 32, 105};
            for (int i = 0; i < 16; i++) {
                data.push_back(0xDC);
                data.push_back(0x05);
            }
        }
        else {
            messageType = "BATTERY";
            data = {'$', 'M', '>', 8, 130};
            data.push_back(0x90); data.push_back(0x41);
            data.push_back(0xFA); data.push_back(0x00);
            data.push_back(0xB0); data.push_back(0x04);
            data.push_back(0x00); data.push_back(0x00);
            data.push_back(0x50);
        }

        uint8_t crc = 0;
        for (size_t i = 3; i < data.size(); i++) {
            crc ^= data[i];
        }
        data.push_back(crc);

        std::cout << "Емуляція " << messageType << std::endl;
        std::cout << "Розмір: " << data.size() << " байт" << std::endl;

        if (dataReceived && !data.empty()) {
            dataReceived(data);
        }

        counter++;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    std::cout << "Потік емуляції даних зупинено" << std::endl;
}