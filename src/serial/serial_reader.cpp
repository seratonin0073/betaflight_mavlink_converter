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
            std::cerr << "Не вдалося відкрити порт " << port_
                      << ": " << strerror(errno) << std::endl;
            return false;
        }

        struct termios tty;
        if (tcgetattr(fd_, &tty) != 0) {
            std::cerr << "Помилка tcgetattr: " << strerror(errno) << std::endl;
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
            default:
                std::cerr << "Невідома швидкість " << baudrate_
                          << ", використовую 115200" << std::endl;
                speed = B115200;
                break;
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
            std::cerr << "Помилка tcsetattr: " << strerror(errno) << std::endl;
            close();
            return false;
        }

        tcflush(fd_, TCIOFLUSH);

        std::cout << "Послідовний порт " << port_
                  << " відкрито на " << baudrate_ << " бод" << std::endl;
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
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                std::cerr << "Помилка читання з порту: " << strerror(errno) << std::endl;
            }
            return -1;
        }

        return bytesRead;
    }

    int writeData(const uint8_t* buffer, size_t size) {
        if (fd_ < 0) {
            std::cerr << "Порт не відкритий для запису" << std::endl;
            return -1;
        }

        ssize_t bytesWritten = ::write(fd_, buffer, size);
        if (bytesWritten < 0) {
            std::cerr << " Помилка запису в порт: " << strerror(errno) << std::endl;
            return -1;
        }

        if (tcdrain(fd_) != 0) {
            std::cerr << "Помилка tcdrain: " << strerror(errno) << std::endl;
        }

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
                std::cout << "Автовизначено порт: " << port_ << std::endl;
                break;
            }
        }

        if (port_.empty()) {
            port_ = "/dev/ttyAMA0";
            std::cout << "Порт не визначено, використовую " << port_ << std::endl;
        }
    }
}

SerialReader::~SerialReader() {
    stopReading();
}

bool SerialReader::startReading() {
    if (isReading()) {
        std::cout << "Читання вже запущено" << std::endl;
        return true;
    }

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

    std::cout << "Читання з порту зупинено" << std::endl;
}

bool SerialReader::isReading() const {
    return !stopRequested_ && serialImpl_ && serialImpl_->isOpen();
}

bool SerialReader::isConnected() const {
    return serialImpl_ && serialImpl_->isOpen();
}

bool SerialReader::writeData(const std::vector<uint8_t>& data) {
    if (!serialImpl_ || !serialImpl_->isOpen()) {
        std::cerr << "Порт не відкритий" << std::endl;
        return false;
    }

    if (data.empty()) {
        std::cerr << "Немає даних для відправки" << std::endl;
        return false;
    }

    std::lock_guard<std::mutex> lock(serialMutex_);

    // Використовуємо метод writeData з SerialImpl
    int bytesWritten = serialImpl_->writeData(data.data(), data.size());

    if (bytesWritten != static_cast<int>(data.size())) {
        std::cerr << "Відправлено не всі дані: " << bytesWritten
                  << "/" << data.size() << " байт" << std::endl;
        return false;
    }

    //std::cout << "Успішно відправлено " << bytesWritten << " байт" << std::endl;
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

    void SerialReader::sendNextRequest() {
        if (!serialImpl_ || !serialImpl_->isOpen() || waitingForResponse) {
            return;
        }

        const uint8_t commands[] = {108, 105, 130};
        const char* commandNames[] = {"ATTITUDE", "RC_CHANNELS", "BATTERY_STATE"};

        currentCommand = commands[requestCounter % 3];
        std::string commandName = commandNames[requestCounter % 3];

        std::cout << "ВІДПРАВКА ЗАПИТУ: " << commandName << " (CMD=" << static_cast<int>(currentCommand) << ")" << std::endl;

        if (sendMSPRequest(currentCommand)) {
            waitingForResponse = true;
            std::cout << "Запит " << commandName << " відправлено успішно" << std::endl;
        } else {
            std::cout << "Помилка відправки запиту " << commandName << std::endl;
        }

        requestCounter++;
    }


void SerialReader::readLoop() {
    uint8_t buffer[256];
    std::cout << "Потік читання запущено" << std::endl;

    int totalReadAttempts = 0;
    int successfulReads = 0;

    sendNextRequest();

    while (!stopRequested_ && serialImpl_ && serialImpl_->isOpen()) {
        totalReadAttempts++;

        int bytesRead = serialImpl_->read(buffer, sizeof(buffer));

        if (bytesRead > 0) {
            successfulReads++;
            std::vector<uint8_t> data(buffer, buffer + bytesRead);

            std::cout << "ОТРИМАНО ВІДПОВІДЬ: " << bytesRead << " байт" << std::endl;

            if (dataReceived && !data.empty()) {
                dataReceived(data);
            }

            waitingForResponse = false;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            sendNextRequest();

        } else if (bytesRead < 0) {
            std::cout << "ПОМИЛКА ЧИТАННЯ" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "Потік читання зупинено" << std::endl;
}

void SerialReader::requestLoop() {
    while (!stopRequested_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

void SerialReader::enableEmulation() {
    stopReading();
    stopRequested_ = false;

    std::cout << "=== РЕЖИМ ЕМУЛЯЦІЇ АКТИВОВАНО ===" << std::endl;
    std::cout << "Генерація тестових даних Betaflight MSP..." << std::endl;

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

        std::cout << "\n--- Емуляція " << messageType << " ---" << std::endl;
        std::cout << "Розмір: " << data.size() << " байт" << std::endl;
        std::cout << "HEX: ";
        for (size_t i = 0; i < std::min(data.size(), size_t(10)); i++) {
            printf("%02X ", data[i]);
        }
        if (data.size() > 10) std::cout << "...";
        std::cout << std::endl;

        if (dataReceived && !data.empty()) {
            dataReceived(data);
        }

        counter++;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    std::cout << "Потік емуляції даних зупинено" << std::endl;
}