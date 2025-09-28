#include "udp_sender.h"
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

class UDPSenderImpl {
public:
    UDPSenderImpl(const std::string& host, int port)
        : socket_(-1), host_(host), port_(port) {}

    bool connect() {
        socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_ < 0) {
            return false;
        }

        memset(&serverAddr_, 0, sizeof(serverAddr_));
        serverAddr_.sin_family = AF_INET;
        serverAddr_.sin_port = htons(port_);

        if (inet_pton(AF_INET, host_.c_str(), &serverAddr_.sin_addr) <= 0) {
            close();
            return false;
        }

        return true;
    }

    void close() {
        if (socket_ >= 0) {
            ::close(socket_);
            socket_ = -1;
        }
    }

    bool send(const uint8_t* data, size_t length) {
        if (socket_ < 0) return false;

        ssize_t sent = sendto(socket_, data, length, 0,
                             (struct sockaddr*)&serverAddr_, sizeof(serverAddr_));
        return sent == static_cast<ssize_t>(length);
    }

    bool isConnected() const {
        return socket_ >= 0;
    }

private:
    int socket_;
    std::string host_;
    int port_;
    struct sockaddr_in serverAddr_;
};

UDPSender::UDPSender(const std::string& host, int port) {
    socketImpl_ = new UDPSenderImpl(host, port);
    bool connected = static_cast<UDPSenderImpl*>(socketImpl_)->connect();
    if (!connected) {
        std::cout << "Помилка підключення UDP" << std::endl;
    }
}

UDPSender::~UDPSender() {
    static_cast<UDPSenderImpl*>(socketImpl_)->close();
    delete static_cast<UDPSenderImpl*>(socketImpl_);
}

bool UDPSender::isConnected() const {
    return static_cast<UDPSenderImpl*>(socketImpl_)->isConnected();
}

void UDPSender::sendMAVLinkMessage(const MAVLinkMessage& message) {
    if (!isConnected()) {
        std::cout << "UDP не підключено" << std::endl;
        return;
    }

    bool success = static_cast<UDPSenderImpl*>(socketImpl_)->send(message.data.data(), message.data.size());
    if (success) {
        std::cout << "Відправлено MAVLink через UDP: " << message.data.size() << " байт" << std::endl;
    } else {
        std::cout << "Помилка відправки MAVLink через UDP" << std::endl;
    }
}