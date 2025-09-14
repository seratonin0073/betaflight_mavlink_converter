#pragma once

#include <string>
#include "../mavlink//mavlink_bridge.h"

class UDPSender {
public:
    UDPSender(const std::string& host, int port);
    ~UDPSender();

    bool isConnected() const;
    void sendMAVLinkMessage(const MAVLinkMessage& message);

private:
    std::string host_;
    int port_;
    void* socketImpl_;
};