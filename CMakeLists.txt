cmake_minimum_required(VERSION 4.0)
project(betaflight_mavlink_converter)


set(CMAKE_CXX_STANDARD 20)

add_executable(betaflight_mavlink_converter src/main.cpp
        src/msp/msp_protocol.h
        src/msp/msp_parser.cpp
        src/msp/msp_parser.h
        src/mavlink/mavlink_bridge.cpp
        src/mavlink/mavlink_bridge.h
        src/serial/serial_reader.cpp
        src/serial/serial_reader.h
        src/udp/udp_sender.cpp
        src/udp/udp_sender.h)
