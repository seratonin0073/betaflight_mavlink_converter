cmake_minimum_required(VERSION 3.20)
project(betaflight_mavlink_converter VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(betaflight_mavlink_converter
        src/msp/msp_protocol.h
        src/msp/msp_parser.cpp
        src/msp/msp_parser.h
        src/mavlink/mavlink_bridge.cpp
        src/mavlink/mavlink_bridge.h
        src/serial/serial_reader.cpp
        src/serial/serial_reader.h
        src/udp/udp_sender.cpp
        src/udp/udp_sender.h
        src/main.cpp)