Betaflight to MAVLink Converter
================================

Опис:
Програма для конвертації телеметрії Betaflight MSP у формат MAVLink.
Читає дані з польотного контролера через UART та відправляє MAVLink пакети через UDP.


Компіляція(cmake version 3.18):
-----------
cd ~/betaflight_mavlink_converter/cmake-build-debug
rm -rf *
cmake ..
make -j4

Використання:
-------------
./betaflight_mavlink_converter [опції]

Опції:
--device DEVICE    Послідовний порт (за замовчуванням: авто-визначення)
--baud BAUDRATE    Швидкість (за замовчуванням: 115200)
--udp              Увімкнути відправку MAVLink через UDP
--emulate          Увімкнути режим емуляції
--help             Показати довідку

Приклади:
---------
./betaflight_mavlink_converter
./betaflight_mavlink_converter --device /dev/ttyAMA0
./betaflight_mavlink_converter --device /dev/ttyAMA0 --baud 115200
./betaflight_mavlink_converter --device /dev/ttyAMA0 --baud 115200 --udp
./betaflight_mavlink_converter --emulate --udp

Підключення:
------------
Raspberry Pi        FC
=======================
TX (GPIO14)   ->    RX
RX (GPIO15)   ->    TX
GND           ->    GND

Формат виводу:
--------------
Розмір: 12 байт
HEX: 24 4D 3E 06 6C 64 00 32 00 C8 ...
[СИРІ ДАНІ] 12 байт отримано для парсингу
MSP команда: 108
ATTITUDE: roll=10°, pitch=5°, yaw=200°
[ATTITUDE] roll=10 pitch=5 yaw=200
MAVLINK ATTITUDE: FD 10 00 00 00 01 9E 1E 00 00 00 00 00 00 C2 B8 32 3E C2 B8 B2 3D F3 66 5F 40 ED B5
[MAVLINK] Створено ATTITUDE пакет

Розмір: 38 байт
HEX: 24 4D 3E 20 69 DC 05 DC 05 DC ...
[СИРІ ДАНІ] 38 байт отримано для парсингу
MSP команда: 105
RC_CHANNELS: 1500, 1500, 1500
[RC_CHANNELS] 1500 1500 1500 1500
MAVLINK RC_CHANNELS: FD 2A 00 00 01 01 9E 41 00 00 E9 03 00 00 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 FF FF FF FF 10 FF ED EC
[MAVLINK] Створено RC_CHANNELS пакет


UDP відправка:
--------------
При увімкненні опції --udp, пакети MAVLink відправляються на 127.0.0.1:14550
Для перегляду використовуйте QGroundControl або інший MAVLink-клієнт


Зупинка програми:
-----------------
Натисніть Ctrl+C для коректного завершення роботи