#!/bin/bash

echo "Збірка з кореневої директорії..."

g++ -std=c++11 -I. -O2 \
    *.cpp \
    -o bfmavconverter \
    -lpthread

if [ -f "bfmavconverter" ]; then
    echo "Успішно!"
    chmod +x bfmavconverter
else
    echo "Не вдалося. Файли .cpp:"
    ls *.cpp
fi