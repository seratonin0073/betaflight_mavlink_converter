#!/bin/bash

echo "Збірка bfmavconverter..."

# Компіляція всіх .cpp файлів
g++ -std=c++11 -I. -O2 \
    src/*.cpp \
    -o bfmavconverter \
    -lpthread

if [ -f "bfmavconverter" ]; then
    echo "bfmavconverter успішно зібрано!"
    chmod +x bfmavconverter
    echo "Розташування: $(pwd)/bfmavconverter"
else
    echo "Помилка компіляції!"
fi