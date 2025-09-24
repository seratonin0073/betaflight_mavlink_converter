#!/bin/bash
echo "🔨 Збірка для Raspberry Pi..."

mkdir -p build
cd build

cmake -DARM=ON

make -j$(nproc)

if [ -f "bfmavconverter" ]; then
    echo "✅ Програму успішно зібрано для Raspberry Pi!"
    chmod +x
fi