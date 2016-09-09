#!/usr/bin/env bash

set -e

mkdir -p build-feather/

../externals/generate-arduino-makefile/generate-arduino-makefile.py \
    --board adafruit_feather_m0 \
    --vendor adafruit \
    --arch samd \
    -s . \
    -B build-feather \
    -L /usr/share/arduino/libraries/ \
    -L ../externals/i2cdevlib/Arduino/ \
    -L ~/.arduino/sketches/libraries/ \
    -l Wire \
    -l SPI \
    -l WiFi101 \
    -o build-feather/Makefile \
    $@

cd build-feather/
make upload
