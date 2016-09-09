#!/usr/bin/env bash

set -e

../externals/generate-arduino-makefile/generate-arduino-makefile.py \
    -p teensy31 \
    -s . \
    -s ../firmware/devices/EEPROM/ \
    -l Wire \
    $@

cd build-teensy31/
make upload
