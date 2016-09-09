#!/usr/bin/env bash

set -e

../externals/generate-arduino-makefile/generate-arduino-makefile.py \
    -p feather-m0 \
    -s . \
    -s ../firmware/devices/EEPROM/ \
    -l Wire \
    $@

cd build-feather-m0/
make upload
