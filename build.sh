#!/bin/sh
mkdir -p build && cd build
cmake .. -DBUILD_EXAMPLES=OFF -DBUILD_PROJECT=ON
make
make test
make install
hciconfig
