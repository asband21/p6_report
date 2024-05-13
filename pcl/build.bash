#!/bin/bash

cd build
cmake ..
make clean
make
cd ..
./build/localization g
