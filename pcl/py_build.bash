#!/bin/bash

cd build
cmake .. -DBUILD_PYTHON_BINDING=ON
make clean
make
cd ..
python3 py_localization.py

