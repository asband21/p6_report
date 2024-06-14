#!/bin/bash

cd build
cmake .. -DBUILD_PYTHON_BINDING=ON -USE_BOOST=ON
make clean
make -j 28
cd ..
python3 py_localization_server.py

