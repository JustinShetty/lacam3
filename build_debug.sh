#!/bin/bash
cmake -DCMAKE_BUILD_TYPE=Debug -B build_debug
make -j4 -C build_debug
