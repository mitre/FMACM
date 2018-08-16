#!/bin/bash

# before running this script, do the following:
# mkdir build
# cd build
# cmake -DCMAKE_BUILD_TYPE=Debug ..
# make -j4
# make run

# Assuming the above has happened already:
lcov -c -d ../build -o cov.info
genhtml cov.info -o coverage_out

# Your output will be HTML in coverage_out/
