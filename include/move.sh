#!/bin/bash
set -e

# header file first, in case it's header-only
git mv include/aaesim/$1.h include/public/$1.h
echo sed -i "'s|\\\"aaesim/$1.h\\\"|\\\"public/$1.h\\\"|g'" \$1 >> include/subs.sh

# cpp file
git mv AAESim/$1.cpp Public/$1.cpp
echo "        $1.cpp" >> Public/list
grep -v $1 AAESim/CMakeLists.txt >AAESim/CMakeLists.1
mv AAESim/CMakeLists.1 AAESim/CMakeLists.txt
