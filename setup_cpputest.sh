#!/usr/bin/env bash

cd ..
git clone https://github.com/cpputest/cpputest.git
cd cpputest
./configure && make
export CPPUTEST_HOME=$(pwd)
cd ../debra
mkdir build
cd build
cmake ..
