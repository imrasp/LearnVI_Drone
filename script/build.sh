#!/bin/bash

echo "Configuring and building Thirdparty/DBoW2 ..."

cd ../Thirdparty/DBoW2
rm -rf build
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."
rm -rf build
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

cd ../../../

rm -rf build && \
mkdir build && \
cd build && \
cmake .. && \
make -j4
