#! /bin/bash

mkdir -p ./build
rm -rf ./build/*
cd ./build
cmake ..
make
cd -
mkdir -p outputs
cp -r ./build/src/*.so outputs
cp -r ./third_party/libs/*.so outputs