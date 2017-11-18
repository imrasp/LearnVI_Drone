#!/bin/bash

rsync -av /home/rasp/CLionProjects/LearnVI_Drone vgl-gpu1:/home/teera/workspace/rasp/

rm -rf build && \
mkdir build
