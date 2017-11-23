#!/bin/bash

ODROID_IP=10.42.0.1
# ODROID_IP=192.42.170.154

rm -rf  /home/rasp/CLionProjects/LearnVI_Drone/odroid_build
mkdir  /home/rasp/CLionProjects/LearnVI_Drone/odroid_build
rsync -a --exclude 'LearnVI_Drone/sample_data' --exclude 'LearnVI_Drone/build' --exclude 'LearnVI_Drone/dowloaded_result' --exclude 'LearnVI_Drone/Performance_output_result' --exclude 'LearnVI_Drone/Vocabulary' /home/rasp/CLionProjects/LearnVI_Drone vgl-gpu1:/home/teera/workspace/rasp/
ssh vgl-gpu1 'cp /home/teera/workspace/rasp/LearnVI_Drone/CMakeLists.txt.xcompile /home/teera/workspace/rasp/LearnVI_Drone/CMakeLists.txt'
rm -rf /tmp/Thirdparty
ssh vgl-gpu1 'rm -rf /home/odroid/workspace/VIDrone/Thirdparty/g2o/build'
ssh vgl-gpu1 'rm -rf /home/odroid/workspace/VIDrone/Thirdparty/DBoW2/build'
rsync -a odroid@$ODROID_IP:/home/odroid/workspace/VIDrone/Thirdparty /tmp/
rsync -a /tmp/Thirdparty vgl-gpu1:/home/teera/workspace/rasp/LearnVI_Drone/
ssh vgl-gpu1 '/home/teera/workspace/rasp/run_docker_odroidx_fast.sh'
rsync -a vgl-gpu1:/home/teera/workspace/rasp/LearnVI_Drone/build/. /home/rasp/CLionProjects/LearnVI_Drone/odroid_build/ && \
rsync -a /home/rasp/CLionProjects/LearnVI_Drone/odroid_build/. odroid@$ODROID_IP:/home/odroid/workspace/VIDrone/build/
