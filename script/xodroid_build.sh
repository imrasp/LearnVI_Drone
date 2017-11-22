#!/bin/bash

rm -rf  /home/rasp/CLionProjects/LearnVI_Drone/odroid_build
mkdir  /home/rasp/CLionProjects/LearnVI_Drone/odroid_build
rsync -a --exclude 'LearnVI_Drone/sample_data' --exclude 'LearnVI_Drone/build' --exclude 'LearnVI_Drone/dowloaded_result' --exclude 'LearnVI_Drone/Performance_output_result' --exclude 'LearnVI_Drone/Vocabulary' /home/rasp/CLionProjects/LearnVI_Drone vgl-gpu1:/home/teera/workspace/rasp/
ssh vgl-gpu1 'cp /home/teera/workspace/rasp/LearnVI_Drone/CMakeLists.txt.xcompile /home/teera/workspace/rasp/LearnVI_Drone/CMakeLists.txt'
ssh vgl-gpu1 'cp /home/teera/workspace/rasp/libDBoW2.so /home/teera/workspace/rasp/LearnVI_Drone/Thirdparty/DBoW2/lib/'
ssh vgl-gpu1 'cp /home/teera/workspace/rasp/libg2o.so /home/teera/workspace/rasp/LearnVI_Drone/Thirdparty/g2o/lib/'
ssh vgl-gpu1 '/home/teera/workspace/rasp/run_docker_odroidx.sh'
rsync -a vgl-gpu1:/home/teera/workspace/rasp/LearnVI_Drone/build/. /home/rasp/CLionProjects/LearnVI_Drone/odroid_build/ && \
rsync -a /home/rasp/CLionProjects/LearnVI_Drone/odroid_build/. odroid@10.42.0.1:/home/odroid/workspace/VIDrone/build/
# rsync -av ../odroid_build/. odroid@192.41.170.154:/home/odroid/workspace/VIDrone/build/
