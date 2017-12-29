#!/bin/bash

# set -x

rm -rf  /home/rasp/CLionProjects/LearnVI_Drone/build
mkdir  /home/rasp/CLionProjects/LearnVI_Drone/build
rsync -a --exclude 'LearnVI_Drone/download_result/' --exclude 'LearnVI_Drone/sample_data' --exclude 'LearnVI_Drone/build' --exclude 'LearnVI_Drone/dowloaded_result' --exclude 'LearnVI_Drone/Performance_output_result' --exclude 'LearnVI_Drone/Vocabulary' /home/rasp/CLionProjects/LearnVI_Drone vgl-gpu1:/home/teera/workspace/rasp/
ssh vgl-gpu1 '/home/teera/workspace/rasp/run_docker_without_gui.sh'
rsync -a vgl-gpu1:/home/teera/workspace/rasp/LearnVI_Drone/build/. /home/rasp/CLionProjects/LearnVI_Drone/build/
