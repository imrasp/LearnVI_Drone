#!/bin/bash

rm -rf server_build
mkdir server_build
rsync -av --exclude 'LearnVI_Drone/sample_data' --exclude 'LearnVI_Drone/build' --exclude 'LearnVI_Drone/dowloaded_result' --exclude 'LearnVI_Drone/Performance_output_result' --exclude 'LearnVI_Drone/Vocabulary' /home/rasp/CLionProjects/LearnVI_Drone vgl-gpu1:/home/teera/workspace/rasp/
ssh vgl-gpu1 '/home/teera/workspace/rasp/run_docker_without_gui.sh'
rsync -av vgl-gpu1:/home/teera/workspace/rasp/LearnVI_Drone/build/. /home/rasp/CLionProjects/LearnVI_Drone/server_build/
