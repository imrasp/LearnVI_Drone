#!/bin/bash

rm -rf server_build
mkdir server_build
rsync -av /home/rasp/CLionProjects/LearnVI_Drone vgl-gpu1:/home/teera/workspace/rasp/
# TODO: build command here
ssh vgl-gpu1 '/home/teera/workspace/rasp/run_docker_without_gui.sh'
rsync -av vgl-gpu1:/home/teera/workspace/rasp/LearnVI_Drone/build/. /home/rasp/CLionProjects/LearnVI_Drone/server_build/
