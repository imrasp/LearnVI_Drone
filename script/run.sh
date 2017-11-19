#!/bin/bash

rm -rf ../sample_data/datasetname
./../server_build/LearnVI_Drone -m "LIVERECORD" -f "datasetname" -t 100000 -b 921600

#./build/LearnVI_Drone -m "LIVERECORD" -f "test" -t 100000 -b 921600 -d "/dev/ttySAC0"
