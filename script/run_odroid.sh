#!/bin/bash

rm -rf ../sample_data/datasetname
./../build/LearnVI_Drone -m "MAVONLY" -f "datasetname" -t 100000 -b 921600 -d "/dev/ttySAC0" -g "DISABLE"
