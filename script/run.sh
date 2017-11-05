#!/bin/bash

rm -rf ../sample_data/datasetname
./../build/LearnVI_Drone -m "LIVERECORD" -f "datasetname" -t 100000 -b 921600
