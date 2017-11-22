#!/bin/bash

rm -rf /home/odroid/workspace/VIDrone/sample_data/datasetname
/home/odroid/workspace/VIDrone/build/LearnVI_Drone -m "MAVONLY" -f "datasetname" -t 100000 -b 921600 -d "/dev/ttyUSB0" -g "DISABLE"

