#!/bin/bash

rm -rf /home/odroid/workspace/VIDrone/sample_data/datasetname
/home/odroid/workspace/VIDrone/build/LearnVI_Drone -m "LIVERECORD" -c "/home/odroid/workspace/VIDrone/config/system.yaml"