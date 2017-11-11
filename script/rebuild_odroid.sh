#!/bin/bash

rm -rf /home/odroid/workspace/VIDrone/build && \
mkdir /home/odroid/workspace/VIDrone/build && \
cd /home/odroid/workspace/VIDrone/build && \
cmake .. && \
make
