#!/bin/bash

python ./prepare.py
source ~/Autoware/ros/devel/setup.bash
python ./router.py
sleep 2
# x-www-browser http://localhost:5000/
