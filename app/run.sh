#!/bin/bash

python ./prepare.py
bash -l -c 'source ../../ros/devel/setup.bash; python ./router.py' &
sleep 2
# x-www-browser http://localhost:5000/
