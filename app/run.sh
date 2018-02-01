#!/bin/bash

python ./prepare.py
bash -l -c 'python ./router.py' &
sleep 2
# x-www-browser http://localhost:5000/
