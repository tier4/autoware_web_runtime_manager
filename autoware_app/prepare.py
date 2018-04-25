#!/usr/bin/env python
# coding: utf-8

import psutil
import socket
from subprocess import call, Popen
from config.env import env


def kill_ros():
    call(["pkill", "-f", "ros"]);
    call(["pkill", "-f", "rosbag"]);


if __name__ == '__main__':
    print("kill ros")
    kill_ros()
