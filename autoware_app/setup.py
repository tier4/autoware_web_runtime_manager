#!/usr/bin/env python
# coding: utf-8

from subprocess import call
from os import getcwd, chdir, listdir
from argparse import ArgumentParser

parser = ArgumentParser(description='This script is ...')
parser.add_argument("-F", "--force", type=str, default="force", help="force")
args = parser.parse_args()

rootpath = getcwd()
print rootpath

# apt-get
call(["sudo", "apt-get", "update"])
call(["sudo", "apt-get", "install", "ros-kinetic-rosbridge-server"])

# pip
call(["pip", "install", "--upgrade", "pip"])
call(["sudo", "pip", "install", "-r", "requirements.txt"])
call(["sudo", "pip", "install", "git+https://github.com/strawlab/python-pcl"])

# env
chdir("{}/config".format(rootpath))
call(["ln", "-s", "sample.env", ".env"])
chdir("{}/ros/wrm_image_publisher/scripts/config".format(rootpath))
call(["ln", "-s", "sample.env", ".env"])
chdir("{}/ros/mqtt_bridge/scripts/config".format(rootpath))
call(["ln", "-s", "sample.env", ".env"])
