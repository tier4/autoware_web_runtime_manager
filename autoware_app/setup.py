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

# pip
call(["pip", "install", "--upgrade", "pip"])
call(["sudo", "pip", "install", "-r", "requirements.txt"])
call(["sudo", "pip", "install", "git+https://github.com/strawlab/python-pcl"])

# res
"""
chdir("{}/controllers".format(rootpath))
file_names = listdir("./")

if args.force == "force":
    call(["rm", "-f",  "res.zip"])
    call(["rm", "-rf",  "res"])
    call(["wget", "https://autoware.blob.core.windows.net/web-ui-resouces/res.zip"])
    call(["unzip", "res.zip"])
elif "res" not in file_names and "res.zip" in file_names:
    call(["unzip",  "res.zip"])
elif "res.zip" not in file_names and "res" not in file_names:
    call(["wget", "https://autoware.blob.core.windows.net/web-ui-resouces/res.zip"])
    call(["unzip", "res.zip"])

call(["rm", "-f", "res.zip"])
"""

# env
chdir("{}/config".format(rootpath))
call(["ln", "-s", "sample.env", ".env"])
chdir("{}/ros/wrm_image_publisher/scripts/config".format(rootpath))
call(["ln", "-s", "sample.env", ".env"])
chdir("{}/ros/mqtt_bridge/scripts/config".format(rootpath))
call(["ln", "-s", "sample.env", ".env"])
