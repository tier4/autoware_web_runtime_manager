#!/usr/bin/env python
# coding: utf-8

from subprocess import call
from os import getcwd, chdir

rootpath = getcwd()
print rootpath

# pip
#call(["pip", "install", "--upgrade", "pip"])
call(["sudo", "pip", "install", "-r", "requirements.txt"])
call(["sudo", "pip", "install", "git+https://github.com/strawlab/python-pcl"])

# res

chdir("{}/controllers".format(rootpath))
call(["wget", "https://autoware.blob.core.windows.net/web-ui-resouces/res.zip"])
call(["unzip", "res.zip"])


# make_launch_files.py
#chdir("{}/app/controllers".format(rootpath))
#call(["python", "make_launch_files.py"])

# env
chdir("{}/config".format(rootpath))
call(["ln", "-s", "sample.env", ".env"])
chdir("{}/ros/wrm_image_publisher/scripts/config".format(rootpath))
call(["ln", "-s", "sample.env", ".env"])
