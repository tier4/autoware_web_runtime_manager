#!/usr/bin/env python
# coding: utf-8

from subprocess import call
from os import getcwd, chdir
from argparse import ArgumentParser

parser = ArgumentParser(description='This script is ...')
parser.add_argument("-F", "--force", type=str, default="force", help="force")
args = parser.parse_args()

rootpath = getcwd()
print rootpath

# apt-get
call(["sudo", "apt-get", "update"])
call(["sudo", "apt-get", "install", "mosquitto", "mosquitto-clients"])
call(["sudo", "apt-get", "install", "-y", "nodejs", "npm"])

# pip
call(["pip", "install", "--upgrade", "pip"])
call(["sudo", "pip", "install", "-r", "requirements.txt"])

# npm, node
call(["sudo", "npm", "cache", "clean"])
call(["sudo", "npm", "install", "n", "-g"])
call(["sudo", "n", "stable"])
call(["sudo", "ln", "-sf", "/usr/local/bin/node", "/usr/bin/node"])

chdir("{}/views".format(rootpath))
call(["npm", "install"])
call(["sudo", "npm", "install", "--save", "-g", "gulp-cli"])
call(["sudo", "npm", "install", "--save", "react-modal"])

# env
chdir("{}/config".format(rootpath))
call(["ln", "-s", "sample.env", ".env"])
chdir("{}/views".format(rootpath))
call(["ln", "-s", "../config/.env", ".env"])
chdir("{}/controllers".format(rootpath))
call(["ln", "-s", "../config", "config"])

# gulp browserify
chdir("{}/views".format(rootpath))
call(["gulp", "browserify"])

