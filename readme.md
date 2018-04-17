# Autoware Web UI

## Installing

```
Web Server
$ cd app
$ python setup.py

Autoware PC
$ cd autoware_app
$ python autoware_app/setup.py

```

## Mosquitto Install and Settings

```angular2html
$ sudo vim /etc/mosquitto/mosquitto.conf

Add the following lines

listener 1883

listener 9091 localhost
protocol websockets

$ sudo service mosquitto restart

```

## Autoware Installing
Installing Autoware as below

http://wiki.ros.org/kinetic/Installation/Ubuntu

https://github.com/CPFL/Autoware

## Autoware Settings
```angular2html
$ cp -r ${HOME}/autoware_web_runtime_manager/autoware_app/ros/* ${HOME}/Autoware/ros/src/util/packages
$ cd Autoware/ros/
$ catkin_make
$ source ./devel/setup.bash
$ rosdep update
$ rosdep install mqtt_bridge

```


## Running

```
$ cd app
$ sh run.sh

Open new terminal
$ cd autoware_app
$ sh run.sh
```

## Access

http://localhost:5000/