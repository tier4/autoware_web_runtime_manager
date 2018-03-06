# Autoware Web UI

## Installing

```
Web Server
$ python app/setup.py

Autoware PC
$ python autoware_app/setup.py

```

## Mosquitto Install and Settings

```angular2html
$ sudo apt-get install mosquitto mosquitto-clients

$ sudo vim /etc/mosquitto/mosquitto.conf

Add the following lines

listener 1883

listener 9091 localhost
protocol websockets

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
$ sh app/run.sh
$ sh autoware_app/run.sh
```

## Access

http://localhost:5000/