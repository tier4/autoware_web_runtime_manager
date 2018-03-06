# Autoware Web UI


## Requirements

- npm and node

```
$ sudo apt-get install -y nodejs npm
$ sudo npm cache clean
$ sudo npm install n -g
$ sudo n stable
$ sudo ln -sf /usr/local/bin/node /usr/bin/node
```

## Installing

```
$ python setup.py
```

## Autoware Setting
Installing Autoware as below

https://github.com/CPFL/Autoware



## Setting


By default, the connection destination is the localhost.

If you start up other host, you have to change settings.

###MQTT_Bridge Setting
```
$ cd autoware_web_runtime_manager/autoware_app/ros/mqtt_bridge/config/
$ vim demo_params.yaml

mqtt:
  client:
    protocol: 4      # MQTTv311
  connection:
    host: localhost  # MQTT Broker address
    port: 1883       # MQTT Broker port
    keepalive: 60
  private_path: device/001
serializer: json:dumps
deserializer: json:loads


```

###Browser Connection Settings
```angular2html
$ cd autoware_web_runtime_manager/app/config

AUTOWARE_WEB_UI_HOST="localhost"  #Host Address of Web Server
AUTOWARE_WEB_UI_PORT="5000"  #Host Port of Web Server
MQTT_HOST="localhost"  #MQTT broker address that browser connect
MQTT_JS_PORT = "9091" #MQTT broker port that browser connect

```

###Autoware Connection Client Connection Settings
```angular2html
$ cd autoware_web_runtime_manager/autoware_app/config
$ vim sample.env

AUTOWARE_WEB_UI_HOST="localhost"  #Host Address of Web Server
AUTOWARE_WEB_UI_PORT="5000"  #Host Port of Web Server
MQTT_HOST="localhost"  #MQTT broker address that browser connect
MQTT_PYTHON_PORT = "1883" #MQTT broker port that browser connect

```

###Autoware Connection Client Connection Settings
```angular2html
$ cd autoware_web_runtime_manager/autoware_app/ros/wrm_image_publisher/scripts/config
$ vim sample.env

AUTOWARE_WEB_UI_HOST="localhost"  #Host Address of Web Server
AUTOWARE_WEB_UI_PORT="5000"  #Host Port of Web Server
MQTT_HOST="localhost"  #MQTT broker address that browser connect
MQTT_PYTHON_PORT = "1883" #MQTT broker port that browser connect

```

## Autoware Settings
```angular2html
$cp -r autoware_web_runtime_manager/autoware_app/ros/* ${HOME}/Autoware/ros/src/util/package

```



## Running

```
$ sh run.sh
```
