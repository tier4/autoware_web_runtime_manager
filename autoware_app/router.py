#!/usr/bin/env python
# coding: utf-8

import rosparam

from copy import deepcopy
from config.env import env
from controllers.ros_controller import ROSController
import traceback
import paho.mqtt.client as mqtt
import signal
import sys
import json
import urllib
import urllib2


class MqttRosLauncher:
    __initial_rtm_status = {
        # get ros launch status for button on/off of web page
        "buttonInit": {
            "topic_send": "",
            "topic_receive": "",
            "subscribe": True
        },
        # setting save/load
        "settingSave": {
            "topic_send": "",
            "topic_receive": "",
            "subscribe": True
        },
        "settingLoad": {
            "topic_send": "",
            "topic_receive": "",
            "subscribe": True
        },
        # button launch signal and status,response
        "initialization": {
            "enable": True,
            "mode": "off",
            "topic_send": "",
            "topic_receive": "",
            "subscribe": True
        },
        "map": {
            "enable": False,
            "mode": "off",
            "topic_send": "",
            "topic_receive": "",
            "subscribe": True
        },
        "localization": {
            "enable": False,
            "mode": "off",
            "topic_send": "",
            "topic_receive": "",
            "subscribe": True
        },
        "mission": {
            "enable": False,
            "mode": "off",
            "topic_send": "",
            "topic_receive": "",
            "subscribe": True
        },
        "motion": {
            "enable": False,
            "mode": "off",
            "topic_send": "",
            "topic_receive": "",
            "subscribe": True
        },
        "sensing": {
            "enable": False,
            "mode": "off",
            "topic_send": "",
            "topic_receive": "",
            "subscribe": True
        },
        "detection": {
            "enable": False,
            "mode": "off",
            "topic_send": "",
            "topic_receive": "",
            "subscribe": True
        },
        "rosbag": {
            "enable": False,
            "mode": "off",
            "topic_send": "",
            "topic_receive": "",
            "subscribe": True
        },
        "play": {
            "enable": False,
            "mode": "off",
            "topic_send": "",
            "topic_receive": "",
            "subscribe": True
        },
        "gateway": {
            "enable": False,
            "mode": "off",
            "topic_send": "",
            "topic_receive": "",
            "subscribe": True
        },
        "on": {
            "enable": False,
            "mode": "off",
            "topic_send": "",
            "topic_receive": "",
            "subscribe": True
        },
        "rviz": {
            "enable": False,
            "mode": "off",
            "topic_send": "",
            "topic_receive": "",
            "subscribe": True
        },
        "setting": {
            "enable": False,
            "mode": "off",
            "topic_send": "",
            "topic_receive": "",
            "subscribe": True
        },
        "allActivation": {
            "enable": False,
            "mode": "off",
            "topic_send": "",
            "topic_receive": "",
            "subscribe": True
        },
        # get rosparam
        "get_param": {
            "topic_send": "",
            "topic_receive": "",
            "subscribe": True
        },
        "ImageRaw": {
            "topic_send": "",
            "topic_receive": "",
            "subscribe": False,
            "mqttparam": True
        },
        "points_raw": {
            "topic_send": "",
            "topic_receive": "",
            "subscribe": False,
            "mqttparam": True
        },
        "ndt_pose": {
            "topic_send": "",
            "topic_receive": "",
            "subscribe": False,
            "mqttparam": True
        },
        "tf": {
            "topic_send": "",
            "topic_receive": "",
            "subscribe": False,
            "mqttparam": True
        },
        "vector_map": {
            "topic_send": "",
            "topic_receive": "",
            "subscribe": False,
            "mqttparam": True
        },
        "lane_waypoints_array": {
            "topic_send": "",
            "topic_receive": "",
            "subscribe": False,
            "mqttparam": True
        },
        "downsampled_next_target_mark": {
            "topic_send": "",
            "topic_receive": "",
            "subscribe": False,
            "mqttparam": True
        },
        "downsampled_trajectory_circle_mark": {
            "topic_send": "",
            "topic_receive": "",
            "subscribe": False,
            "mqttparam": True
        },
        "map_pose": {
            "topic_send": "",
            "topic_receive": "",
            "subscribe": False,
            "mqttparam": True
        },
        "clock": {
            "topic_send": "",
            "topic_receive": "",
            "subscribe": False,
            "mqttparam": True
        },
        "initialpose": {
            "topic_send": "",
            "topic_receive": "",
            "subscribe": False,
            "mqttparam": True
        }
    }

    def __init__(self):
        self.rosController = ROSController(env)
        self.client = mqtt.Client(protocol=mqtt.MQTTv311)

        self.rtm_status = {}

    @staticmethod
    def make_topic(user_id, vehicle_id, topic_type, domain, label, autoware, user):
        return {
            "topic_send": "/" + user_id + "/" + vehicle_id + "/" + topic_type +
                          "/" + domain + "/" + label + "/" + autoware + "/" + user,
            "topic_receive": "/" + user_id + "/" + vehicle_id + "/" + topic_type +
                             "/" + domain + "/" + label + "/" + user + "/" + autoware
        }

    @staticmethod
    def parse_topic(topic):
        parsed_topic = topic.split("/")

        return {
            "userID": parsed_topic[1],
            "vehicleID": parsed_topic[2],
            "type": parsed_topic[3],
            "domain": parsed_topic[4],
            "label": parsed_topic[5],
            "Autoware": parsed_topic[6],
            "User": parsed_topic[7]
        }

    def TopicGetAndCreate(self):
        url = "http://" + env["AUTOWARE_WEB_UI_HOST"] + ":" + env["AUTOWARE_WEB_UI_PORT"] + "/topicData"
        # url = "http://localhost:5000/topicData"
        try:
            params = urllib.urlencode({'name': "test"})
            req = urllib2.Request(url, params)
            res = urllib2.urlopen(req)
            json_data = json.loads(res.read())
            # set fixed data to variable and ros param.

            user_id = json_data["fixeddata"]["userID"]
            vehicle_id = json_data["fixeddata"]["vehicleID"]
            autoware = json_data["fixeddata"]["Autoware"]
            user = json_data["fixeddata"]["User"]

            ros_bridge_params = {
                "host": env["MQTT_HOST"],
                "port": int(env["MQTT_PYTHON_PORT"]),
                "keepalive": json_data["ros_bridge_data"]["keepalive"],
                "protocol": json_data["ros_bridge_data"]["protocol"],
                "bridge": []
            }

            image_bridge_params = {
                "host": env["MQTT_HOST"],
                "port": int(env["MQTT_PYTHON_PORT"]),
                "topic_send": "",
                "topic_receive": ""
            }

            for key, value in json_data["topicdata"].items():
                if key in self.__initial_rtm_status:
                    type = value["type"]
                    domain = value["domain"]
                    label = value["label"]
                    topics = \
                        self.make_topic(user_id, vehicle_id, type, domain, label, autoware, user)
                    self.__initial_rtm_status[key]["topic_send"] = topics["topic_send"]
                    self.__initial_rtm_status[key]["topic_receive"] = topics["topic_receive"]
                    if type == "rostopic":
                        if value["ros_to_mqtt"]:
                            bridge_data = {
                                "factory": "mqtt_bridge.bridge:RosToMqttBridge",
                                "msg_type": value["msg_type"],
                                "topic_from": value["ros_topic"],
                                "topic_to": topics["topic_send"]
                            }
                            ros_bridge_params["bridge"].append(bridge_data)
                        elif value["mqtt_to_ros"]:
                            bridge_data = {
                                "factory": "mqtt_bridge.bridge:MqttToRosBridge",
                                "msg_type": value["msg_type"],
                                "topic_from": topics["topic_receive"],
                                "topic_to": value["ros_topic"]
                            }
                            ros_bridge_params["bridge"].append(bridge_data)
                    if type == "image":
                        image_bridge_params["topic_send"] = topics["topic_send"]
                        image_bridge_params["topic_receive"] = topics["topic_receive"]

            self.rosController.setRosBridgeData(ros_bridge_params, image_bridge_params)

            self.rtm_status = deepcopy(self.__initial_rtm_status)
            return True
        except urllib2.HTTPError, e:
            print e.code, e.reason
            return False

    def mqttStart(self):
        # mqtt setting
        self.client.on_connect = self.__on_connect
        self.client.on_message = self.__on_message
        self.client.on_disconnect = self.__on_disconnect
        self.client.connect(env["MQTT_HOST"], int(env["MQTT_PYTHON_PORT"]))

        self.client.loop_forever()

    def disConnect(self):
        print("disconnect")
        self.client.loop_stop()
        self.client.disconnect()

    def __exitRTM(self):
        print("exitRTM")
        self.rosController.killall()
        del self.rosController
        self.rosController = ROSController(env)
        return "ok"

    def __initializeRtmStatus(self):
        self.rtm_status = deepcopy(self.__initial_rtm_status)

    def __getRTMStatus(self):
        print("getRTMStatus")
        res = {
            "rtm_status": self.rtm_status,
            "parameter_info": self.rosController.get_params()
        }
        return json.dumps(res)

    def __settingSaveLoad(self, label, message):
        if label == "settingSave":
            return self.rosController.settingSave(message)
        elif label == "settingLoad":
            return self.rosController.settingLoad(message)
        else:
            return "error"

    def __roslaunch(self, domain, label, message):
        self.rtm_status[label]["mode"] = message
        try:
            if (domain, label) == ("rosbag", "play"):
                if message == "on":
                    self.rosController.play_rosbag()
                else:
                    self.rosController.pause_rosbag()
                return "ok"
            elif (domain, label) == ("gateway", "on"):
                if message == "on":
                    self.rosController.gateway_on()
                else:
                    self.rosController.gateway_off()
                return "ok"
            else:
                if (domain, label, message) == ("initialization", "initialization", "off"):
                    self.__exitRTM()
                    self.__initializeRtmStatus()
                self.rosController.launch(domain, label, message)
                return "ok"
        except:
            traceback.print_exc()
            return "error"

    def __allLaunch(self, domain, label, message):
        all_activation = json.loads(message)
        on_flag = all_activation["on"]
        self.rtm_status[label]["mode"] = on_flag
        launch_mode = all_activation["settingParams"]["mode"]["mode"]
        try:
            self.rosController.all_launch(domain, label, on_flag, launch_mode)
            return "ok"
        except:
            traceback.print_exc()
            return "error"

    def __settingParams(self, message):
        params = json.loads(message)
        if self.rosController.set_param(params):
            return "ok"
        else:
            return "error"

    def __getParam(self, message):
        return str(rosparam.get_param(message))

    def __execution(self, parsed_topic, payload):
        topic_type = parsed_topic["type"]
        domain = parsed_topic["domain"]
        label = parsed_topic["label"]

        if topic_type == "buttonInit":
            return self.__getRTMStatus()
        elif topic_type == "settingSaveLoad":
            return self.__settingSaveLoad(label, payload)
        elif topic_type == "button":
            return self.__roslaunch(domain, label, payload)
        elif topic_type == "allActivation":
            return self.__allLaunch(domain, label, payload)
        elif topic_type == "settingParams":
            return self.__settingParams(payload)
        elif topic_type == "getParam":
            return self.__getParam(payload)

    def __on_connect(self, client, userdata, flags, respons_code):
        print('status {0}'.format(respons_code))

        for key, value in self.rtm_status.items():
            if value["subscribe"]:
                topic = value["topic_receive"]
                self.client.subscribe(topic)

    def __on_message(self, client, userdata, msg):

        parsed_topic = self.parse_topic(msg.topic)
        res = self.__execution(parsed_topic, msg.payload)

        topic = self.rtm_status[parsed_topic["label"]]["topic_send"]
        self.client.publish(topic, str(res))

    def __on_disconnect(self, client, userdata, rc):
        sys.stderr.write("DisConnected result code " + str(rc))
        # logging.debug("DisConnected result code " + str(rc))
        # self.client.loop_stop()


mqtt_roslauncher = MqttRosLauncher()


def handler(signal, frame):
    mqtt_roslauncher.disConnect();
    sys.exit(0)


if __name__ == '__main__':
    signal.signal(signal.SIGINT, handler)

    if mqtt_roslauncher.TopicGetAndCreate():
        mqtt_roslauncher.mqttStart()
