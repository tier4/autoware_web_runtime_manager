#!/usr/bin/env python
# coding: utf-8
from copy import deepcopy
from os import listdir
from os.path import realpath, abspath, dirname
from config.env import env
from controllers.ros_controller import ROSController
from controllers.vector_map_loader import VectorMap
from prepare import kill_web_video_server
import traceback
from pprint import PrettyPrinter
import paho.mqtt.client as mqtt
import signal
import sys

class MQTT_RosLauncher:
    __host = '127.0.0.1'
    __port = 1883
    __userid = "test"
    __carid = "test"
    __toAutoware = "UtoA";
    __fromAutoware = "AtoU";


    __initial_rtm_status = {
        "initialization": {
            "initialization": {
                "topic" : "initialization.initialization",
                "enable": True,
                "mode": "off"
            }
        },
        "map": {
            "map": {
                "topic" : "map.map",
                "enable": False,
                "mode": "off"
            }
        },
        "localization": {
            "localization": {
                "topic" : "localization.localization",
                "enable": False,
                "mode": "off"
            }
        },
        "mission": {
            "mission": {
                "topic" : "mission.mission",
                "enable": False,
                "mode": "off"
            }
        },
        "motion": {
            "motion": {
                "topic" : "motion.motion",
                "enable": False,
                "mode": "off"
            }
        },
        "sensing": {
            "sensing": {
                "topic" : "sensing.sensing",
                "enable": False,
                "mode": "off"
            }
        },
        "detection": {
            "detection": {
                "topic" : "detection.detection",
                "enable": False,
                "mode": "off"
            }
        },
        "rosbag": {
            "rosbag": {
                "topic" : "rosbag.rosbag",
                "enable": False,
                "mode": "off"
            },
            "play": {
                "topic" : "rosbag.play",
                "enable": False,
                "mode": "off"
            }
        },
        "gateway": {
            "gateway": {
                "topic" : "gateway.gateway",
                "enable": False,
                "mode": "off"
            },
            "on": {
                "topic" : "gateway.on",
                "enable": False,
                "mode": "off"
            }
        }
    }

    def __init__(self):
        self.rtm_status = deepcopy(self.__initial_rtm_status)
        self.rosController = ROSController(env)
        self.client = mqtt.Client(protocol=mqtt.MQTTv311)



    def mqtt_start(self):
        #mqtt setting
        self.client.on_connect = self.__on_connect
        self.client.on_message = self.__on_message
        self.client.on_disconnect = self.__on_disconnect
        self.client.connect(self.__host, self.__port)

        self.client.loop_forever()

    def disConnect(self):
        print("切断します")
        self.client.loop_stop()
        self.client.disconnect()
    
    
    def __exitRTM(self):
        print("exitRTM")
        self.rosController.killall()
        del self.rosController
        self.rosController = ROSController(env)
        kill_web_video_server()
        return "ok"
    
    def __initialize_rtm_status(self):
        self.rtm_status = deepcopy(self.__initial_rtm_status)
        
    def __roslaunch(self,domain,label,message):
        self.rtm_status[domain][label]["mode"] = message
        try:
            if (domain,label) == ("rosbag","play"):
                if message == "on":
                    self.rosController.play_rosbag()
                else:
                    self.rosController.pause_rosbag()
                return "ok"
            elif (domain,label) == ("gateway","on"):
                if message == "on":
                    self.rosController.gateway_on()
                else:
                    self.rosController.gateway_off()
                return "ok"
            else:
                if (domain, label, message) == ("initialization","initialization", "off"):
                    self.__exitRTM()
                    self.__initialize_rtm_status()
                self.rosController.launch(domain, label, message)
                return "ok"
        except:
            traceback.print_exc()
            return "error"
                        
    def __execution(self,msg):
        space,header,body,direction = msg.topic.split("/")
        mtype,domain,label = body.split(".")
        
        res = ""
        if mtype == "button":
            res = self.__roslaunch(domain,label,msg.payload)
        elif mtype == "image":
            #to do
            print("test")
        return res
        
    def __on_connect(self,client, userdata, flags, respons_code):
        print('status {0}'.format(respons_code))

        header = "/" + self.__userid + "." + self.__carid
        direction  = "/" + self.__toAutoware

        for key,value in self.rtm_status.items():
            for key2,value2 in value.items():
                if "topic" in value2.keys():
                    body = "/" + "button" + "." + value2["topic"]
                    topic = header + body + direction
                    print topic
                    self.client.subscribe(topic)


    def __on_message(self,client, userdata, msg):
        print(msg.topic + ' ' + str(msg.payload))        
        res = self.__execution(msg)
        
        space,header,body,direction = msg.topic.split("/")
        topic = "/" + header + "/" + body + "/" + self.__fromAutoware
        print(topic,res)
        self.client.publish(topic,res)

    def __on_disconnect(self):
        logging.debug("DisConnected result code "+str(rc))
        #self.client.loop_stop()


mqtt_roslauncher = MQTT_RosLauncher()

def handler(signal, frame):
    mqtt_roslauncher.disConnect();
    sys.exit(0)
        
if __name__ == '__main__':
    signal.signal(signal.SIGINT,handler)

    mqtt_roslauncher.mqtt_start()
    

    
