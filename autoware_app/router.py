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
import paho.mqtt.client as mqtt
import signal
import sys
import json
import urllib
import urllib2
import rosparam

class MqttRosLauncher:

    __initial_rtm_status = {
        #get ros launch status for button on/off of web page
        "buttonInit":{
            "topic" : ""
        },
        #button launch signal and status,response
        "initialization": {
            "enable": True,
            "mode": "off",
            "topic": ""
        },
        "map": {
            "enable": False,
            "mode": "off",
            "topic": ""
        },
        "localization": {
            "enable": False,
            "mode": "off",
            "topic": ""
        },
        "mission": {
            "enable": False,
            "mode": "off",
            "topic": ""
        },
        "motion": {
            "enable": False,
            "mode": "off",
            "topic": ""
        },
        "sensing": {
            "enable": False,
            "mode": "off",
            "topic": ""
        },
        "detection": {
            "enable": False,
            "mode": "off",
            "topic": ""
        },
        "rosbag": {
            "enable": False,
            "mode": "off",
            "topic": ""
        },
        "play": {
            "enable": False,
            "mode": "off",
            "topic": ""
        },
        "gateway": {
            "enable": False,
            "mode": "off",
            "topic": ""
        },
        "on": {
            "enable": False,
            "mode": "off",
            "topic": ""
        },
        #get rosparam
        "get_param": {
            "topic": ""
        }
    }    
    
    def __init__(self):
        self.rosController = ROSController(env)
        self.client = mqtt.Client(protocol=mqtt.MQTTv311)


    def TopicGet(self):
        url = "http://localhost:5000/topicData"
        try :
            params = urllib.urlencode({'name':"test"})
            req = urllib2.Request(url, params)
            res= urllib2.urlopen(req)
            json_data = json.loads(res.read())
            self.__userid = json_data["fixeddata"]["userid"]
            self.__carid = json_data["fixeddata"]["carid"]
            self.__toAutoware = json_data["fixeddata"]["toAutoware"]
            self.__fromAutoware = json_data["fixeddata"]["fromAutoware"]
            for key,value in json_data["topicdata"].items():
                if key in self.__initial_rtm_status:
                    self.__initial_rtm_status[key]["topic"] = value["topic"]
                    print(value["topic"])
            self.rtm_status = deepcopy(self.__initial_rtm_status)            
            print(self.rtm_status)
            return True
        except urllib2.HTTPError, e:
            print e.code,e.reason
            return False
            

    def mqttStart(self):
        #mqtt setting
        self.client.on_connect = self.__on_connect
        self.client.on_message = self.__on_message
        self.client.on_disconnect = self.__on_disconnect
        self.client.connect(env["MQTT_HOST"], int(env["MQTT_PYTHON_PORT"]))

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
    
    def __initializeRtmStatus(self):
        self.rtm_status = deepcopy(self.__initial_rtm_status)

    def __getRTMStatus(self):
        print("getRTMStatus")
        return self.rtm_status

        
    def __roslaunch(self,domain,label,message):
        self.rtm_status[label]["mode"] = message
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
                    self.__initializeRtmStatus()
                self.rosController.launch(domain, label, message)
                return "ok"
        except:
            traceback.print_exc()
            return "error"

    def __getParam(self,message):
        print("get rosparam:" + message)
        return str(rosparam.get_param(message))
        
    def __execution(self,msg):
        space,header,body,direction = msg.topic.split("/")
        topic_type,domain,label = body.split(".")
        
        if topic_type == "buttonInit":
            return json.dumps(self.__getRTMStatus())
        elif topic_type == "button":
            return self.__roslaunch(domain,label,msg.payload)
        elif topic_type == "getParam":
            return self.__getParam(msg.payload)
        
    def __on_connect(self,client, userdata, flags, respons_code):
        print('status {0}'.format(respons_code))

        #topic name making and subscriber start
        header = "/" + self.__userid + "." + self.__carid
        direction  = "/" + self.__toAutoware

        for key,value in self.rtm_status.items():
            body = "/" + value["topic"]
            topic = header + body + direction
            print(topic)
            self.client.subscribe(topic)

            
    def __on_message(self,client, userdata, msg):
        print(msg.topic + ' ' + str(msg.payload))
        res = self.__execution(msg)
        
        space,header,body,direction = msg.topic.split("/")
        topic = "/" + header + "/" + body + "/" + self.__fromAutoware
        self.client.publish(topic,str(res))

        
    def __on_disconnect(self,a,b,c,):
        logging.debug("DisConnected result code "+str(rc))
        #self.client.loop_stop()


mqtt_roslauncher = MqttRosLauncher()

def handler(signal, frame):
    mqtt_roslauncher.disConnect();
    sys.exit(0)
        
if __name__ == '__main__':
    signal.signal(signal.SIGINT,handler)

    if mqtt_roslauncher.TopicGet():
        mqtt_roslauncher.mqttStart()
    

    
