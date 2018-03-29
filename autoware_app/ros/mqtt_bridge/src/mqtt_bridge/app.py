# -*- coding: utf-8 -*-
from __future__ import absolute_import

import inject
import paho.mqtt.client as mqtt
import rospy

from .bridge import create_bridge
from .mqtt_client import create_private_path_extractor
from .util import lookup_object

from config.env import env
import json

def create_config(mqtt_client, serializer, deserializer, mqtt_private_path):
    if isinstance(serializer, basestring):
        serializer = lookup_object(serializer)
    if isinstance(deserializer, basestring):
        deserializer = lookup_object(deserializer)
    private_path_extractor = create_private_path_extractor(mqtt_private_path)
    def config(binder):
        binder.bind('serializer', serializer)
        binder.bind('deserializer', deserializer)
        binder.bind(mqtt.Client, mqtt_client)
        binder.bind('mqtt_private_path_extractor', private_path_extractor)
    return config


def mqtt_bridge_node():
    # init node
    rospy.init_node('mqtt_bridge_node')

    # load parameters
    params = rospy.get_param("~", {})
    mqtt_params = params.pop("mqtt", {})
    conn_params = mqtt_params.pop("connection")
    mqtt_private_path = mqtt_params.pop("private_path", "")
    bridge_params = params.get("bridge", [])

    # load wrm parameters
    wrm_path = params.pop("wrm")
    sf = open(env["PATH_WRM_DIR"] + "/config.json", "r")
    json_data = json.load(sf)
    
    __userid = json_data["fixeddata"]["userid"]
    __carid = json_data["fixeddata"]["carid"]
    __toAutoware = json_data["fixeddata"]["toAutoware"]
    __fromAutoware = json_data["fixeddata"]["fromAutoware"]
    __topic = json_data["topicdata"]["ImageRaw"]["topic"]
    conn_params["host"] = env["MQTT_HOST"]
    conn_params["port"] = env["MQTT_PYTHON_PORT"]

    header = "/" + __userid + "." + __carid
    fromdirection  = "/" + __fromAutoware
    todirection  = "/" + __toAutoware
    
    for i,value in enumerate(bridge_params):
        if value["factory"] == "mqtt_bridge.bridge:RosToMqttBridge":
            key = value["topic_to"]
            body = "/" + json_data["topicdata"][key]["topic"]
            bridge_params[i]["topic_to"] = header + body + fromdirection
        elif value["factory"] == "mqtt_bridge.bridge:MqttToRosBridge":
            key = value["topic_from"]
            body = "/" + json_data["topicdata"][key]["topic"]
            bridge_params[i]["topic_from"] = header + body + todirection
    
    # create mqtt client
    mqtt_client_factory_name = rospy.get_param(
        "~mqtt_client_factory", ".mqtt_client:default_mqtt_client_factory")
    mqtt_client_factory = lookup_object(mqtt_client_factory_name)
    mqtt_client = mqtt_client_factory(mqtt_params)

    # load serializer and deserializer
    serializer = params.get('serializer', 'json:dumps')
    deserializer = params.get('deserializer', 'json:loads')

    # dependency injection
    config = create_config(
        mqtt_client, serializer, deserializer, mqtt_private_path)
    inject.configure(config)

    # configure and connect to MQTT broker
    mqtt_client.on_connect = _on_connect
    mqtt_client.on_disconnect = _on_disconnect
    mqtt_client.connect(**conn_params)

    # configure bridges
    bridges = []
    for bridge_args in bridge_params:
        bridges.append(create_bridge(**bridge_args))

    # start MQTT loop
    mqtt_client.loop_start()

    # register shutdown callback and spin
    rospy.on_shutdown(mqtt_client.disconnect)
    rospy.on_shutdown(mqtt_client.loop_stop)
    rospy.spin()


def _on_connect(client, userdata, flags, response_code):
    rospy.loginfo('MQTT connected')


def _on_disconnect(client, userdata, response_code):
    rospy.loginfo('MQTT disconnected')


__all__ = ['mqtt_bridge_node']
