# -*- coding: utf-8 -*-
from __future__ import absolute_import

import inject
import paho.mqtt.client as mqtt
import rospy

from .bridge import create_bridge
from .mqtt_client import create_private_path_extractor
from .util import lookup_object

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
    topic_params_string = rospy.get_param("~topic_params", {})
    topic_params = json.loads(topic_params_string)

    host = topic_params["host"]
    port = topic_params["port"]
    keepalive = topic_params["keepalive"]
    protocol = topic_params["protocol"]

    mqtt_params = {
        "client": {
            "protocol": protocol
        },
        "connection": {
            "host": host,
            "port": port,
            "keepalive": keepalive
        }
    }
    conn_params = mqtt_params["connection"]
    mqtt_private_path = "device/001"
    bridge_params = topic_params["bridge"]

    # create mqtt client
    mqtt_client_factory_name = rospy.get_param(
        "~mqtt_client_factory", ".mqtt_client:default_mqtt_client_factory")
    mqtt_client_factory = lookup_object(mqtt_client_factory_name)
    mqtt_client = mqtt_client_factory(mqtt_params)

    # load serializer and deserializer
    serializer = "json:dumps"
    deserializer = "json:loads"

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
