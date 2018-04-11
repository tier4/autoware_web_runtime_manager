#!/usr/bin/env python
# coding: utf-8


CONST = {
    "VEHICLE": {
        "milee": "milee",
        "estime": "estima",
        "prius": "prius",
        "default": "default"
    },
    "VEHICLEMODELPATH": {
        "milee": "Autoware/ros/src/.config/model/milee.urdf",
        "estime": "Autoware/ros/src/.config/model/estima_white.urdf",
        "prius": "Autoware/ros/src/.config/model/prius.urdf",
        "default": "Autoware/ros/src/.config/model/default.urdf"
    },
    "DATAPATH": "/res/data/",
    "MAPPATH": {
        "POINTSMAP": "/points/",
        "VECTORMAP": "/vectors/",
        "TF": "/tf/tf.launch"
    },
    "RVIZSETTING": {
        "PATH": "/res/rviz/rviz_setting/",
        "NAME": "rviz_setting.rviz"
    },
    "ROSBAG": {
        "BASEPATH": "/rosbag/",
        "REPEAT": "--loop"
    },
    "WAYPOINTSPATH": {
        "BASEPATH": "/waypoints/"
    },
    "SETTING": {
        "PATH": "/res/parameter/save/"
    }
}
