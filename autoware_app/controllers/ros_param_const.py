#!/usr/bin/env python
# coding: utf-8


CONST = {
    "MAPDATAPATH": "/res/data/map/",
    "VEHICLEINFOPATH": "/res/data/vehicle/vehicle_info/vehicle_info.yaml",
    "VEHICLEMODELPATH": "/res/data/vehicle/vehicle_model/",
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
    },
    "LAUNCHPATH": {
        "INITIALIZATION": "/res/initialization/initialization.launch",
        "ROSBAG": "/res/rosbag/rosbag.launch",
        "MAP": "/res/map/map.launch",
        "SENSING": "/res/sensing/sensing.launch",
        "LOCALIZATION": "/res/localization/localization.launch",
        "MISSION": "/res/mission/mission.launch",
        "MOTION": "/res/motion/motion.launch",
    },
    "MODE": {
        "rosbagMode": {
            "allActivation": "/res/allActivation/allActivationRosbag.launch",
            "domain": "allActivationRosbag"
        },
        "simulatorMode": {
            "allActivation": "/res/allActivation/allActivationSimulator.launch",
            "domain": "allActivationSimulator"
        },
        "driveMode": {
            "allActivation": "/res/allActivation/allActivationDrive.launch",
            "domain": "allActivationDrive"
        }
    }

}
