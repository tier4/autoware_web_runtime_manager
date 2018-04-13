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
    "VEHICLE_INFO_PATH": "/vehicle_info/vehicle_info.yaml",
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
    }

}
