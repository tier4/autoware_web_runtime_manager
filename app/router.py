#!/usr/bin/env python
# coding: utf-8
from copy import deepcopy
from flask import Flask, request, send_from_directory, current_app, render_template, Response, jsonify
from flask_cors import CORS
from os import listdir
from os.path import realpath, abspath, dirname
from config.env import env
from controllers.ros_controller import ROSController
from controllers.vector_map_loader import VectorMap
from prepare import kill_web_video_server
import traceback
from pprint import PrettyPrinter
import json

pp = PrettyPrinter(indent=2).pprint
"""
initial_rtm_status = {
    "initialization": {
        "initialization": {
            "enable": True,
            "mode": "off"
        }
    },
    "map": {
        "map": {
            "enable": False,
            "mode": "off"
        }
    },
    "localization": {
        "localization": {
            "enable": False,
            "mode": "off"
        }
    },
    "mission": {
        "mission": {
            "enable": False,
            "mode": "off"
        }
    },
    "motion": {
        "motion": {
            "enable": False,
            "mode": "off"
        }
    },
    "sensing": {
        "sensing": {
            "enable": False,
            "mode": "off"
        }
    },
    "detection": {
        "detection": {
            "enable": False,
            "mode": "off"
        }
    },
    "rosbag": {
        "rosbag": {
            "enable": False,
            "mode": "off"
        },
        "play": {
            "enable": False,
            "mode": "off"
        }
    },
    "gateway": {
        "gateway": {
            "enable": False,
            "mode": "off"
        },
        "on": {
            "enable": False,
            "mode": "off"
        }
    }
}
"""

flask = Flask(__name__)
CORS(flask)
with flask.app_context():
    flask.rosController = ROSController(env)
    #flask.rtm_status = deepcopy(initial_rtm_status)

"""
def initialize_rtm_status():
    flask.rtm_status = deepcopy(initial_rtm_status)
"""

def api_response(code=200, message={}):
    response = jsonify(message)
    response.status_code = code
    return response


@flask.route('/', methods=["GET"])
def root():
    print("root")
    return send_from_directory(
        directory="./views/", filename="index.html")

#@flask.route("/getRTMStatus", methods=["GET"])
#def getRTMStatus():
    #print("getRTMStatus", flask.rtm_status, request.args.get("date"))
#    return api_response(200, flask.rtm_status)

@flask.route("/topicData", methods=["POST"])
def topicGetter():
    print("came to topicGetter")
    if request.method == "POST":
        name = request.form["name"]
        f = open('./app/topic.json', 'r')
        topic_init = json.load(f)
        topic_init["fixeddata"]["userid"]="test"
        topic_init["fixeddata"]["carid"]="test"        
        #print('json_dict:{}'.format(topic_init))
        return api_response(200, topic_init)
    else:
        return api_response(500, "")


"""
@flask.route("/getRTMStatus", methods=["GET"])
def getRTMStatus():
    print("getRTMStatus", flask.rtm_status, request.args.get("date"))
    return api_response(200, flask.rtm_status)
"""

@flask.route("/.config/model/<path:path>", methods=["GET"])
def getVehicleModel(path):
    print("getVehicleModel", path)
    return send_from_directory(
        directory=env["PATH_AUTOWARE_DIR"] + "/ros/src/.config/model/", filename=path, as_attachment=True)


@flask.route("/res/<type>/<path:path>", methods=["GET"])
def getResources(type, path):
    print("getResources", type, path)
    if type in ["lib", "node_modules", "build", "static"]:
        return send_from_directory(
            directory='./views/'+type+"/", filename=path, as_attachment=True)
    else:
        return api_response(500, {"type": type, "path": path})


@flask.route("/getVectorMapViewData")
def getVectorMapViewData():
    pathDir = realpath("./app/controllers/res/map/vectors/")+"/"
    vectorMap = VectorMap()
    vectorMap.load(pathDir)
    return api_response(code=200, message=vectorMap.getViewData())


@flask.route('/getPCDFileNames')
def getPCDFileNames():
    pathDir = realpath("./app/controllers/res/map/points/")
    return api_response(code=200, message=listdir(pathDir))


@flask.route('/getPCDFile/<filename>')
def getPCDFile(filename):
    pathDir = realpath("./app/controllers/res/map/points")
    return send_from_directory(
        directory=pathDir, filename=filename, as_attachment=True)

"""
@flask.route('/roslaunch/<domain>/<target>/<mode>')
def roslaunch(domain, target, mode):
    print("roslaunch", domain, target, mode)
    flask.rtm_status[domain][target]["mode"] = mode
    try:
        if (domain, target) == ("rosbag", "play"):
            if mode == "on":
                flask.rosController.play_rosbag()
            else:
                flask.rosController.pause_rosbag()
            return api_response(200, {"domain": domain, "target": target, "mode": mode})
        elif (domain, target) == ("gateway", "on"):
            if mode == "on":
                flask.rosController.gateway_on()
            else:
                flask.rosController.gateway_off()
            return api_response(200, {"domain": domain, "target": target, "mode": mode})
        else:
            if (domain, target, mode) == ("initialization", "initialization", "off"):
                exitRTM()
                initialize_rtm_status()
            flask.rosController.launch(domain, target, mode)
            return api_response(200, {"domain": domain, "target": target, "mode": mode})
    except:
        traceback.print_exc()
        return api_response(500, {"target": target, "mode": mode})
"""

"""
@flask.route("/exitRTM")
def exitRTM():
    print("exitRTM")
    flask.rosController.killall()
    del flask.rosController
    flask.rosController = ROSController(env)
    kill_web_video_server()
    return api_response(200, {"exitRTM": "done"})
"""

if __name__ == '__main__':
    print("flask run")
    flask.run(host=env["AUTOWARE_WEB_UI_HOST"], port=int(env["AUTOWARE_WEB_UI_PORT"]))#, processes=1, threaded=True)
    # flask.run(debug=True)
