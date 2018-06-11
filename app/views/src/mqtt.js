/*import React from 'react';
  import ReactDOM from 'react-dom';
  import Responsive, { WidthProvider } from 'react-grid-layout';
  const ResponsiveReactGridLayout = WidthProvider(Responsive);
  import Button from "./button";
  import ROSLaunchRequest from "./roslaunch_request";
*/
import {WEB_UI_URL, MQTT_HOST_NAME, MQTT_PORT} from "./dotenv";
import {CONST} from "./const";

export default class MqttWrapper {

    constructor() {

        this.host = MQTT_HOST_NAME;
        this.port = parseInt(MQTT_PORT);

        //label is unique
        this.topics = {
            "topicdata": {
                "buttonInit": {
                    "domain": CONST.BUTTON_INIT.DOMAIN,
                    "label": CONST.BUTTON_INIT.LABEL,
                    "type": "buttonInit",
                    "topic_send": "",
                    "topic_receive": "",
                    "errorPublishMessage": "buttonInit topic can not publish.",
                    "callback": function () {
                    }
                },
                "settingSave": {
                    "domain": CONST.SETTING_SAVE.DOMAIN,
                    "label": CONST.SETTING_SAVE.LABEL,
                    "type": "settingSaveLoad",
                    "topic_send": "",
                    "topic_receive": "",
                    "errorPublishMessage": "settingSaveLoad topic can not publish.",
                    "callback": function () {
                    }
                },
                "settingLoad": {
                    "domain": CONST.SETTING_LOAD.DOMAIN,
                    "label": CONST.SETTING_LOAD.LABEL,
                    "type": "settingSaveLoad",
                    "topic_send": "",
                    "topic_receive": "",
                    "errorPublishMessage": "settingSaveLoad topic can not publish.",
                    "callback": function () {
                    }
                },
                "initialization": {
                    "domain": CONST.BUTTON.INITIALIZATION.DOMAIN,
                    "label": CONST.BUTTON.INITIALIZATION.LABEL,
                    "type": "button",
                    "topic_send": "",
                    "topic_receive": "",
                    "errorPublishMessage": "initialization topic can not publish.",
                    "callback": function () {
                    }
                },
                "map": {
                    "domain": CONST.BUTTON.MAP.DOMAIN,
                    "label": CONST.BUTTON.MAP.LABEL,
                    "type": "button",
                    "topic_send": "",
                    "topic_receive": "",
                    "errorPublishMessage": "map topic can not publish.",
                    "callback": function () {
                    }
                },
                "localization": {
                    "domain": CONST.BUTTON.LOCALIZATION.DOMAIN,
                    "label": CONST.BUTTON.LOCALIZATION.LABEL,
                    "type": "button",
                    "topic_send": "",
                    "topic_receive": "",
                    "errorPublishMessage": "localization topic can not publish.",
                    "callback": function () {
                    }
                },
                "mission": {
                    "domain": CONST.BUTTON.MISSION.DOMAIN,
                    "label": CONST.BUTTON.MISSION.LABEL,
                    "type": "button",
                    "topic_send": "",
                    "topic_receive": "",
                    "errorPublishMessage": "mission topic can not publish.",
                    "callback": function () {
                    }
                },
                "motion": {
                    "domain": CONST.BUTTON.MOTION.DOMAIN,
                    "label": CONST.BUTTON.MOTION.LABEL,
                    "type": "button",
                    "topic_send": "",
                    "topic_receive": "",
                    "errorPublishMessage": "motion topic can not publish.",
                    "callback": function () {
                    },
                },
                "sensing": {
                    "domain": CONST.BUTTON.SENSING.DOMAIN,
                    "label": CONST.BUTTON.SENSING.LABEL,
                    "type": "button",
                    "topic_send": "",
                    "topic_receive": "",
                    "errorPublishMessage": "sensing topic can not publish.",
                    "callback": function () {
                    }
                },
                "detection": {
                    "domain": CONST.BUTTON.DETECTION.DOMAIN,
                    "label": CONST.BUTTON.DETECTION.LABEL,
                    "type": "button",
                    "topic_send": "",
                    "topic_receive": "",
                    "errorPublishMessage": "detection topic can not publish.",
                    "callback": function () {
                    }
                },
                "rosbag": {
                    "domain": CONST.BUTTON.ROSBAG.DOMAIN,
                    "label": CONST.BUTTON.ROSBAG.LABEL,
                    "type": "button",
                    "topic_send": "",
                    "topic_receive": "",
                    "errorPublishMessage": "rosbag topic can not publish.",
                    "callback": function () {
                    }
                },
                "play": {
                    "domain": CONST.BUTTON.ROSBAG.DOMAIN,
                    "label": CONST.BUTTON.ROSBAG_PLAY.LABEL,
                    "type": "button",
                    "topic_send": "",
                    "topic_receive": "",
                    "errorPublishMessage": "rosbag_play topic can not publish.",
                    "callback": function () {
                    }
                },
                "gateway": {
                    "domain": CONST.BUTTON.GATEWAY.DOMAIN,
                    "label": CONST.BUTTON.GATEWAY.LABEL,
                    "type": "button",
                    "topic_send": "",
                    "topic_receive": "",
                    "errorPublishMessage": "gateway topic can not publish.",
                    "callback": function () {
                    }
                },
                "on": {
                    "domain": CONST.BUTTON.GATEWAY.DOMAIN,
                    "label": CONST.BUTTON.GATEWAY_ON.LABEL,
                    "type": "button",
                    "topic_send": "",
                    "topic_receive": "",
                    "errorPublishMessage": "gateway_on topic can not publish.",
                    "callback": function () {
                    }
                },
                "rviz": {
                    "domain": CONST.BUTTON.RVIZ.DOMAIN,
                    "label": CONST.BUTTON.RVIZ.LABEL,
                    "type": "button",
                    "topic_send": "",
                    "topic_receive": "",
                    "errorPublishMessage": "rviz topic can not publish.",
                    "callback": function () {
                    }
                },
                "setting": {
                    "domain": CONST.BUTTON.SETTING.DOMAIN,
                    "label": CONST.BUTTON.SETTING.LABEL,
                    "type": "button",
                    "topic_send": "",
                    "topic_receive": "",
                    "errorPublishMessage": "setting topic can not publish.",
                    "callback": function () {
                    }
                },
                "allActivation": {
                    "domain": CONST.BUTTON.ALL_ACTIVATION.DOMAIN,
                    "label": CONST.BUTTON.ALL_ACTIVATION.LABEL,
                    "type": "allActivation",
                    "topic_send": "",
                    "topic_receive": "",
                    "errorPublishMessage": "All Activation topic can not publish.",
                    "callback": function () {
                    }
                },
                "ImageRaw": {
                    "domain": CONST.VISUALIZATION_OBJECT.IMAGE_RAW,
                    "label": CONST.VISUALIZATION_OBJECT.IMAGE_RAW,
                    "type": "image",
                    "topic_send": "",
                    "topic_receive": "",
                    "callback": function () {
                    }
                },
                "points_raw": {
                    "topic_send": "",
                    "topic_receive": "",
                    "callback": function () {
                    }
                },
                "ndt_pose": {
                    "topic_send": "",
                    "topic_receive": "",
                    "callback": function () {
                    }
                },
                "tf": {
                    "topic_send": "",
                    "topic_receive": "",
                    "callback": function () {
                    }
                },
                "vector_map": {
                    "topic_send": "",
                    "topic_receive": "",
                    "callback": function () {
                    }
                },
                "lane_waypoints_array": {
                    "topic_send": "",
                    "topic_receive": "",
                    "callback": function () {
                    }
                },
                "downsampled_next_target_mark": {
                    "topic_send": "",
                    "topic_receive": "",
                    "callback": function () {
                    }
                },
                "downsampled_trajectory_circle_mark": {
                    "topic_send": "",
                    "topic_receive": "",
                    "callback": function () {
                    }
                },
                "map_pose": {
                    "topic_send": "",
                    "topic_receive": "",
                    "callback": function () {
                    }
                },
                "get_param": {
                    "topic_send": "",
                    "topic_receive": "",
                    "callback": function () {
                    }
                },
                "clock": {
                    "topic_send": "",
                    "topic_receive": "",
                    "callback": function () {
                    }
                },
                "initialpose": {
                    "topic_send": "",
                    "topic_receive": "",
                    "callback": function () {
                    }
                }
            }
        };

        let time = new Date();
        let clientId = "wrmClient" + String(time.getTime());
        this.mqttClient = new Paho.MQTT.Client(this.host, this.port, clientId);

        this.mqttClient.onConnectionLost = onConnectionLost;
        this.mqttClient.onMessageArrived = onMessageArrived.bind(this);

        function onConnect() {
            for (const key of Object.keys(this.topics["topicdata"])) {
                this.onSubscribeTopic(key);
            }
            //get button status that the autoware has.
            this.onPublish("buttonInit", "GET");
        }

        function onMessageArrived(message) {
            let parsed_topic = MqttWrapper.parse_topic(message.destinationName);
            this.topics["topicdata"][parsed_topic["label"]].callback(message);
        }

        function onConnectionLost(responseObject) {
            this.connection_flag = false;
            if (responseObject.errorCode !== 0) {
                console.log("onConnectionLost:" + responseObject.errorMessage);
                alert("disconnect mqtt");
            }
        }

        const url = WEB_UI_URL + "/topicData";
        let data = new FormData();
        data.append("name", "test");
        fetch(url,
            {
                method: "POST",
                body: data
            })
            .then((response) => {
                if (!response.ok) {
                    throw Error(response.statusText);
                }
                return response.json();
            })
            .then((json) => {
                let user_id = json["fixeddata"]["userID"];
                let vehicle_id = json["fixeddata"]["vehicleID"];
                let Autoware = json["fixeddata"]["Autoware"];
                let User = json["fixeddata"]["User"];

                for (const key in this.topics["topicdata"]) {
                    let type = json["topicdata"][key]["type"];
                    let domain = json["topicdata"][key]["domain"];
                    let label = json["topicdata"][key]["label"];

                    this.topics["topicdata"][key]["topic_send"] = "/" + user_id + "/" + vehicle_id + "/" + type + "/"
                        + domain + "/" + label + "/" + User+ "/" + Autoware;
                    this.topics["topicdata"][key]["topic_receive"] = "/" + user_id + "/" + vehicle_id + "/" + type + "/"
                        + domain + "/" + label + "/" + Autoware + "/" + User;
                }

                // connect the client
                this.mqttClient.connect({onSuccess: onConnect.bind(this)});
            })
            .catch((e) => {
                console.error(e);
            });
    }

    static parse_topic(topic) {
        let parsed_topic = topic.split("/");
        return {
            userID: parsed_topic[1],
            vehicleID: parsed_topic[2],
            type: parsed_topic[3],
            domain: parsed_topic[4],
            label: parsed_topic[5],
            Autoware: parsed_topic[6],
            User: parsed_topic[7]
        }
    }

    //arg label:set key of this.topics.topicdata
    onPublish(label, msg){
        // console.log(label,msg);
        try {
            let message = new Paho.MQTT.Message(msg);
            message.destinationName = this.getPublishTopicName(label);
            this.mqttClient.send(message);

            // console.log("pub topic name:" + message.destinationName);
            // console.log("pub message:" + message.payloadString);
        } catch (error) {
            console.log("error:" + error);
            if ("errorPublishMessage" in this.topics["topicdata"][label]) {
                alert(this.topics["topicdata"][label]["errorPublishMessage"]);
            }
        }
    }

    //label:set key of topicdata of this.topics
    onSubscribeTopic(label) {
        const topic_name = this.getSubscribeTopicName(label);
        if (topic_name !== "") {
            this.mqttClient.subscribe(topic_name);
        }
        //console.log("topic subscribe:" + topic_name);
    }



    //label:set key of topicdata of this.topics
    onSubscribeTopicWithMethod(label, method) {
        const topic_name = this.getSubscribeTopicName(label);
        if (topic_name !== "") {
            this.mqttClient.subscribe(topic_name, {onSuccess: method});
        }
        //console.log("topic subscribe:" + topic_name);
    }

    //arg label:set key of this.topics.topicdata
    unSubscribeTopic(label) {
        const topic_name = this.getSubscribeTopicName(label);
        this.mqttClient.unsubscribe(topic_name);
        //console.log("topic unsubscribe:" + topic_name);
    }

    //arg label:set key of this.topics.topicdata
    setCallback(label, callback) {
        // console.log(label + " callback set");
        // console.log(callback);
        try {
            this.topics["topicdata"][label]["callback"] = callback;
        } catch (error) {
            console.log(label + " serror:" + error);
        }
    }

    //header and direction is fixed.These are subscribed from web server
    //body is topic of this.topics.topicdata
    getPublishTopicName(label) {
        try {
            return this.topics["topicdata"][label]["topic_send"];
        } catch (error) {
            return "";
        }
    }

    //header and direction is fixed.These are subscribed from web server
    //body is topic of this.topics.topicdata
    getSubscribeTopicName(label) {
        try {
            return this.topics["topicdata"][label]["topic_receive"];
        } catch (error) {
            return "";
        }
    }

    disConnect() {
        console.log("disconnect");
        this.mqttClient.disconnect();
    }
}
