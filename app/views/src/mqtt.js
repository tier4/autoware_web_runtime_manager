/*import React from 'react';
  import ReactDOM from 'react-dom';
  import Responsive, { WidthProvider } from 'react-grid-layout';
  const ResponsiveReactGridLayout = WidthProvider(Responsive);
  import Button from "./button";
  import ROSLaunchRequest from "./roslaunch_request";
*/
import {WEB_UI_URL,MQTT_HOST_NAME,MQTT_PORT} from "./dotenv";
import { CONST } from "./const";

export default class MqttWrapper{

    constructor(){
	
	this.host = MQTT_HOST_NAME;
	this.port = parseInt(MQTT_PORT);

	//label is unique.
	this.topics = {
	    "fixeddata" : {
		"userid" : "",
		"carid" : "",
		"toAutoware" : "",
		"fromAutoware" : ""
	    },
	    "topicdata":{
		"buttonInit" : {
		    "domain" : CONST.BUTTON_INIT.DOMAIN,
		    "label" : CONST.BUTTON_INIT.LABEL,
		    "type" : "buttonInit",
		    "topic" : "",
		    "erroePublishMessage" : "buttonInit topic can not publish.",
		    "callback" : function(){}
		},
		"initialization" : {
		    "domain" : CONST.BUTTON.INITIALIZATION.DOMAIN,
		    "label" : CONST.BUTTON.INITIALIZATION.LABEL,
		    "type" : "button",
		    "topic" : "",
		    "erroePublishMessage" : "initialization topic can not publish.",
		    "callback" : function(){}
		},
		"map" : {
		    "domain" : CONST.BUTTON.MAP.DOMAIN,
		    "label" : CONST.BUTTON.MAP.LABEL,
		    "type" : "button",
		    "topic" : "",
		    "erroePublishMessage" : "map topic can not publish.",
		    "callback" : function(){}
		},
		"localization" : {
		    "domain" : CONST.BUTTON.LOCALIZATION.DOMAIN,
		    "label" : CONST.BUTTON.LOCALIZATION.LABEL,
		    "type" : "button",
		    "topic" : "",
		    "erroePublishMessage" : "localization topic can not publish.",
		    "callback" : function(){}
		},
		"mission" : {
		    "domain" : CONST.BUTTON.MISSION.DOMAIN,
		    "label" : CONST.BUTTON.MISSION.LABEL,
		    "type" : "button",
		    "topic" : "",
		    "erroePublishMessage" : "mission topic can not publish.",
		    "callback" : function(){}
		},
		"motion" : {
		    "domain" : CONST.BUTTON.MOTION.DOMAIN,
		    "label" : CONST.BUTTON.MOTION.LABEL,
		    "type" : "button",
		    "topic" : "",
		    "erroePublishMessage" : "motion topic can not publish.",
		    "callback" : function(){},
		},
		"sensing" : {
		    "domain" : CONST.BUTTON.SENSING.DOMAIN,
		    "label" : CONST.BUTTON.SENSING.LABEL,
		    "type" : "button",
		    "topic" : "",
		    "erroePublishMessage" : "sensing topic can not publish.",
		    "callback" : function(){}
		},
		"detection" : {
		    "domain" : CONST.BUTTON.DETECTION.DOMAIN,
		    "label" : CONST.BUTTON.DETECTION.LABEL,
		    "type" : "button",
		    "topic" : "",
		    "erroePublishMessage" : "detection topic can not publish.",
		    "callback" : function(){}
		},
		"rosbag" : {
		    "domain" : CONST.BUTTON.ROSBAG.DOMAIN,
		    "label" : CONST.BUTTON.ROSBAG.LABEL,
		    "type" : "button",
		    "topic" : "",
		    "erroePublishMessage" : "rosbag topic can not publish.",
		    "callback" : function(){}
		},
		"play" : {
		    "domain" : CONST.BUTTON.ROSBAG.DOMAIN,
		    "label" : CONST.BUTTON.ROSBAG_PLAY.LABEL,
		    "type" : "button",
		    "topic" : "",
		    "erroePublishMessage" : "rosbag_play topic can not publish.",
		    "callback" : function(){}
		},
		"gateway" : {
		    "domain" : CONST.BUTTON.GATEWAY.DOMAIN,
		    "label" : CONST.BUTTON.GATEWAY.LABEL,
		    "type" : "button",
		    "topic" : "",
		    "erroePublishMessage" : "gateway topic can not publish.",
		    "callback" : function(){}
		},
		"on" : {
		    "domain" : CONST.BUTTON.GATEWAY.DOMAIN,
		    "label" : CONST.BUTTON.GATEWAY_ON.LABEL,
		    "type" : "button",
		    "topic" : "",
		    "erroePublishMessage" : "gateway_on topic can not publish.",
		    "callback" : function(){}
		},		
		"image" : {
		    "domain" : CONST.VISUALIZATION_OBJECT.IMAGE_RAW,
		    "label" : CONST.VISUALIZATION_OBJECT.IMAGE_RAW,
		    "type" : "image",
		    "topic" : "",
		    "callback" : function(){}
		},
	    }
	}

	this.mqttClient = new Paho.MQTT.Client(this.host, this.port, "clientId");

	this.mqttClient.onConnectionLost = onConnectionLost;
	this.mqttClient.onMessageArrived = onMessageArrived.bind(this);
	
	function onConnect() {
	    console.log("mqtt onConnect");
	    
	    // Once a connection has been made, make a subscription.
	    const head = this.topics["fixeddata"]
	    const header = "/" + head["userid"] + "." + head["carid"];
	    const direction = "/" + head["fromAutoware"];
	    
	    for(const key of Object.keys(this.topics["topicdata"])){
		const body = "/" + this.topics["topicdata"][key]["topic"];
		this.mqttClient.subscribe(header + body + direction);
		console.log(header + body + direction);
	    }
	    
	    this.onPublish("buttonInit","GET");
	}


	function onMessageArrived(message){
	    console.log("mqtt message arrived" + message.destinationName);

	    const topic_factor = message.destinationName.split("/");
	    const message_factor = topic_factor[2].split(".");
	    const index = message_factor[0];
	    this.topics["topicdata"][index].callback(message);
	}
	
	// called when the client loses its connection
	function onConnectionLost(responseObject) {
	    if (responseObject.errorCode !== 0) {
		console.log("onConnectionLost:"+responseObject.errorMessage);
		alert("余裕っす");
	    }
	}

	
        const url = WEB_UI_URL+"/topicData";
	const obj = {name: "test"};
	var data = new FormData();
	data.append("name","test")
	fetch(url,
	      {
		  method:"POST",
		  body: data
	      })
	    .then((response) => {
		console.log(response)
		if(!response.ok) {
                    throw Error(response.statusText);
		}
		return response.json();
            })
            .then((json) => {
		console.log("initializeButtonRGLState", json);
		for(const key in this.topics["topicdata"]){
		    this.topics["fixeddata"]["userid"] = json["fixeddata"]["userid"];
		    this.topics["fixeddata"]["carid"] = json["fixeddata"]["carid"];
		    this.topics["fixeddata"]["toAutoware"] = json["fixeddata"]["toAutoware"];
		    this.topics["fixeddata"]["fromAutoware"] = json["fixeddata"]["fromAutoware"];
		    		    
		    if(key !== "image"){
			this.topics["topicdata"][key]["topic"] = json["button_topics"][key]["topic"];
			
		    }else{
			this.topics["topicdata"][key]["topic"] = json["image_topics"][key]["topic"];
		    }
		}
		
		
		// connect the client
		this.mqttClient.connect({onSuccess:onConnect.bind(this)});
            })
	    .catch((e) => { console.error(e);} );

    }

    onPublish(label,msg){
	try{
	    var message = new Paho.MQTT.Message(msg);
	    const head = this.topics["fixeddata"]
	    const header = "/" + head["userid"] + "." + head["carid"];
	    const body = "/" + this.topics["topicdata"][label]["topic"];
	    const direction = "/" + head["toAutoware"];
	    
	    message.destinationName = header + body + direction;
	    this.mqttClient.send(message);
	    //console.log(message.destinationName);
	    //console.log(message.payloadString);
	}catch(error){
	    console.log("error:" + error);
	    if("errorPublishMessage" in this.topics["topicdata"][label]){
		alert(this.topics["topicdata"][label]["errorPublishMessage"]);
	    }
	}
    }

    setCallback(label,callback) {
	//console.log("image callback set")
	//console.log(index)
	try{
	    if(label !== "image"){
		this.topics["topicdata"][label]["callback"] = callback;
	    }else{
		this.topics["topicdata"][label]["callback"] = callback;
	    }
	}catch(error){
	    console.log("error:" + error);
	}
    }

    disConnect(){
	console.log("disconnect");
	this.mqttClient.disonnet();
    }
}
