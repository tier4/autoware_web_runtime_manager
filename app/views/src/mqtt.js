/*import React from 'react';
import ReactDOM from 'react-dom';
import Responsive, { WidthProvider } from 'react-grid-layout';
const ResponsiveReactGridLayout = WidthProvider(Responsive);
import Button from "./button";
import ROSLaunchRequest from "./roslaunch_request";
*/
import {MQTT_HOST} from "./dotenv";
import { CONST } from "./const";

export default class MqttWrapper{

    constructor(){
	
	this.toAutoware = "UtoA";
	this.fromAutoware = "AtoU";
	this.userid = "test";
	this.carid = "test";
	this.host = MQTT_HOST;
	this.port = 9091;

	//label is unique.
	this.topics = [
	    {
		domain : CONST.BUTTON_INIT.DOMAIN,
		label : CONST.BUTTON_INIT.LABEL,
		type : "buttonInit",
		callback : function(){}
	    },
	    {
		domain : CONST.BUTTON.INITIALIZATION.DOMAIN,
		label : CONST.BUTTON.INITIALIZATION.LABEL,
		type : "button",
		callback : function(){}
	    },
	    {
		domain : CONST.BUTTON.MAP.DOMAIN,
		label : CONST.BUTTON.MAP.LABEL,
		type : "button",
		callback : function(){}
	    },
	    {
		domain : CONST.BUTTON.LOCALIZATION.DOMAIN,
		label : CONST.BUTTON.LOCALIZATION.LABEL,
		type : "button",
		callback : function(){}
	    },
	    {
		domain : CONST.BUTTON.MISSION.DOMAIN,
		label : CONST.BUTTON.MISSION.LABEL,
		type : "button",
		callback : function(){}
	    },
	    {
		domain : CONST.BUTTON.MOTION.DOMAIN,
		label : CONST.BUTTON.MOTION.LABEL,
		type : "button",
		callback : function(){},
	    },
	    {
		domain : CONST.BUTTON.SENSING.DOMAIN,
		label : CONST.BUTTON.SENSING.LABEL,
		type : "button",
		callback : function(){}
	    },
	    {
		domain : CONST.BUTTON.DETECTION.DOMAIN,
		label : CONST.BUTTON.DETECTION.LABEL,
		type : "button",
		callback : function(){}
	    },
	    {
		domain : CONST.BUTTON.ROSBAG.DOMAIN,
		label : CONST.BUTTON.ROSBAG.LABEL,
		type : "button",
		callback : function(){}
	    },
	    {
		domain : CONST.BUTTON.ROSBAG.DOMAIN,
		label : CONST.BUTTON.ROSBAG_PLAY.LABEL,
		type : "button",
		callback : function(){}
	    },
	    {
		domain : CONST.BUTTON.GATEWAY.DOMAIN,
		label : CONST.BUTTON.GATEWAY.LABEL,
		type : "button",
		callback : function(){}
	    },
	    {
		domain : CONST.BUTTON.GATEWAY.DOMAIN,
		label : CONST.BUTTON.GATEWAY_ON.LABEL,
		type : "button",
		callback : function(){},
	    },		
	    {
		domain : CONST.VISUALIZATION_OBJECT.IMAGE_RAW,
		label : CONST.VISUALIZATION_OBJECT.IMAGE_RAW,
		type : "image",
		callback : function(){},
	    },
	]

	this.mqttClient = new Paho.MQTT.Client(this.host, this.port, "clientId");
	this.mqttClient.onConnectionLost = onConnectionLost;
	this.mqttClient.onMessageArrived = onMessageArrived.bind(this);

	// connect the client
	this.mqttClient.connect({onSuccess:onConnect.bind(this)});

	function onConnect() {
	    // Once a connection has been made, make a subscription.
	    console.log("mqtt onConnect");
	    for(let i in this.topics){
		const header = "/" + this.userid + "." + this.carid;
		const body = "/" + this.topics[i].type + "." + this.topics[i].domain + "." + this.topics[i].label;
		const direction = "/" + this.fromAutoware;

		this.mqttClient.subscribe(header + body + direction);
		console.log(header + body + direction);
	    }
	    this.onPublish("buttonInit","GET");
	}

	function onMessageArrived(message){
	    console.log("mqtt message arrived");

	    const topic_factor = message.destinationName.split("/");
	    const message_factor = topic_factor[2].split(".");
            const index = this.topics.findIndex(topic => topic.label === message_factor[1]);

	    if(index !== -1){
		//console.log(this.topics[index].callback)
		//console.log(index)
		this.topics[index].callback(message);
	    }
	}

	// called when the client loses its connection
	function onConnectionLost(responseObject) {
	    if (responseObject.errorCode !== 0) {
		console.log("onConnectionLost:"+responseObject.errorMessage);
		alert("余裕っす");
	    }
	}
    }

    onPublish(label,msg){
	const index = this.topics.findIndex(topic => topic.label === label);
	if(index !== -1){
	    var message = new Paho.MQTT.Message(msg);
	    const header = "/" + this.userid + "." + this.carid;
	    const body = "/" + this.topics[index].type + "." + this.topics[index].domain  + "." + this.topics[index].label;
	    const direction = "/" + this.toAutoware;
	    
	    message.destinationName = header + body + direction;
	    this.mqttClient.send(message);
	    //console.log(message.destinationName);
	    //console.log(message.payloadString);
	}
    }

    setCallback(label,callback) {
	const index = this.topics.findIndex(topic => topic.label === label);
	//console.log("image callback set")
	//console.log(index)
	if( index !== -1){
	    this.topics[index].callback = callback;
	}
    }

    disConnect(){
	console.log("disconnect");
	this.mqttClient.disonnet();
    }
}
