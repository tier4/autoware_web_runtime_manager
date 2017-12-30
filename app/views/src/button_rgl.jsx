import React from 'react';
import ReactDOM from 'react-dom';
import Responsive, { WidthProvider } from 'react-grid-layout';
const ResponsiveReactGridLayout = WidthProvider(Responsive);
import Button from "./button";
import ROSLaunchRequest from "./roslaunch_request";
import {WEB_UI_URL} from "./dotenv";

class GridSizeWrapper extends React.Component {
    render() {
        const that = this;
        const newChildren = React.Children.map(
            this.props.children,
            function(child) {
                return React.cloneElement(
                    child,
                    {
                        width: that.props.style.width,
                        height: that.props.style.height
                    }
                )
            }
        );
        return (
            <div
                {...this.props}
            >
                {newChildren}
            </div>
        );
    }
}


export default class ButtonRGL extends React.Component {
    getLayout() {
        const layout = [];
        for(const node of this.props.structure.nodes) {
            layout.push(
                {
                    i: node.id + "",
                    x: node.x,
                    y: node.y,
                    w: node.w,
                    h: node.h,
                    static: true}
            );
        }
        return layout;
    }
    getButtons() {
        const buttons = [];
        for(const node of this.props.structure.nodes) {
            let buttonState = null;
            if(node.on) {
                buttonState = "on";
            }
            else{
                buttonState = "off";
            }
            if(node.isLoading) {
                buttonState = "disabled";
            }
            if(node.isKilling) {
                buttonState = "disabled";
            }
            if(!node.enabled) {
                buttonState = "disabled";
            }
            buttons.push((
                <GridSizeWrapper key={node.id}>
                    <Button
                        span={node.span}
                        buttonState={buttonState}
                        onClick={this.onClickButton.bind(this, node.id)}
                    />
                </GridSizeWrapper>
            ));
        }
        return buttons;
    }
    componentWillMount() {
        this.initializeButtonRGLState();

	//mqtt callback method creating
	const mqttClient = this.props.mqttClient;
	
	var buttonMethod = function(message){
	    //console.log(message.payloadString);
	    const topic_factor = message.destinationName.split("/");
	    const message_factor = topic_factor[2].split(".");
            const index = this.props.structure.nodes.findIndex(node => node.label === message_factor[2]);   
	    //console.log(index);
	    
	    if(message.payloadString === "ok"){
		this.props.structure.nodes[index].span = (<span>{this.props.structure.nodes[index].display}</span>);
	    }else{
		this.props.structure.nodes[index].span = (<span>error</span>);
	    }
	    this.props.updateStructure(this.props.structure);
	};

	for(const node of this.props.structure.nodes){
	    mqttClient.setCallback(node.label,buttonMethod.bind(this));
	}
    }
    
    render() {
        return (
            <ResponsiveReactGridLayout
                className="layout"
                layout={this.getLayout()}
                cols={this.props.structure.cols}
                rowHeight={this.props.structure.rowHeight}
            >
                {this.getButtons()}
            </ResponsiveReactGridLayout>
        );
    }
    initializeButtonRGLState() {
        //const date = new Date();
	//todo:change mqtt
        //const url = WEB_UI_URL+"/getRTMStatus?date="+date.getTime().toString();

	var initMethod = function(message){
	    //console.log(message.payloadString);
	    var json = JSON.parse(message.payloadString);
            //console.log("initializeButtonRGLState", json);
            const structure = this.props.structure;
            for(const domain of Object.keys(json)){
                for(const label of Object.keys(json[domain])){
                    const index = this.props.structure.nodes.findIndex(function(x) { return x.domain == domain && x.label == label; });
		    if(index !== -1){
			structure.nodes[index].enabled = json[domain][label]["enable"];
		    }
                }
            }
            for(const domain of Object.keys(json)){
                for(const label of Object.keys(json[domain])){
                    const index = this.props.structure.nodes.findIndex(function(x) { return x.domain == domain && x.label == label; });
                    if(json[domain][label]["mode"]=="on" && index !== -1){
                        structure.nodes = this.getUpdatedNodes(
                            this.props.structure.nodes[index].id,
                            !this.props.structure.nodes[index].on,
      this.props.structure.nodes);
                    }
                }
            }
            this.props.updateStructure(structure);
        };
	this.props.mqttClient.setCallback("buttonInit",initMethod.bind(this));
		
    }
    onClickButton(nodeID) {
        const index = this.props.structure.nodes.findIndex(node => node.id === nodeID);
        if(this.props.structure.nodes[index].enabled) {
            const structure = this.props.structure;
            structure.nodes = this.getUpdatedNodes(
                nodeID,
                !this.props.structure.nodes[index].on,
                this.props.structure.nodes);

            const nodeDomain = structure.nodes[index].domain;
            const nodeLabel = structure.nodes[index].label;
            const nodeDisplay = structure.nodes[index].display;
            const url = WEB_UI_URL+"/roslaunch/"+nodeDomain+"/"+nodeLabel+"/"+nodeID+"/"+(this.props.structure.nodes[index].on ? "on" : "off");

	    /*
            structure.nodes[index].span = (<ROSLaunchRequest
                url={url}
                errorCallback={() => { return (<span>error</span>); }}
                isLoadingCallback={() => {
                    return (<span>{(this.props.structure.nodes[index].on ? "loading.." : "killing..")}</span>); }
                }
                responseCallback={() => {
                    if(index==0 && !this.props.structure.nodes[index].on) {
                        location.reload();
                    }
                    return (<span>{nodeDisplay}</span>);
                }}
                defaultCallback={() => { return (<span>{nodeDisplay}</span>); }}
					   />);
	    */

	    // set callback handlers
	    const label = structure.nodes[index].label;
	    const message = structure.nodes[index].on ? "on" : "off";
	    this.props.mqttClient.onPublish(label,message);
	    structure.nodes[index].span = (<span>{(structure.nodes[index].on ? "loading.." : "killing..")}</span>);

	    
            /*
            let that = this;
            const xhttpRequest = new XMLHttpRequest();
            xhttpRequest.onreadystatechange = function() {
                if (this.readyState == 4 && this.status == 200) {
                    structure.nodes[index].isLoading = false;
                    structure.nodes[index].on = !structure.nodes[index].on;
                    if(index==0 && structure.nodes[index].on) {
                        location.reload();
                    }
                    that.props.updateStructure(structure);
                }
            };
            if(structure.nodes[index].on){
                structure.nodes[index].isLoading = true;
            }
            else{
                structure.nodes[index].isKilling = true;
            }
            xhttpRequest.open("GET", url, true);
            xhttpRequest.send();
            */

            this.props.updateStructure(structure);
        }
    }
    getUpdatedNodes(nodeID, on, nodes) {
        const index = nodes.findIndex(node => node.id === nodeID);
        nodes[index].on = on;
        let changeFlag = true;
        while(changeFlag) {
            changeFlag=false;
            for(const nodeIndex in nodes) {
                const node = nodes[nodeIndex];
                let enabled = true;
                for(const requiredNodeID of node.required.forEnable.off) {
                    if(nodes[nodes.findIndex(node => node.id === requiredNodeID)].on) {
                        enabled = false;
                        break;
                    }
                }
                if(enabled) {
                    for(const requiredNodeID of node.required.forEnable.disable) {
                        if(nodes[nodes.findIndex(node => node.id === requiredNodeID)].enabled) {
                            enabled = false;
                            break;
                        }
                    }
                    if(enabled) {
                        for(const requiredNodeID of node.required.forEnable.enable) {
                            if(!nodes[nodes.findIndex(node => node.id === requiredNodeID)].enabled) {
                                enabled = false;
                                break;
                            }
                        }
                        if(enabled) {
                            for(const requiredNodeID of node.required.forEnable.on) {
                                if(!nodes[nodes.findIndex(node => node.id === requiredNodeID)].on) {
                                    enabled = false;
                                    break;
                                }
                            }
                        }
                    }
                }
                if(node.enabled != enabled) {
                    nodes[nodeIndex].enabled = enabled;
                    changeFlag=true;
                    break;
                }
            }
        }
        return nodes;
    }
}
