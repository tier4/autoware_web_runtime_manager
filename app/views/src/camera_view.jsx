import React from 'react';
import ReactResizeDetector from 'react-resize-detector';
import { ROSVIDEO_URL } from "./dotenv";
import { CONST } from "./const";

export default class CameraView extends React.Component {
    constructor(props) {
        super(props);
        this.state = {
            width: parseInt(this.props.width, 10),
            height: parseInt(this.props.height, 10),
	    image:""
        };

	var pub_msg = this.props.visualizationObjects[CONST.VISUALIZATION_OBJECT.IMAGE_RAW].topics[CONST.TOPIC.IMAGE_RAW.NAME].name + "."+this.state.width + "." + this.state.height+".20";
	this.props.mqttClient.onPublish("ImageRaw",pub_msg);
	//console.log(pub_msg);
	
	var imageMethod = function(message){
	    this.state.image = 'data:image/jpeg;base64,'+message.payloadString;
	    this.setState(this.state);
	};
	
	this.props.mqttClient.setCallback("ImageRaw",imageMethod.bind(this));

    }
    render() {
        //const url = ROSVIDEO_URL+"/stream?topic="+this.props.visualizationObjects[CONST.VISUALIZATION_OBJECT.IMAGE_RAW].topics[CONST.TOPIC.IMAGE_RAW.NAME].name+"&width="+this.state.width+"&height="+this.state.height+"&type=mjpeg&quality=20";
	//console.log(url);
        return (
            <div>
              <ReactResizeDetector handleWidth handleHeight onResize={(w, h)=>this.onDetectParentResize(w, h)}/>
		<img src={this.state.image}></img>
            </div>
        );
    }
    onDetectParentResize(w, h) {
        if(6 < Math.abs(h-this.state.height)) {
            this.setState({height: h});
        }
        this.setState({width: w}); 
    }
    componentWillUpdate(nectProps,nextStructure){
	if(nextStructure.width !== this.state.width || nextStructure.height !== this.state.height){
	    var pub_msg = this.props.visualizationObjects[CONST.VISUALIZATION_OBJECT.IMAGE_RAW].topics[CONST.TOPIC.IMAGE_RAW.NAME].name + "."+nextStructure.width + "." + nextStructure.height+".20";
	    this.props.mqttClient.onPublish("ImageRaw",pub_msg);
	    //console.log(pub_msg);
	}
    }
}

