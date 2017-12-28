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
	    image: document.createElement("img")
        };

	var imageMethod = function(message){
	    this.state.image.src = message.payloadBytes;
	    this.setState(this.state);
	};
	
	this.props.mqtt_client.setCallback("ImageRaw",imageMethod.bind(this));

    }
    render() {
        //const url = ROSVIDEO_URL+"/stream?topic="+this.props.visualizationObjects[CONST.VISUALIZATION_OBJECT.IMAGE_RAW].topics[CONST.TOPIC.IMAGE_RAW.NAME].name+"&width="+this.state.width+"&height="+this.state.height+"&type=mjpeg&quality=20";
	//console.log(url);
        return (
            <div>
              <ReactResizeDetector handleWidth handleHeight onResize={(w, h)=>this.onDetectParentResize(w, h)}/>
		<img id="img"></img>
            </div>
        );
    }
    onDetectParentResize(w, h) {
        if(6 < Math.abs(h-this.state.height)) {
            this.setState({height: h});
        }
        this.setState({width: w}); 
    }
}

