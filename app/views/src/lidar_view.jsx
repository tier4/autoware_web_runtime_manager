import React from 'react';
import ReactResizeDetector from 'react-resize-detector';
import { WEB_UI_URL } from "./dotenv";
import RosView from "./rosView";

export default class LidarView extends React.Component {
    constructor() {
        super();
        this.state = {};
    }
    render() {
        // console.log("LidarView.render()", this.props);
        return (
            <ReactResizeDetector handleWidth handleHeight onResize={(w, h)=>this.onDetectParentResize(w, h)} />
        );
    }
    runViewInstance(props) {
//            far: 2000,
//            cameraPose: {x: -30, y: -20, z: 30},
        const viewInstance = props.viewInstance;
        viewInstance.elementID = props.parentId;
        viewInstance.width = parseInt(props.width);
        viewInstance.height = parseInt(props.height);
        viewInstance.visualizationObjects = props.visualizationObjects;
        viewInstance.prepare();
        viewInstance.onGetPointsRaw();
        this.setState({viewInstance: viewInstance});
    }
    componentDidMount() {
        console.log("LidarView.componentDidMount()", this.props);
        this.runViewInstance(this.props);
    }
    componentWillReceiveProps(nextProps) {
        console.log("LidarView.componentWillReceiveProps()", nextProps);
        this.runViewInstance(nextProps);
    }
    onDetectParentResize(w, h) {
        console.log("LidarView.onDetectParentResize", w, h);

        if(typeof this.state.viewInstance !== 'undefined'){
            this.state.viewInstance.resize(w, h);
        }
    }
    componentWillUnmount(){
        console.log("LidarView.componentWillUnmount");
        this.props.viewInstance.reset();
    }
}
