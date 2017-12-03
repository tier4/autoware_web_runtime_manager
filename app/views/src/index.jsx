import React from 'react';
import ReactDOM from 'react-dom';

import { CONST } from "./const";

import ButtonRGL from "./button_rgl";
import ViewRGL from './view_rgl';

import GoogleMapsView from "./googleMapsView";
import RosView from "./rosView";

import Map2DView from './map_2d_view';
import RadarView from './radar_view';
import Map3DView from './map_3d_view';
import CameraView from './camera_view';


class Index extends React.Component {
    constructor() {
        super();
        const rosView = new RosView();
        console.log(CONST);
        this.state = {
            buttonRGL: {
                nodeWidth: 4,
                nodeHeight: 3,
                cols: 25,
                rowHeight: 8,
                nodes: [
                    {id: 1, x: 0,  y: 4, w: 4, h: 3, physics: false, chosen: false, domain: CONST.BUTTON.INITIALIZATION.DOMAIN, label: CONST.BUTTON.INITIALIZATION.LABEL, display: "Initialization", span: (<span>Initialization</span>)},
                    {id: 2, x: 5,  y: 0, w: 4, h: 3, physics: false, chosen: false, domain: CONST.BUTTON.MAP.DOMAIN, label: CONST.BUTTON.MAP.LABEL, display: "Map", span: (<span>Map</span>)},
                    {id: 3, x: 10, y: 0, w: 4, h: 3, physics: false, chosen: false, domain: CONST.BUTTON.LOCALIZATION.DOMAIN, label: CONST.BUTTON.LOCALIZATION.LABEL, display: "Localization", span: (<span>Localization</span>)},
                    {id: 4, x: 15, y: 0, w: 4, h: 3, physics: false, chosen: false, domain: CONST.BUTTON.MISSION.DOMAIN, label: CONST.BUTTON.MISSION.DOMAIN, display: "Mission", span: (<span>Mission Planning</span>)},
                    {id: 5, x: 20, y: 0, w: 4, h: 3, physics: false, chosen: false, domain: "motion", label: 'motion', display: "Motion", span: (<span>Motion Planning</span>)},
                    {id: 6, x: 5,  y: 4, w: 4, h: 3, physics: false, chosen: false, domain: "sensing", label: 'sensing', display: "Sensing", span: (<span>Sensing</span>)},
                    {id: 7, x: 15, y: 4, w: 4, h: 3, physics: false, chosen: false, domain: "detection", label: 'detection', display: "Detection", span: (<span>Detection</span>)},
                    {id: 8, x: 0,  y: 0, w: 2, h: 3, physics: false, chosen: false, domain: "rosbag", label: 'rosbag', display: "ROSBAG", span: (<span>ROSBAG</span>)},
                    {id: 9, x: 2,  y: 0, w: 2, h: 3, physics: false, chosen: false, domain: "rosbag", label: 'play', display: "Play", span: (<span>play</span>)},
                    {id: 10, x: 20, y: 4, w: 3, h: 3, physics: false, chosen: false, domain: "gateway", label: 'gateway', display: "Vehicle Gateway", span: (<span>Vehicle Gateway</span>)},
                    {id: 11, x: 23, y: 4, w: 1, h: 3, physics: false, chosen: false, domain: "gateway", label: 'on', display: "On", span: (<span>On</span>)},
                ],
                edges: [
                    {from: 1, to: 2, physics: true,  label: "Initialization -> Map"},
                    {from: 1, to: 8, physics: true,  label: "Initialization -> ROSBAG"},
                    {from: 2, to: 3, physics: true,  label: "Map -> Localization"},
                    {from: 3, to: 4, physics: true,  label: "Localization -> Mission Planning"},
                    {from: 4, to: 5, physics: true,  label: "Mission Planning -> Motion Planning"},
                    {from: 1, to: 6, physics: true,  label: "Initialization -> Sensing"},
                    {from: 6, to: 3, physics: true,  label: "Sensing -> Localization"},
                    // {from: 6, to: 7, physics: true,  label: "Sensing -> Detection"},
                    {from: 7, to: 5, physics: false, label: "Detection -> Motion Planning"},
                    {from: 8, to: 9, physics: true,  label: "ROSBAG -> Play"},
                    {from: 5, to: 10, physics: true,  label: "Mission Planning -> Vehicle Gateway"},
                    {from: 10, to: 11, physics: true,  label: "Vehicle Gateway -> On"},
                ]
            },
            viewRGL: {
                cols: 25,
                rowHeight: 25,
                rosView: rosView,
                onButtonIDs: [],
                contents: {
                    "2d": {
                        triggerButtonIDs: {
                            open: 2,
                            close: null,
                        },
                        layout: {
                            i: "2d",
                            x: 0, y: 0, w: 10, h: 12,
                            isDraggable: false,
                            isResizable: false,
                        },
                        component: Map2DView,
                        viewInstance: new GoogleMapsView(),
                        visualizationObjects: {
                            "GoogleMaps": {
                                buttonID: 2,
                                data: null,
                            },
                            "vehicle": {
                                buttonID: 3,
                                topics: {
                                    vehiclePose: {
                                        ros: null,
                                        name : "/ndt_pose",
                                        messageType : 'geometry_msgs/PoseStamped',
                                    },
                                }
                            }
                        },
                        visibleObjectIDs: [],
                    },
                    [CONST.VIEW_CONTENT.MAP3D]: {
                        triggerButtonIDs: {
                            open: 2,
                            close: null,
                        },
                        layout: {
                            i: CONST.VIEW_CONTENT.MAP3D,
                            x: 10, y: 0, w: 14, h: 25,
                            isDraggable: false,
                            isResizable: false,
                        },
                        component: Map3DView,
                        viewInstance: new RosView(),
                        visualizationObjects: {
                            "PointsMap": {
                                buttonID: 2,
                                topics: {},
                            },
                            "VectorMap": {
                                buttonID: 2,
                                topics: {
                                    vectorMap: {
                                        name : "/vector_map",
                                        messageType : 'visualization_msgs/MarkerArray',
                                    }
                                },
                            },
                            "PointsRaw": {
                                buttonID: 3,
                                topics: {
                                    pointsRaw: {
                                        name: "/downsampled_points_raw",
                                        messageType: "sensor_msgs/PointCloud2"
                                    }
                                },
                            },
                            "Vehicle": {
                                buttonID: 3,
                                topics: {
                                    tf: {
                                        name: "/tf",
                                        messageType: "tf2_msgs/TFMessage"
                                    },
                                    vehiclePose: {
                                        name : "/ndt_pose",
                                        messageType : 'geometry_msgs/PoseStamped',
                                    },
                                },
                            },
                            "Waypoints": {
                                buttonID: 4,
                                topics: {
                                    waypoints: {
                                        name : "/lane_waypoints_array",
                                        messageType : 'autoware_msgs/LaneArray',
                                    }
                                },
                            },
                            [CONST.VISUALIZATION_OBJECT.TARGETS]: {
                                buttonID: 5,
                                topics: {
                                    nextTarget: {
                                        name : "/downsampled_next_target_mark",
                                        messageType : 'visualization_msgs/Marker',
                                    },
                                    trajectoryCircle: {
                                        name : "/downsampled_trajectory_circle_mark",
                                        messageType : 'visualization_msgs/Marker',
                                    }
                                },
                            }
                        },
                        visibleObjectIDs: [],
                    },
                    "camera": {
                        triggerButtonIDs: {
                            open: 6,
                            close: null,
                        },
                        layout: {
                            i: "camera",
                            x: 0, y: 12, w: 10, h: 13,
                            isDraggable: false,
                            isResizable: false,
                        },
                        component: CameraView,
                        viewInstance: null,
                        visualizationObjects: {},
                        visibleObjectIDs: [],
                    },
//                    "radar": {
//                        triggerButtonIDs: {
//                            open: 6,
//                            close: 3,
//                        },
//                        component: RadarView
//                    },
                },
            },
        };
    }
    render() {
        return (
            <div>
                <ButtonRGL
                    structure={this.state.buttonRGL}
                    updateStructure={this.updateButtonRGLStructure.bind(this)}
                />
                <ViewRGL
                    structure={this.state.viewRGL}
                    updateStructure={this.updateViewRGLStructure.bind(this)}
                />
            </div>
        );
    }
    updateButtonRGLStructure(nextStructure) {
        this.setState({buttonRGL: nextStructure});
        this.updateViewRGLVisibility();
    }
    updateViewRGLStructure(nextStructure) {
        this.setState({viewRGL: nextStructure});
    }
    updateViewRGLVisibility() {
        console.log("index.updateViewRGLVisibility");
        const viewRGL = this.state.viewRGL;
        const buttonRGL = this.state.buttonRGL;

        viewRGL.onButtonIDs = [];
        for(const button of buttonRGL.nodes) {
            if( button.physics && button.chosen ){
                viewRGL.onButtonIDs.push(button.id);
            }
        }

        for(const contentID in viewRGL.contents) {
            viewRGL.contents[contentID].visibleObjectIDs = [];
            for(const objectID in viewRGL.contents[contentID].visualizationObjects ){
                if(viewRGL.onButtonIDs.includes(viewRGL.contents[contentID].visualizationObjects[objectID].buttonID)){
                    viewRGL.contents[contentID].visibleObjectIDs.push(objectID);
                }
            }
        }
        this.setState({viewRGL: viewRGL});
    }
}

ReactDOM.render(
    <Index/>,
    document.getElementById('content')
);

