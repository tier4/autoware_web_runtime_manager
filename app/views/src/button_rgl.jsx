import React from 'react';

import Responsive, {WidthProvider} from 'react-grid-layout';
import Button from "./button";
import {CONST} from "./const";
import SettingModal from "./setting_modal";
import MqttWrapper from "./mqtt";


const ResponsiveReactGridLayout = WidthProvider(Responsive);


class GridSizeWrapper extends React.Component {
    render() {
        const that = this;
        const newChildren = React.Children.map(
            this.props.children,
            function (child) {
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
    constructor(props) {
        super(props);
        this.state = {
            modalIsOpen: false,
            nodeID: -1
        };

    }

    openModal(nodeID) {
        this.setState({
            modalIsOpen: true,
            nodeID: nodeID
        });
    }

    saveSettingModal(file_name) {
        this.props.mqttClient.onPublish(CONST.SETTING_SAVE.LABEL, file_name);
    }

    loadSettingModal(file_name) {
        this.props.mqttClient.onPublish(CONST.SETTING_LOAD.LABEL, file_name);
    }

    submitModeModal(nodeID) {
        console.log("mode submit", nodeID);
        const structure = this.props.structure;
        const index = structure.nodes.findIndex(node => node.id === nodeID);
        structure.nodes = this.getUpdatedNodes(
            structure.nodes[index].id,
            !structure.nodes[index].on,
            structure.nodes);
        const message = JSON.stringify({
            on: structure.nodes[index].on ? "on" : "off",
            settingParams: this.props.settingParams
        });
        this.props.mqttClient.onPublish(structure.nodes[index].label, message);
        structure.nodes[index].span = (<span>{(structure.nodes[index].on ? "loading.." : "killing..")}</span>);
        this.setState({
            modalIsOpen: false,
            nodeID: -1
        });
        this.props.updateStructure(structure);
    }

    submitSettingModal(nodeID) {
        console.log("mode submit", nodeID);
        const structure = this.props.structure;
        const index = structure.nodes.findIndex(node => node.id === nodeID);
        const message = JSON.stringify(this.props.settingParams);
        this.props.mqttClient.onPublish(structure.nodes[index].label, message);
        this.setState({
            modalIsOpen: false,
            nodeID: -1
        });
        this.props.updateStructure(structure);
    }

    closeModal() {
        this.setState({
            modalIsOpen: false,
            nodeID: -1
        });
    }

    getLayout() {
        const layout = [];
        for (const node of this.props.structure.nodes) {
            layout.push(
                {
                    i: node.id + "",
                    x: node.x,
                    y: node.y,
                    w: node.w,
                    h: node.h,
                    static: true
                }
            );
        }
        return layout;
    }

    getButtons() {
        const buttons = [];
        for (const node of this.props.structure.nodes) {
            let buttonState = null;
            if (node.on) {
                buttonState = "on";
            }
            else {
                buttonState = "off";
            }
            if (node.isLoading) {
                buttonState = "disabled";
            }
            if (node.isKilling) {
                buttonState = "disabled";
            }
            if (!node.enabled) {
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
        this.setInitializeButtonRGLState();
        this.setSaveLoadCallback();
        this.setButtonCallback();

    }

    render() {
        return (
            <div>
                <SettingModal
                    isActive={this.state.modalIsOpen}
                    onClose={this.closeModal.bind(this)}
                    onModeSubmit={this.submitModeModal.bind(this)}
                    onSettingSubmit={this.submitSettingModal.bind(this)}
                    onSaveSetting={this.saveSettingModal.bind(this)}
                    onLoadSetting={this.loadSettingModal.bind(this)}
                    updateSettingParamsStructure={this.props.updateSettingParamsStructure}
                    settingParams={this.props.settingParams}
                    nodeID={this.state.nodeID}
                    structure={this.props.structure}
                />

                <ResponsiveReactGridLayout
                    className="layout"
                    layout={this.getLayout()}
                    cols={this.props.structure.cols}
                    rowHeight={this.props.structure.rowHeight}
                >
                    {this.getButtons()}
                </ResponsiveReactGridLayout>
            </div>
        );
    }

    setInitializeButtonRGLState() {
        //const date = new Date();
        //const url = WEB_UI_URL+"/getRTMStatus?date="+date.getTime().toString();

        let initMethod = function (message) {
            //console.log(message.payloadString);
            const json = JSON.parse(message.payloadString);
            console.log("initializeButtonRGLState", json);
            const structure = this.props.structure;

            let rtm_stateus = json["rtm_status"];
            let parameter_info = json["parameter_info"];

            for (const topic_name of Object.keys(rtm_stateus)) {
                //console.log("topic:" + json[topic_name]);
                const index = this.props.structure.nodes.findIndex(function (x) {
                    return x.label === topic_name;
                });
                if (index !== -1) {
                    structure.nodes[index].enabled = rtm_stateus[topic_name]["enable"];
                }
            }

            for (const topic_name of Object.keys(rtm_stateus)) {
                //console.log("topic:" + json[topic_name]);
                const index = this.props.structure.nodes.findIndex(function (x) {
                    return x.label === topic_name;
                });
                if (index !== -1) {
                    if (rtm_stateus[topic_name]["mode"] === "on" && index !== -1) {
                        structure.nodes = this.getUpdatedNodes(
                            this.props.structure.nodes[index].id,
                            !this.props.structure.nodes[index].on,
                            this.props.structure.nodes);
                    }
                }
            }


            let settingParams = parameter_info["settingParams"];

            this.props.updateSettingParamsStructure(settingParams);
            this.props.updateStructure(structure);
        };
        this.props.mqttClient.setCallback("buttonInit", initMethod.bind(this));
    }

    setSaveLoadCallback() {
        let saveMethod = function (message) {
            //console.log(message.payloadString);
            const save_file_list = JSON.parse(message.payloadString);
            console.log("save file list", save_file_list);
            if (save_file_list["settingParams"] !== "error") {
                this.props.updateSettingParamsStructure(save_file_list["settingParams"]);
            } else {
                alert("Fail to save.");
            }
        };
        this.props.mqttClient.setCallback("settingSave", saveMethod.bind(this));

        let loadMethod = function (message) {
            //console.log(message.payloadString);
            const settingParams = JSON.parse(message.payloadString);

            if (settingParams["settingParams"] !== "error") {
                this.props.updateSettingParamsStructure(settingParams["settingParams"]);
            } else {
                alert("Fail to load.");
            }
        };
        this.props.mqttClient.setCallback("settingLoad", loadMethod.bind(this));

    }

    setButtonCallback() {
        let buttonMethod = function (message) {
            //console.log(message.payloadString);
            let parsed_topic = MqttWrapper.parse_topic(message.destinationName);
            const index = this.props.structure.nodes.findIndex(node => node.label === parsed_topic["label"]);

            if (message.payloadString === "ok") {
                this.props.structure.nodes[index].span = (<span>{this.props.structure.nodes[index].display}</span>);
            } else {
                this.props.structure.nodes[index].span = (<span>error</span>);
            }
            this.props.updateStructure(this.props.structure);
        };

        for (const node of this.props.structure.nodes) {
            this.props.mqttClient.setCallback(node.label, buttonMethod.bind(this));
        }
    }

    onClickButton(nodeID) {
        const index = this.props.structure.nodes.findIndex(node => node.id === nodeID);
        if (this.props.structure.nodes[index].enabled) {
            const structure = this.props.structure;
            if (structure.nodes[index].label === CONST.BUTTON.SETTING.LABEL) {
                if (!structure.nodes[index].on) {
                    this.openModal(nodeID);
                }
            } else if (structure.nodes[index].label === CONST.BUTTON.ALL_ACTIVATION.LABEL) {
                if (!structure.nodes[index].on) {
                    this.openModal(nodeID);
                } else {
                    structure.nodes = this.getUpdatedNodes(
                        nodeID,
                        !structure.nodes[index].on,
                        structure.nodes);
                    const label = structure.nodes[index].label;
                    const message = JSON.stringify({
                        on: structure.nodes[index].on ? "on" : "off",
                        settingParams: this.props.settingParams
                    });

                    this.props.mqttClient.onPublish(label, message);
                    structure.nodes[index].span = (
                        <span>{(structure.nodes[index].on ? "loading.." : "killing..")}</span>);
                }
            } else if (structure.nodes[index].label === CONST.REDISPLAY.LABEL) {
                location.reload(true);
            } else {
                structure.nodes = this.getUpdatedNodes(
                    nodeID,
                    !structure.nodes[index].on,
                    structure.nodes);
                // set callback handlers
                const label = structure.nodes[index].label;
                const message = structure.nodes[index].on ? "on" : "off";
                this.props.mqttClient.onPublish(label, message);
                structure.nodes[index].span = (<span>{(structure.nodes[index].on ? "loading.." : "killing..")}</span>);
            }
            this.props.updateStructure(structure);
        }
    }

    getUpdatedNodes(nodeID, on, nodes) {
        const index = nodes.findIndex(node => node.id === nodeID);
        nodes[index].on = on;

        if (index === nodes.findIndex(node => node.label === CONST.BUTTON.ALL_ACTIVATION.LABEL)) {
            this.setOnNode(nodes, index);
        }
        this.setEnable(nodes);
        /*
        if (nodes[nodes.findIndex(node => node.label === CONST.ROSBAG_MODE.LABEL)].on === false &&
        nodes[nodes.findIndex(node => node.label === CONST.SIM_MODE.LABEL)].on === false &&
        nodes[nodes.findIndex(node => node.label === CONST.DRIVE_MODE.LABEL)].on === false){
            nodes[nodes.findIndex(node => node.label === CONST.BUTTON.INITIALIZATION.LABEL)].enabled = true;
        }else{
            nodes[nodes.findIndex(node => node.label === CONST.BUTTON.INITIALIZATION.LABEL)].enabled = true;
        }
        */
        if (nodes[nodes.findIndex(node => node.label === CONST.BUTTON.ALL_ACTIVATION.LABEL)].on === false) {
            for (const id of nodes[nodes.findIndex(node => node.label === CONST.BUTTON.ALL_ACTIVATION.LABEL)].required.forDisable.on) {
                if (nodes[nodes.findIndex(node => node.id === id)].on === true) {
                    nodes[nodes.findIndex(node => node.label === CONST.BUTTON.ALL_ACTIVATION.LABEL)].enabled = true;
                }
            }
        }

        return nodes
    }

    setEnable(nodes) {
        let changeFlag = true;
        while (changeFlag) {
            changeFlag = false;
            for (const nodeIndex in nodes) {
                const node = nodes[nodeIndex];
                let enabled = true;
                for (const requiredNodeID of node.required.forEnable.off) {
                    if (nodes[nodes.findIndex(node => node.id === requiredNodeID)].on) {
                        enabled = false;
                        break;
                    }
                }
                if (enabled) {
                    for (const requiredNodeID of node.required.forEnable.disable) {
                        if (nodes[nodes.findIndex(node => node.id === requiredNodeID)].enabled) {
                            enabled = false;
                            break;
                        }
                    }
                    if (enabled) {
                        for (const requiredNodeID of node.required.forEnable.enable) {
                            if (!nodes[nodes.findIndex(node => node.id === requiredNodeID)].enabled) {
                                enabled = false;
                                break;
                            }
                        }
                        if (enabled) {
                            for (const requiredNodeID of node.required.forEnable.on) {
                                if (!nodes[nodes.findIndex(node => node.id === requiredNodeID)].on) {
                                    enabled = false;
                                    break;
                                }
                            }
                        }
                    }
                }
                if (node.enabled !== enabled) {
                    nodes[nodeIndex].enabled = enabled;
                    changeFlag = true;
                    break;
                }
            }
        }
        return nodes
    }

    setOnNode(nodes, index) {
        for (const nodeIndex in nodes) {
            const node = nodes[nodeIndex];
            let on_flag = node.on;
            for (const requiredNodeID of node.required.forOn.on) {
                on_flag = nodes[nodes.findIndex(node => node.id === requiredNodeID)].on;
            }

            if (node.on !== on_flag && Number(nodeIndex) !== index) {
                console.log(index, nodeIndex);
                nodes[nodeIndex].on = on_flag;
            }
        }

        return nodes
    }

}
