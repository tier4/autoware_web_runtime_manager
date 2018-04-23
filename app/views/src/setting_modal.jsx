import React from 'react';
import Modal from 'react-modal';
import OverviewSetting from "./overview_setting";
import ModeSetting from "./mode_setting";
import {CONST} from "./const";

const customStyles = {
    content: {
        top: '10%',
        left: '20%',
        right: 'auto',
        bottom: 'auto',
        height: "80%",
        width: "60%",
        marginRight: '-20%',
        //transform: 'translate(-50%, -50%)',
        overflow: 'scroll'
    },
    overlay: {
        backgroundColor: 'black'
    }
};

Modal.setAppElement(document.getElementById('content'));

export default class SettingModal extends React.Component {

    constructor(props) {
        super(props);
    }

    getDetailSetting() {
        if (this.props.nodeID !== -1) {
            let index = this.props.structure.nodes.findIndex(node => node.id === this.props.nodeID);
            if (this.props.structure.nodes[index].label === CONST.BUTTON.ALL_ACTIVATION.LABEL) {
                return (
                    <ModeSetting
                        onClose={this.props.onClose.bind(this)}
                        onSubmit={this.props.onModeSubmit.bind(this, this.props.nodeID)}
                        updateSettingParamsStructure={this.props.updateSettingParamsStructure}
                        settingParams={this.props.settingParams}
                    />
                )
            } else if (this.props.structure.nodes[index].label === CONST.BUTTON.SETTING.LABEL) {
                return (
                    <OverviewSetting
                        onClose={this.props.onClose.bind(this)}
                        onSubmit={this.props.onSettingSubmit.bind(this, this.props.nodeID)}
                        onSaveSetting={this.props.onSaveSetting.bind(this)}
                        onLoadSetting={this.props.onLoadSetting.bind(this)}
                        updateSettingParamsStructure={this.props.updateSettingParamsStructure}
                        settingParams={this.props.settingParams}
                    />
                )
            }
        }
    }

    render() {
        return (
            <div>
                <Modal
                    isOpen={this.props.isActive}
                    onRequestClose={this.props.onClose.bind(this)}
                    style={customStyles}
                    contentLabel="Example Modal"
                >
                    {this.getDetailSetting()}
                </Modal>
            </div>
        )
    }
}