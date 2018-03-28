import React from 'react';
import Modal from 'react-modal';
import SettingDetails from './setting_details'


const customStyles = {
    content: {
        top: '10%',
        left: '30%',
        right: 'auto',
        bottom: 'auto',
        height: "80%",
        width: "40%",
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

    render() {
        return (
            <div>
                <Modal
                    isOpen={this.props.isActive}
                    onRequestClose={this.props.onClose}
                    style={customStyles}
                    contentLabel="Example Modal"
                >
                    <SettingDetails
                        onClose={this.props.onClose}
                        onSubmit={this.props.onSubmit}
                        updateSettingParamsStructure={this.props.updateSettingParamsStructure}
                        settingParams={this.props.settingParams}
                    />
                </Modal>
            </div>
        )
    }
}