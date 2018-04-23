import React from 'react';

export default class ModeSetting extends React.Component {

    constructor(props) {
        super(props);
    }

    setActivationMode(e){
        let settingParams = this.props.settingParams;
        console.log(settingParams);
        settingParams.mode.mode = e.target.value;
        this.props.updateSettingParamsStructure(settingParams)
    }

    render() {
        return (
            <div>
                <h2>Mode Set:</h2>
                <input type="radio" className="modeRadio" name="mode" value="rosbagMode"
                       onChange={this.setActivationMode.bind(this)}/>
                <label htmlFor="mode" className="modeRadioChar">&nbsp;Rosbag Mode</label>
                <br/>
                <input type="radio" className="modeRadio" name="mode" value="simulatorMode"
                       onChange={this.setActivationMode.bind(this)}/>
                <label htmlFor="mode" className="modeRadioChar">&nbsp;Simulator Mode</label>
                <br/>
                <input type="radio" className="modeRadio" name="mode" value="driveMode"
                       onChange={this.setActivationMode.bind(this)}/>
                <label htmlFor="mode" className="modeRadioChar">&nbsp;Drive Mode</label>
                <br/>
                <br/>
                <button onClick={this.props.onSubmit}>submit</button>
                <button onClick={this.props.onClose}>close</button>
            </div>
        )
    }
}