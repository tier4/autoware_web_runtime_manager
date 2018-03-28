import React from 'react';

export default class SettingDetails extends React.Component {


    constructor(props) {
        super(props);
        this.state = {setting_number: 0};
    }

    set_tf(e) {
        var settingParams = this.props.settingParams;
        settingParams.setup.tf[e.target.name] = parseFloat(e.target.value);

        this.setState({setting_number: settingParams.setup.tf[e.target.name]});
        this.props.updateSettingParamsStructure(settingParams)
    }

    set_tf_file(e) {
        var settingParams = this.props.settingParams;
        var files = e.target.files;
        var reader = new FileReader();

        reader.onload = function (theFile) {
            var text = theFile.target.result;
            var json = JSON.parse(text);
            settingParams.setup.tf = json.tf;
        };
        reader.readAsText(files[0], "utf-8");
        this.props.updateSettingParamsStructure(settingParams)
    }

    set_vehicle_model(e){


    }


    render() {
        return (
            <div>
                <h2 ref={subtitle => this.subtitle = subtitle}>Set up</h2>
                <h3>TF</h3>
                <form id="settings">
                    <label htmlFor="set_tf_x">x</label>
                    <input type="number" ref="set_tf_x" name="tf_x" value={this.props.settingParams.setup.tf.tf_x}
                           step="0.1" min="-10000" max="10000" onChange={this.set_tf.bind(this)}/><br/>
                    <label htmlFor="set_tf_y">y</label>
                    <input type="number" ref="set_tf_y" name="tf_y" value={this.props.settingParams.setup.tf.tf_y}
                           step="0.1" min="-10000" max="10000" onChange={this.set_tf.bind(this)}/><br/>
                    <label htmlFor="set_tf_z">z</label>
                    <input type="number" ref="set_tf_z" name="tf_z" value={this.props.settingParams.setup.tf.tf_z}
                           step="0.1" min="-1000" max="1000" onChange={this.set_tf.bind(this)}/><br/>
                    <label htmlFor="set_tf_yaw">yaw</label>
                    <input type="number" ref="set_tf_yaw" name="tf_yaw" value={this.props.settingParams.setup.tf.tf_yaw}
                           step="0.1" min="-50" max="50" onChange={this.set_tf.bind(this)}/><br/>
                    <label htmlFor="set_tf_pitch">pitch</label>
                    <input type="number" ref="set_tf_pitch" name="tf_pitch" value={this.props.settingParams.setup.tf.tf_pitch}
                           step="0.1" min="-50" max="50" onChange={this.set_tf.bind(this)}/><br/>
                    <label htmlFor="set_tf_roll">roll</label>
                    <input type="number" ref="set_tf_roll" name="tf_roll" value={this.props.settingParams.setup.tf.tf_roll}
                           step="0.1" min="-50" max="50" onChange={this.set_tf.bind(this)}/><br/>
                    <label htmlFor="set_tf_file">set parameter by file</label>
                    <input type="file" id="set_tf_file" onChange={this.set_tf_file.bind(this)}/><br/>
                </form>

                <h3>Vehicle Model</h3>
                <form id="set_vehicle_model">
                    <label htmlFor="set_vehicle_model">Load vehicle model</label>
                    <input type="file" id="set_vehicle_model" onChange={this.set_vehicle_model.bind(this)}/><br/>
                </form>

                <h2 ref={subtitle => this.subtitle = subtitle}>Set up</h2>
                <h3>TF</h3>
                <form id="settings">
                    <meter id="mtr1" value="0" min="0" max="22" low="10" high="17" optimum="22"/>
                    <p>{this.state.setting_number}</p>
                    <br/>
                    <label htmlFor="set_tf_x">x</label>
                    <input type="number" ref="set_tf_x" name="set_tf_x"
                           step="0.1" min="-10000" max="10000" placeholder="0" onChange={this.set_tf.bind(this)}/><br/>
                    <label htmlFor="set_tf_y">y</label>
                    <input type="number" ref="set_tf_y" name="set_tf_y"
                           step="0.1" min="-10000" max="10000" placeholder="0" onChange={this.set_tf.bind(this)}/><br/>
                    <label htmlFor="set_tf_z">z</label>
                    <input type="number" ref="set_tf_z" name="set_tf_z"
                           step="0.1" min="-1000" max="1000" placeholder="0" onChange={this.set_tf.bind(this)}/><br/>
                    <label htmlFor="set_tf_yaw">yaw</label>
                    <input type="number" ref="set_tf_yaw" name="set_tf_yaw"
                           step="0.1" min="-50" max="50" placeholder="0" onChange={this.set_tf.bind(this)}/><br/>
                    <label htmlFor="set_tf_pitch">pitch</label>
                    <input type="number" ref="set_tf_pitch" name="set_tf_pitch"
                           step="0.1" min="-50" max="50" placeholder="0" onChange={this.set_tf.bind(this)}/><br/>
                    <label htmlFor="set_tf_roll">roll</label>
                    <input type="number" ref="set_tf_roll" name="set_tf_roll"
                           step="0.1" min="-50" max="50" placeholder="0" onChange={this.set_tf.bind(this)}/><br/>
                    <label htmlFor="set_tf_file">set parameter by file</label>
                    <input type="file" id="set_tf_file" onChange={this.set_tf_file.bind(this)}/><br/>
                </form>


                <br/><br/>
                <button onClick={this.props.onSubmit}>submit</button>
                <button onClick={this.props.onClose}>close</button>
            </div>
        )
    }
}