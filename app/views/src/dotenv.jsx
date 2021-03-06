import {MQTT_HOST,MQTT_JS_PORT} from "react-native-dotenv";

const AUTOWARE_WEB_UI_HOST = location.hostname;
const AUTOWARE_WEB_UI_PORT = location.port;

const WEB_UI_URL = "http://"+AUTOWARE_WEB_UI_HOST+":"+AUTOWARE_WEB_UI_PORT;
const MQTT_HOST_NAME = MQTT_HOST;
const MQTT_PORT = MQTT_JS_PORT;

export {WEB_UI_URL, MQTT_HOST_NAME,MQTT_PORT};

