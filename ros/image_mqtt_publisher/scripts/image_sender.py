#!/usr/bin/env python
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import paho.mqtt.client as mqtt

#from __future__ import print_function

class MqttClient:

    __host = "10.254.0.31"
    __port = 1883
    __userid = "test"
    __carid = "test"
    __topic_type = "image"
    __topic_label = "ImageRaw"
    __toAutoware = "UtoA"
    __fromAutoware = "AtoU"
    __packet_size = 3000
    
    def __init__(self):
        self.client = mqtt.Client(protocol=mqtt.MQTTv311)
        self.client.on_connect = self.__on_connect
        self.client.on_disconnect = self.__on_disconnect
        self.client.connect(self.__host, self.__port)
        
        header = "/" + self.__userid + "." + self.__carid
        body = "/" + self.__topic_type + "." + self.__topic_label + "." + self.__topic_label
        direction = "/" + self.__fromAutoware
        self.topic = header + body + direction
        print(self.topic)

    def onPublish(self,message):
        #byteArray = bytes(message)
        self.client.publish(self.topic,message,0)

    def disConnect(self):
        self.client.disconnect()

    def __on_connect(self,client, userdata, flags, respons_code):
        print('status {0}'.format(respons_code))
        print("mqtt connect.")

    def __on_disconnect(self,client,userdata,rc):
        print("DisConnected result code "+str(rc))
        #self.client.loop_stop()


class ImageMqttPublisher:

    def __init__(self,client):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image_raw",Image,self.callback)
        self.mqtt_client = client

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60 :
            cv2.circle(cv_image, (50,50), 10, 255)

        cv_image = cv2.imread("./test.jpg",0)
        
        encode_param=[int(cv2.IMWRITE_JPEG_QUALITY),20]
        img=cv2.imencode('.jpg',cv_image,encode_param)[1].tostring()
        self.mqtt_client.onPublish(img)



def main():
    rospy.init_node('WRM_mqtt_publisher', anonymous=True)
    print("WRM_mqtt_publisher run.")
    mqtt_client = MqttClient()
    ip = ImageMqttPublisher(mqtt_client)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
