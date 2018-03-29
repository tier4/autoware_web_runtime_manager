mqtt:
  client:
    protocol: 4      # MQTTv311
  connection:
    host: localhost
    port: 1883
    keepalive: 60
  private_path: device/001
serializer: json:dumps
deserializer: json:loads
wrm: ${HOME}/autoware_web_runtime_manager/autoware_app/mqtt_setting/config.json
bridge:
  # ndt_pose
  - factory: mqtt_bridge.bridge:RosToMqttBridge
    msg_type: geometry_msgs.msg:PoseStamped
    topic_from: /ndt_pose
    topic_to: /test.test/rostopic.ndt_pose.ndt_pose/AtoU
  # ndt_pose_for_map
  - factory: mqtt_bridge.bridge:RosToMqttBridge
    msg_type: geometry_msgs.msg:PoseStamped
    topic_from: /ndt_pose
    topic_to: /test.test/rostopic.map_pose.map_pose/AtoU
  # current_latlon
  - factory: mqtt_bridge.bridge:RosToMqttBridge
    msg_type: sensor_msgs.msg:NavSatFix
    topic_from: /current_latlon
    topic_to: /test.test/rostopic.current_latlon.current_latlon/AtoU
  # points_raw  
  - factory: mqtt_bridge.bridge:RosToMqttBridge
    msg_type: sensor_msgs.msg:PointCloud2
    topic_from: /downsampled_points_raw
    topic_to: /test.test/rostopic.points_raw.points_raw/AtoU
  # tf  
  - factory: mqtt_bridge.bridge:RosToMqttBridge
    msg_type: tf2_msgs.msg:TFMessage
    topic_from: /tf
    topic_to: /test.test/rostopic.tf.tf/AtoU
  # vector_map  
  - factory: mqtt_bridge.bridge:RosToMqttBridge
    msg_type: visualization_msgs.msg:MarkerArray
    topic_from: /vector_map
    topic_to: /test.test/rostopic.vector_map.vector_map/AtoU
  # lane_waypoints_array
  - factory: mqtt_bridge.bridge:RosToMqttBridge
    msg_type: autoware_msgs.msg:LaneArray
    topic_from: /lane_waypoints_array
    topic_to: /test.test/rostopic.lane_waypoints_array.lane_waypoints_array/AtoU
  # downsampled_next_target_mark
  - factory: mqtt_bridge.bridge:RosToMqttBridge
    msg_type: visualization_msgs.msg:Marker
    topic_from: /downsampled_next_target_mark
    topic_to: /test.test/rostopic.downsampled_next_target_mark.downsampled_next_target_mark/AtoU
  # downsampled_trajectory_circle_mark	    
  - factory: mqtt_bridge.bridge:RosToMqttBridge
    msg_type: visualization_msgs.msg:Marker
    topic_from: /downsampled_trajectory_circle_mark
    topic_to: /test.test/rostopic.downsampled_trajectory_circle_mark.downsampled_trajectory_circle_mark/AtoU
  # clock
  - factory: mqtt_bridge.bridge:RosToMqttBridge
    msg_type: rosgraph_msgs.msg:Clock
    topic_from: /Clock
    topic_to: /test.test/rostopic.clock.clock/AtoU
  # initial_pose
  - factory: mqtt_bridge.bridge:MqttToRosBridge
    msg_type: geometry_msgs.msg:PoseWithCovarianceStamped
    topic_from: /test.test/rostopic.initialpose.initialpose/UtoA 
    topic_to: /initialpose

