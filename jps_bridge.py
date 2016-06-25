import json

import jps
import rosjson
import rospy


class RosToJpsBridge:
    def __init__(self, topic_name, msg_type):
        self._ros_subscriber = rospy.Subscriber(topic_name, msg_type, self._to_jps)
        self._jps_publisher = jps.Publisher(topic_name)

    def _to_jps(self, msg):
        json_msg = rosjson.ros_message_to_json(msg)
        self._jps_publisher.publish(json_msg)


class JpsToRosBridge:
    def __init__(self, topic_name, msg_type):
        self._jps_subscriber = jps.Subscriber(topic_name, self._to_ros)
        self._ros_publisher = rospy.Publisher(topic_name, msg_type, queue_size=1)
        self._msg_type = msg_type
        self._jps_subscriber.spin(use_thread=True)

    def _to_ros(self, json_msg):
        msg_dict = json.loads(json_msg)
        ros_msg = self._msg_type()
        # ex) cmd_vel (geometry_msgs/Twist)
        # {linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}
        def json_data_to_ros_msg(json_obj, ros_obj):
            if isinstance(json_obj, (dict)):
                for key, val in json_obj.iteritems():
                    setattr(ros_obj, key, json_data_to_ros_msg(val, getattr(ros_obj, key)))
                return ros_obj
            # list is not supported yet
            if isinstance(json_obj, (list)):
                return [json_data_to_ros_msg(x) for x in json_obj]
            else:
                return json_obj
        ros_msg = json_data_to_ros_msg(msg_dict, ros_msg)
        self._ros_publisher.publish(ros_msg)


def main():
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Image

    rospy.init_node('jps_bridge')
    ros_to_jps = RosToJpsBridge('cv_camera/image_raw', Image)
    jps_to_ros = JpsToRosBridge('/cmd_vel2', Twist)
    rospy.spin()

if __name__ == '__main__':
    main()
