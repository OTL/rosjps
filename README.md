rosjps
=============

Bridge Between ROS(http://wiki.ros.org) and jps(https://github.com/OTL/jps)

how to use
------------
Below example converts one ROS topic to jps topic, and converts one jps topic to ROS.

- /cv_camera/image_raw (ROS) -> jps
- /cmd_vel (jps) -> ROS

```python
from rosjps import jps_bridge
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

rospy.init_node('jps_bridge')
ros_to_jps = jps_bridge.RosToJpsBridge('/cv_camera/image_raw', Image)
jps_to_ros = jps_bridge.JpsToRosBridge('/cmd_vel', Twist)
rospy.spin()
```
