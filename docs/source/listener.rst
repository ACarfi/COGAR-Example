
#### Example content for `listener.rst`:

```rst
Listener Node
==============

This page describes the listener node.

The listener node subscribes to a topic and prints the message:

```python
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("I heard %s", data.data)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('chatter', String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
