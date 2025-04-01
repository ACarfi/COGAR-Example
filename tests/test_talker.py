#!/usr/bin/env python

import unittest
import rospy
from std_msgs.msg import String
import time

class TestTalkerNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_talker_node', anonymous=True)

    def setUp(self):
        self.received_messages = []
        rospy.Subscriber('chatter', String, self.callback)
    
    def callback(self, msg):
        self.received_messages.append(msg)

    def test_publishing_frequency(self):
        start_time = time.time()

         # Wait for a few messages to be received
        timeout = 5  # Time in seconds
        rate = 10  # Expected frequency in Hz

        while len(self.received_messages) < timeout * rate:
            rospy.sleep(0.1)  # Wait for a bit to collect messages

        end_time = time.time()
        elapsed_time = end_time - start_time
        expected_message_count = elapsed_time * rate

        self.assertGreater(len(self.received_messages), expected_message_count * 0.9, "The publishing frequency is below 10 Hz")
        self.assertLess(len(self.received_messages), expected_message_count * 1.1, "The publishing frequency is above 10 Hz")

        rospy.loginfo(f"Elapsed Time: {elapsed_time} seconds")
        rospy.loginfo(f"Received Messages: {len(self.received_messages)}")

if __name__ == '__main__':
    import rostest
    rostest.rosrun('example_pkg', 'test_talker_node', TestTalkerNode)