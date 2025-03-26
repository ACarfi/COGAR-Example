#!/usr/bin/env python

import unittest
import rospy
import random
import string
from std_msgs.msg import String
from rosgraph_msgs.msg import Log

class TestListenerNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.TARGET_NODE = "listener.py"  # Adjust as needed
        rospy.init_node('test_listener_node', anonymous=True)
        # Allow time for the subscriber to establish connection

    def setUp(self):
        self.num_messages = 10
        self.log_queue = []
        self.node_start_time = rospy.Time.now()

        rospy.Subscriber('/rosout', Log, self.log_callback)
        rospy.sleep(1)

    def log_callback(self, msg):    
        if msg.header.stamp < self.node_start_time:
            return  # Skip the message if it's older than the node's start time

        if msg.file == self.TARGET_NODE:
            self.log_queue.append(msg.msg)
            self.msg = msg.msg
            self.log_received = True  # Mark that a log was received
    
    #Generate a random string of fixed length.
    def generate_random_message(self, length=10):
        random_message = ''.join(random.choices(string.ascii_letters + string.digits, k=length))
        return random_message

    def test_callback(self):
        pub = rospy.Publisher('chatter', String, queue_size=10)
        rospy.sleep(1)  # Allow time for the subscriber to connect

        messages = [self.generate_random_message() for _ in range(self.num_messages)]
        out_messages = ["I heard " + messages[i] for i in range(self.num_messages)]

        for msg in messages:
            pub.publish(msg)
            rospy.sleep(0.1) 

        self.assertGreater(len(self.log_queue), 0, "No log messages were received")
        for i in range(len(out_messages)):
            self.assertIn(out_messages[i], self.log_queue[i], f"Expected log message '{messages[i]}' not found")


    def test_edge_length(self):
        pub = rospy.Publisher('chatter', String, queue_size=10)
        rospy.sleep(1)  # Allow time for the subscriber to connect

        message_sizes = list(range(100, 10001, 100))
        messages = [self.generate_random_message(message_sizes[i]) for i in range(len(message_sizes))]
        out_messages = ["I heard " + messages[i] for i in range(len(message_sizes))]

        for msg in messages:
            pub.publish(msg)
            rospy.sleep(0.1)  # Give time for the callback to process the message

            
        # Assert that the log contains the published message
        self.assertGreater(len(self.log_queue), 0, "No log messages were received")
        # Check that the message is in the log
        for i in range(len(messages)):
            self.assertIn(out_messages[i], self.log_queue[i], f"Expected log message of size {message_sizes[i]} not found in the log")

    def test_frequency(self):
        pub = rospy.Publisher('chatter', String, queue_size=10)
        rospy.sleep(1)  # Allow time for the subscriber to connect

        messages = [self.generate_random_message() for _ in range(self.num_messages)]
        out_messages = ["I heard " + messages[i] for i in range(self.num_messages)]

        for i, msg in enumerate(messages):
            pub.publish(msg)
            
            # Decrease the sleep time (e.g., starting at 0.1 and getting smaller)
            sleep_time = max(0.01, 0.1 / (i + 1))  # Ensure sleep time doesn't go below 0.01 seconds
            rospy.sleep(sleep_time)

        self.assertGreater(len(self.log_queue), 0, "No log messages were received")
        for i in range(len(out_messages)):
            self.assertIn(out_messages[i], self.log_queue[i], f"Expected log message '{messages[i]}' not found")

if __name__ == '__main__':
    import rostest
    rostest.rosrun('example_pkg', 'test_listener_node', TestListenerNode)