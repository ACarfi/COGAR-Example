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
        cls.node_start_time = rospy.Time.now()
        # Allow time for the subscriber to establish connection

    def setUp(self):
        self.num_messages = 10
        self.log_queue = []

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
        return "I heard " + random_message

    def test_callback(self):
        pub = rospy.Publisher('chatter', String, queue_size=10)
        rospy.sleep(1)  # Allow time for the subscriber to connect

        messages = [self.generate_random_message() for _ in range(self.num_messages)]

        for msg in messages:
            pub.publish(msg)
            rospy.sleep(0.1) 

        self.assertGreater(len(self.log_queue), 0, "No log messages were received")
        rospy.loginfo(messages)
        rospy.loginfo(self.log_queue)
        for i in range(len(messages)):
            self.assertIn(messages[i], self.log_queue[i], f"Expected log message '{messages[i]}' not found")


if __name__ == '__main__':
    import rostest
    rostest.rosrun('example_pkg', 'test_listener_node', TestListenerNode)


''' from __future__ import absolute_import

import unittest
import rospy
import rostest
import sys
import os
import logging
import io
import re  # Import regular expression module
from std_msgs.msg import String

sys.path.append(os.path.join(os.path.dirname(__file__), '../scripts'))

from listener import listener, callback

class TestListenerNode(unittest.TestCase):
    def setUp(self):
        # Initialize ROS node for testing
        rospy.init_node('test_listener_node', anonymous=True)
        
    def test_callback(self):
        # Create a custom in-memory logger to capture rospy logs
        log_stream = io.StringIO()
        logging.basicConfig(stream=log_stream, level=logging.INFO)
        logger = logging.getLogger('rospy')

        # Call the callback directly with a test message
        msg = String()
        msg.data = "Hello, ROS!"
        callback(msg)
        
        # Sleep to ensure the callback has time to log the message
        rospy.sleep(1)

        # Extract just the log message content using regular expressions
        log_output = log_stream.getvalue()
        match = re.search(r"I heard (.*)", log_output)

        # Check if the expected log message is in the captured logs
        self.assertTrue(match and match.group(1) == "Hello, ROS!")

    def test_listener(self):
        # Set up a publisher to send messages to 'chatter'
        pub = rospy.Publisher('chatter', String, queue_size=10)
        
        # Allow time for the listener to subscribe
        rospy.sleep(1)
        
        # Create a custom in-memory logger to capture rospy logs
        log_stream = io.StringIO()
        logging.basicConfig(stream=log_stream, level=logging.INFO)
        logger = logging.getLogger('rospy')

        # Publish a test message
        pub.publish("Test message")
        
        # Sleep to allow time for the listener to process the message
        rospy.sleep(1)

        # Extract just the log message content using regular expressions
        log_output = log_stream.getvalue()
        match = re.search(r"I heard (.*)", log_output)

        print(match)
        # Check that the callback was triggered by inspecting the logs
        self.assertTrue(match and match.group(1) == "Test message")

if __name__ == '__main__':
    rostest.rosrun('example_pkg', 'test_listener_node', TestListenerNode)



class TestListener(unittest.TestCase):

    # reference: https://docs.python.org/3/library/unittest.html#unittest.TestCase.setUpClass
    @classmethod
    def setUpClass(cls):
        cls.io_listener = IoListener()
        cls.num_of_ports = 8
        cls.bits_per_port = 16
        cls.max_int = pow(2, cls.bits_per_port)
        cls.input_pubs = [rospy.Publisher(
            'io/input/port%d' % (i), UInt16, queue_size=10) for i in range(1, cls.num_of_ports + 1)]
        cls.output_pubs = [rospy.Publisher(
            'io/output/port%d' % (i), UInt16, queue_size=10) for i in range(1, cls.num_of_ports + 1)]
        rospy.sleep(1)  # wait of initialization

    # reference: https://docs.python.org/3/library/unittest.html#unittest.TestCase.setUp
    def setUp(self) -> None:
        # reset inputs and outputs before each test fixture
        self.io_listener.reset_inputs_outputs()

    def test_input_callback(self):
        inputs = [random.randint(0, self.max_int) for i in range(self.num_of_ports)]
        for i in range(self.num_of_ports):
            self.input_pubs[i].publish(UInt16(data=inputs[i]))
            rospy.sleep(0.2)  # give some time for callback function to execute
        self.assertEqual(self.io_listener.get_inputs(), inputs)

    def test_output_callback(self):
        outputs = [random.randint(0, self.max_int) for i in range(self.num_of_ports)]
        for i in range(self.num_of_ports):
            self.output_pubs[i].publish(UInt16(data=outputs[i]))
            rospy.sleep(0.2)  # give some time for callback function to execute
        self.assertEqual(self.io_listener.get_outputs(), outputs)


if __name__ == "__main__":
    import rostest
    # When publisher/subscriber involved, we have to init a ROS node
    rospy.init_node('test_listener')
    rostest.rosrun('example_pkg', 'test_listener', TestListener)'''