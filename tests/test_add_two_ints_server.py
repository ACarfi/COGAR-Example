#!/usr/bin/env python
import rospy
import unittest
import random
from example_pkg.srv import AddTwoInts


class TestAddTwoIntsServer(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        max_int = pow(2, 64)/2 # We are using a 64 bit signed integer
        # Since the output of the sum it is still 64 bit signed integer,
        # each addend can be maximum half of the int range to avoid overflow
        cls.max_random_int = max_int / 2 
        rospy.wait_for_service('add_two_ints') # wait the service to be ready

    def test_handle_add_two_ints(self, num=10): # the default number of numbers used is 10
        A = [random.randint(-self.max_random_int, self.max_random_int) for i in range(num)] # list of first addend
        B = [random.randint(-self.max_random_int, self.max_random_int) for i in range(num)] # list of second addend
        expected_list = [A[i] + B[i] for i in range(num)] # list of results.
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts) # call service
        for i in range(len(A)):
            resp = add_two_ints(A[i], B[i])
            self.assertEqual(resp.sum, expected_list[i])


if __name__ == "__main__":
    import rostest
    rostest.rosrun('example_pkg', 'test_add_two_ints_server',
                   TestAddTwoIntsServer)