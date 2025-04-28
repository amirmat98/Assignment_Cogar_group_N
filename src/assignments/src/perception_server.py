#!/usr/bin/env python3

"""
.. module:: perception_server
   :platform: Unix
   :synopsis: Simulates an object perception service for detecting objects.

.. moduleauthor:: AmirMahdi Matin

This module implements a ROS service node that simulates object detection:
- Listens for perception requests on the `/perception` service.
- Simulates a 90% chance of successfully detecting requested objects.
- Returns a boolean response indicating success or failure.
"""

import rospy
import random
from assignments.srv import Perception, PerceptionResponse

def handle_perception_request(req):
    """
    Callback function that processes incoming perception service requests.

    Simulates the perception of requested objects with a 90% success probability.

    :param req: The perception request containing a list of objects to detect.
    :type req: assignments.srv.PerceptionRequest
    :return: A response indicating whether the objects were successfully detected.
    :rtype: assignments.srv.PerceptionResponse
    """
    rospy.loginfo(f"Perception request received for objects: {req.objects}")

    # Simulate 90% success rate
    success = random.random() < 0.9

    if success:
        rospy.loginfo("Objects found!")
    else:
        rospy.logwarn("Some objects not found!")

    return PerceptionResponse(found=success)

def perception_server():
    """
    Initializes the perception server node and starts the /perception service.

    Waits for incoming perception requests and handles them using `handle_perception_request`.
    """
    rospy.init_node('perception_server')
    service = rospy.Service('/perception', Perception, handle_perception_request)
    rospy.loginfo("Perception server ready at /perception")
    rospy.spin()

if __name__ == '__main__':
    try:
        perception_server()
    except rospy.ROSInterruptException:
        pass
