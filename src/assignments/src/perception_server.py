#!/usr/bin/env python3

import rospy
import random
from assignments.srv import Perception, PerceptionResponse

def handle_perception_request(req):
    rospy.loginfo(f"Perception request received for objects: {req.objects}")

    # Simulate 90% success rate
    success = random.random() < 0.9

    if success:
        rospy.loginfo("Objects found!")
    else:
        rospy.logwarn("Some objects not found!")

    return PerceptionResponse(found=success)

def perception_server():
    rospy.init_node('perception_server')
    service = rospy.Service('/perception', Perception, handle_perception_request)
    rospy.loginfo("Perception server ready at /perception")
    rospy.spin()

if __name__ == '__main__':
    try:
        perception_server()
    except rospy.ROSInterruptException:
        pass
