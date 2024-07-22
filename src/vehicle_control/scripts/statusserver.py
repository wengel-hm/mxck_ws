#!/usr/bin/env python
import rospy
from vehicle_control.srv import ChangeStatus, ChangeStatusResponse

def handle_change_status(req):
    rospy.loginfo("Received request from: {}".format(req.sender))
    print("hat funktioniert")
    success = True # or False, based on your business logic
    return ChangeStatusResponse(success)

def change_status_server():
    rospy.init_node('vehicle_control')
    s = rospy.Service('change_status', ChangeStatus, handle_change_status)
    rospy.loginfo("Ready to change status.")
    rospy.spin()

if __name__ == "__main__":
    change_status_server()

