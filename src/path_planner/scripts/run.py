#!/usr/bin/env python
import rospy
import sys
import os

base_dir = os.getcwd()
pyc_dir = '/noetic_ws/src/path_planner/scripts/__pycache__'

sys.path.append(pyc_dir)


from lane_path_planner import LanePathPlanner

if __name__ == '__main__':
    try:
        planner = LanePathPlanner(debug = False)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
        
