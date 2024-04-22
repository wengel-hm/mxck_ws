#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDriveStamped

class PIDcontrol():

    def __init__(self):

        # read parameters
        self.read_parameters()

        # subscribe offset
        self.offset_sub = rospy.Subscriber('/lane_center_offset', Float32, self.callback) # 20hz

        # publish ackermann
        self.ackermann_pub = rospy.Publisher('/autonomous/ackermann_cmd', AckermannDriveStamped, queue_size=1) 

        # define messages
        self.ackMsg = AckermannDriveStamped()
        
        # max steering angle [deg]
        self.t_previous = rospy.Time.now()
        self.e_previous = 0
        self.P, self.I, self.D = 0, 0, 0
        
        self.speed_mps, self.max_angle = 0, 0

        # Setup timer to update parameters every 2 seconds
        rospy.Timer(rospy.Duration(2), lambda e: self.read_parameters())
    
    def validate_parameters(self):
        pass

    def read_parameters(self):
        try:
            # Re-read parameters to see if they have been changed
            self.kp = rospy.get_param('pid/kp')
            self.ki = rospy.get_param('pid/ki')
            self.kd = rospy.get_param('pid/kd')
            self.max_angle = rospy.get_param('pid/max_angle')
            self.speed = rospy.get_param('pid/speed')

        except rospy.ROSException as e:
            rospy.logwarn(f"Failed to retrieve parameter: {e}")
        
        self.validate_parameters()
    
    def callback(self, data):
        error = data.data
        
        t = rospy.Time.now()
        dt = (t - self.t_previous).to_sec()
        de = self.e_previous - error 
        self.P = error
        self.I = self.I + error * dt
        self.D = de / dt
        steer_rad = self.kp * self.P + self.ki * self.I + self.kd * self.D
        
        # clip output to +/- max_angle
        steer_rad = max(-self.max_angle, min(steer_rad, self.max_angle))
        
        self.t_previous = t
        self.e_previous = error

        self.ackMsg.header.stamp = rospy.Time.now()
        self.ackMsg.drive.steering_angle = steer_rad
        self.ackMsg.drive.speed = self.speed_mps

        try:
            self.ackermann_pub.publish(self.ackMsg)
        except rospy.ROSException as e:
            rospy.logerr(f"Failed to publish message: {str(e)}")

if __name__ == "__main__":
    rospy.init_node("pid", anonymous = False)
    
    pid = PIDcontrol()

    rospy.spin()
