#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
import transformations
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
import rospkg
import sys
import os
import time
from math import sin, cos, atan2
from std_msgs.msg import Bool

# The following line of code dynamically modifies the Python search path for modules at runtime by appending a specific directory.
# It uses the `rospkg` module, which provides an interface to ROS package locations, to find the directory of the 'mpc' package.
# Specifically, it appends the 'include' directory within the 'mpc' package to the `sys.path` list. This is necessary because the
# 'include' directory is where some Python modules or packages are located, but it's not a standard location that Python automatically
# searches for modules. By appending this directory to `sys.path`, it allows Python scripts to import modules from this non-standard location.
# In this case, after modifying `sys.path`, the script imports the `SupportFilesCar` module from the 'include' directory of the 'mpc' package.
sys.path.append(os.path.join(rospkg.RosPack().get_path('stanley'), 'include'))
from cubic_spline_interpolator import generate_cubic_spline
#roslaunch stanley run_stanley.launch to launch the stanley controller

# Node that subscribes to /path topic
class Stanley(object):
    def __init__(self): 
        rospy.init_node('stanley')


        # Subscribe to the /path topic
        self.path_sub = rospy.Subscriber('/path', Path, self.callback)
        self.pit_ride_sub = rospy.Subscriber('/pit_ride', Bool, self.callbackpit)

        # Publish ackermann messages to VESC
        self.ackermann_pub = rospy.Publisher('/autonomous/ackermann_cmd', AckermannDriveStamped, queue_size=1) 

        # Define messages
        self.ackMsg = AckermannDriveStamped()
        

        
        self.X=0.  
        self.Y=0.    
        self.last_time = rospy.Time.now()
        self.ds = 0.05
        self.px = []
        self.py = []
        self.pyaw = []
        self.k = 0.25 # between 0.1 and 0.6; control_gain; Increase this value to increase the magnitude of the steering angle.
        self.k_soft = 3.8 # softening_gain reduction steering error; with speed scaled values between 1 and 5 
        self.k_yaw_rate = 0.03 # yaw_rate_gain increase yaw error around 0.1 is max
        self.k_damp_steer = 0.05#steering_damp_gain prevents big jumps in steeringangle change; smaller means more dampening
        self.max_steer = 0.42
        self.wheelbase = 0.36
        self.target_velocity = 1.00  #drive between 1 and 2 m/s 
        self.wheel_angle = 0.0
        self.vmax = 1.1
        self.vmin = 0.5
        self.k_damp_speed = 1.0 
        self.last_time = rospy.Time.now()
        self.r = rospy.Rate(30)
        self.correct = Bool()
        self.correct = True 
        self.monitor_path()
        self.pit_ride = False
        
    def get_trajectory(self, msg):
        x_coords, y_coords, z_rotations = [], [], []

        for pose_stamped in msg.poses: #for pathplanning
        #for pose_stamped in msg.poses[1:]:
            # Extract x, y coordinates
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            x_coords.append(x)
            y_coords.append(y)

            # Extract quaternion orientation
            quaternion = (
                pose_stamped.pose.orientation.x,
                pose_stamped.pose.orientation.y,
                pose_stamped.pose.orientation.z,
                pose_stamped.pose.orientation.w
            )

            # Convert quaternion to Euler angles
            euler = transformations.euler_from_quaternion(quaternion)

            # z-axis rotation in radians
            z_rotation = euler[2]
            z_rotations.append(z_rotation)
        
        return np.array(x_coords), np.array(y_coords), np.array(z_rotations)
    
    def callbackpit(self, msg):									
        self.pit_ride = msg.data 
        print("pitride", self.pit_ride)
    
    
    
    def normalise_angle(angle):
    
        #Normalize the given angle to the range [-pi, pi].
        alpha = atan2(sin(angle), cos(angle))
    
        return alpha
        
    def find_target_path_id(self, x, y, yaw):  

        # position of the front axle
        fx = 0 #x + self.wheelbase * cos(yaw)
        fy = 0 #y + self.wheelbase * sin(yaw)

        dx = fx - self.px    # Find the x-axis of the front axle relative to the path
        dy = fy - self.py    # Find the y-axis of the front axle relative to the path
        d = np.hypot(dx, dy) # Find the distance from the front axle to the path
        target_index = np.argmin(d) # Find the shortest distance in the array
        
        return target_index, dx[target_index], dy[target_index], d[target_index]

    def calculate_yaw_term(self, target_index, yaw):

        errorangle = self.pyaw[target_index] - yaw
        
        yaw_error = atan2(sin(errorangle), cos(errorangle))

        return yaw_error

    def calculate_crosstrack_term(self, target_velocity, yaw, dx, dy, absolute_error):

        front_axle_vector = np.array([sin(yaw), -cos(yaw)])
        nearest_path_vector = np.array([dx, dy])
        crosstrack_error = np.sign(nearest_path_vector@front_axle_vector) * absolute_error

        crosstrack_steering_error = atan2((self.k * crosstrack_error), (self.k_soft + target_velocity))

        return crosstrack_steering_error, crosstrack_error

    def calculate_yaw_rate_term(self, target_velocity, steering_angle):


        yaw_rate_error = self.k_yaw_rate*(-target_velocity*sin(steering_angle))/self.wheelbase
        print("yaw_rate_error", yaw_rate_error)
        return yaw_rate_error

    def calculate_steering_delay_term(self, computed_steering_angle, previous_steering_angle):

        steering_delay_error = self.k_damp_steer*(computed_steering_angle - previous_steering_angle)

        return steering_delay_error

    def stanley_control(self, x, y, yaw, target_velocity, steering_angle):

        target_index, dx, dy, absolute_error = self.find_target_path_id(x, y, yaw)
        yaw_error = self.calculate_yaw_term(target_index, yaw)
        crosstrack_steering_error, crosstrack_error = self.calculate_crosstrack_term(target_velocity, yaw, dx, dy, absolute_error)
        yaw_rate_damping = self.calculate_yaw_rate_term(target_velocity, steering_angle)
        
        desired_steering_angle = yaw_error + crosstrack_steering_error + yaw_rate_damping
        print("yaw_error", yaw_error)
        print("crosstrack_steering_error", crosstrack_steering_error)
        print("yaw_rate_damping", yaw_rate_damping)
        # Constrains steering angle to the vehicle limits
        desired_steering_angle += self.calculate_steering_delay_term(desired_steering_angle, steering_angle)
        limited_steering_angle = np.clip(desired_steering_angle, -self.max_steer, self.max_steer)

        return limited_steering_angle, target_index, crosstrack_error
        
    def get_speed(self, yaw_angles, v_max, v_min, target_velocity):
        """
        Calculate the mean desired vehicle speed based on the yaw angles of the trajectory.

        Parameters:
        yaw_angles (array-like): Array of yaw angles (in radians).
        v_max (float): Maximum speed when yaw angle is 0.
        v_min (float): Minimum speed when yaw angle is at its extreme values.

        Returns:
        float: The mean calculated speed.
        """
        yaw_angles = np.array(yaw_angles)
        num_angles = len(yaw_angles)
        # Normalize yaw angles to the range [-1, 1] based on ±1.2 radians
        normalized_yaw = yaw_angles / 1.5
        
        # Clip the normalized_yaw values to the range [-1, 1]
        normalized_yaw = np.clip(normalized_yaw, -1, 1)
        #print("yaw", normalized_yaw)
        # Calculate speeds based on cosine of normalized yaw angles
        speeds = v_min + (v_max - v_min) * (1 + np.cos(normalized_yaw * np.pi)) / 2
        print("speeds", speeds)
        # Calculate and return the mean speed
        weights = np.linspace(1, num_angles, num_angles)
    
        # Calculate the weighted mean speed
        mean_speed = np.average(speeds, weights=weights)
        print("mean speed", mean_speed)
       
        #speed = self.k_damp_speed * (mean_speed - target_velocity)
        #velocity = speed + target_velocity
        #print("actual speed", velocity)
        return mean_speed
        
    def driveback(self):    
        self.r = rospy.Rate(30)

        print("Fahre Rückwärts") 

        for _ in range(40):
            self.ackMsg.drive.steering_angle = self.wheel_angle * 0.8
            self.ackMsg.drive.speed = -0.8
            self.ackermann_pub.publish(self.ackMsg)
            self.r.sleep()

    def callback(self, msg):

        
        X_ref, Y_ref, psi_int = self.get_trajectory(msg)
        self.last_time = rospy.Time.now()
        self.px, self.py, self.pyaw, _ = generate_cubic_spline(X_ref, Y_ref, self.ds)
        self.pyaw = self.pyaw * -1
        self.py = self.py * -1
        print("py1", self.py[0])
        #self.target_velocity = self.get_speed(self.pyaw, self.vmax, self.vmin, self.target_velocity) #to change the speed according to the corner
        if self.pit_ride == True: 
            self.target_velocity = 0.5
        print("pyaw", self.pyaw)
        print("Targetvel", self.target_velocity)
        # Calculate the turning angle of the car based on the delta
        self.wheel_angle, self.target_id, self.crosstrack_error = self.stanley_control(0, 0, 0, self.target_velocity, self.wheel_angle)

        print("steeringAngle", self.wheel_angle)	
        speed = self.target_velocity 
        steering_angle = self.wheel_angle

        self.ackMsg.header.stamp = rospy.Time.now()
        self.ackMsg.drive.steering_angle = steering_angle
        self.ackMsg.drive.speed = speed

        self.ackermann_pub.publish(self.ackMsg)
        
    def monitor_path(self):
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            elapsed_time = current_time - self.last_time
            
            # Check if the time since last message is over 5 seconds
            if elapsed_time.to_sec() > 5:
                if self.correct == True:
                	self.driveback()
                
                

    def spin(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        node = Stanley()
        node.spin()
    except rospy.ROSInterruptException:
        pass
