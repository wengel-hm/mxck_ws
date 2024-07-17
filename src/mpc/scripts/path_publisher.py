#!/usr/bin/env python

from scipy.integrate import cumtrapz, quad
import numpy as np
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import transformations as tf  #import tf

# Corrected quaternion calculation from heading
def heading_to_quaternion(heading_angle):
    # ROS uses quaternions to represent orientation. 
    # The heading_angle is considered as a yaw (rotation around z-axis).
    # tf.transformations.quaternion_from_euler expects roll, pitch, and yaw as inputs.
    quaternion = tf.transformations.quaternion_from_euler(0, 0, heading_angle)
    return quaternion

# Function to convert heading angle (in radians) to quaternion
def heading_to_quaternion(heading_angle):
    quaternion = tf.transformations.quaternion_from_euler(0, 0, heading_angle)
    return quaternion

# Function to compute the differential arc length of the sinus curve
def diff_arc_length(x, peak_height, x_period_length):
    # dy/dx of the sinus curve
    dy_dx = (2 * np.pi / x_period_length) * peak_height * np.cos((2 * np.pi / x_period_length) * x)
    # Differential arc length formula: sqrt(1 + (dy/dx)^2)
    return np.sqrt(1 + dy_dx**2)

def main():
    waypoints = 100
    x_period_length = 5  # in meters, for example
    peak_height = 1.5  # in meters, for example
    speed = 1  # Speed in m/s
    H = 8+1 # horizon

    # Initialize ROS node
    rospy.init_node('path_publisher', anonymous=True)
    
    # Publisher for the Path message on a given topic
    path_pub = rospy.Publisher('/path', Path, queue_size=10)
    

    # Initialize the Path message
    path_msg = Path()
    path_msg.header.frame_id = "front_axle"
    path_msg.header.stamp = rospy.Time.now()

    # Compute the total length of the sinus curve
    total_length, _ = quad(diff_arc_length, 0, x_period_length, args=(peak_height, x_period_length))

    # Desired distance between each point along the curve
    distance_between_points = total_length / (waypoints - 1)

    print("Distance: %.2fm" %distance_between_points)

    # Generate a high-resolution x values
    x_high_res = np.linspace(0, x_period_length, 10000)

    # Cumulative arc length along the curve
    cumulative_arc_length = cumtrapz(diff_arc_length(x_high_res, peak_height, x_period_length), x_high_res, initial=0)

    # Interpolate to find the x positions
    x_positions = np.interp(np.arange(waypoints) * distance_between_points, cumulative_arc_length, x_high_res)

    # Compute the corresponding y positions for these x positions
    y_positions = np.sin((2 * np.pi / x_period_length) * x_positions) * peak_height

    # Calculating dy/dx for the x positions
    dy_dx_positions = (2 * np.pi / x_period_length) * peak_height * np.cos((2 * np.pi / x_period_length) * x_positions)

    # Calculating the heading direction in radians
    heading_directions = np.arctan2(dy_dx_positions, np.ones_like(dy_dx_positions))

    trajectory_points = np.vstack((x_positions, y_positions, heading_directions)).T  

    # Rate at which the path is published
    duration = total_length / speed
    hz = int(1 / (duration / waypoints))
    rate = rospy.Rate(hz)  # Hz
    rospy.loginfo("length: %.1f m, speed: %.1f m/s, rate: %d hz" %(total_length, speed, hz))


    while not rospy.is_shutdown():
        
        # Fill the Path message
        for k in range(len(trajectory_points) - H+1):
            trajectory = trajectory_points[k : k+H]

            path_msg.poses = []
            x0, y0, theta0 = trajectory[0]

            rotation_matrix = np.array([
                [np.cos(theta0), np.sin(theta0)],
                [-np.sin(theta0), np.cos(theta0)]
            ])

            for i, point in enumerate(trajectory):


                x, y, heading = point
                x_rel = x - x0
                y_rel = y - y0

                # Apply the rotation matrix to the translated coordinates
                rotated_point = rotation_matrix.dot(np.array([x_rel, y_rel]))
                x_rot, y_rot = rotated_point

                # Adjust the heading relative to the first point's heading
                heading_rel = heading - theta0

                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = "front_axle"
                pose_stamped.header.stamp = rospy.Time.now() + rospy.Duration(i * distance_between_points / speed)
                
                pose_stamped.pose.position.x = x_rot # alternativ: x
                pose_stamped.pose.position.y = y_rot # alternativ: y
                pose_stamped.pose.position.z = 0
            
                # ROS Convention: In ROS, the positive yaw (heading) direction follows
                # the right-hand rule around the z-axis, which is also counter-clockwise 
                # when looking down from above.
                quaternion = heading_to_quaternion(heading_rel) # alternativ: heading
                pose_stamped.pose.orientation.x = quaternion[0]
                pose_stamped.pose.orientation.y = quaternion[1]
                pose_stamped.pose.orientation.z = quaternion[2]
                pose_stamped.pose.orientation.w = quaternion[3]
                
                path_msg.poses.append(pose_stamped)

            # Publish the Path message
            path_pub.publish(path_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
