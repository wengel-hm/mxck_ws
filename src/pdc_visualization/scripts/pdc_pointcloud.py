#!/usr/bin/env python
import rospy
import math
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Int16MultiArray, Header
import sensor_msgs.point_cloud2 as pc2
from tf2_geometry_msgs import PointStamped
import tf2_geometry_msgs
import numpy as np

class UltrasonicSensorProcessor:
    def __init__(self):

        rospy.init_node('ultrasonic_pointcloud')

        self.max_distance = 1.5 # meters
        self.min_distance = 0.02 # meters
        self.fov = 60 # degrees field of view
        self.num_points = 20 # number of points per wave
        self.num_waves = 20 # number of waves per sensor
        self.base = "base_link" # all points get transformed to this frame

        self.intervals = np.linspace(self.min_distance, self.max_distance, self.num_waves)
        self.angle_start = np.deg2rad(-self.fov / 2)  # Start angle in radians
        self.angle_end = np.deg2rad(self.fov / 2)    # End angle in radians
        self.thetas = np.linspace(self.angle_start, self.angle_end, self.num_points) # Calculate all theta angles for the points on the arc

        # Mapping from array indices to corresponding tf frames
        # Links sensor data indices from '/uss_values' to their tf frames for localization.
        self.index2frame = {
            0: "USS_SRB",
            1: "USS_SRF",
            2: "USS_FR",
            3: "USS_FC",
            4: "USS_FL",
            5: "USS_SLF",
            6: "USS_SLB",
            7: "USS_BL",
            8: "USS_BC",
            9: "USS_BR"
        }

        self.frames = list(self.index2frame.values())

        # Single Publisher for combined PointCloud2 message
        self.publisher = rospy.Publisher("/scan", PointCloud2, queue_size=10)
               
        # TF2 Listener setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Fetch and store all transformations
        self.transformations = {}
        for frame in self.frames:
            try:
                # Wait for the transform to become available and store it
                transformation = self.tf_buffer.lookup_transform(self.base, frame, rospy.Time(0), rospy.Duration(10.0))
                self.transformations[frame] = transformation
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logerr('Error fetching transform from %s to %s: %s', frame, self.base, e)

        # Subscriber for ultrasonic distances
        self.subscriber = rospy.Subscriber('/uss_values', Int16MultiArray, self.callback)

    def callback(self, data):

        all_points = []
        distances = data.data

        for i, distance in enumerate(distances):

            if distance <= 0: continue

            distance_meters = float(distance) / 100  # Convert cm to meter
            points = self.generate_points_at_waves(distance_meters)  # Generate 100 points
            transformed_points = self.transform_points(points, self.index2frame[i], self.base)
            all_points.extend(transformed_points)

        pointcloud_msg = self.points_to_pointcloud_message(all_points, self.base)
        self.publisher.publish(pointcloud_msg)
     

    def generate_points_at_waves(self, measurement):
        
        distances = self.intervals[self.intervals < measurement]

        if len(distances) > 0:
            distances[-1] = measurement
        else:
            distances = [measurement]


        # Initialize an empty list to collect all points
        all_points = []
        for d in distances:
            # Calculate x and y coordinates
            x = d * np.cos(self.thetas)
            y = d * np.sin(self.thetas)
            z = np.full(self.num_points, d)  # Set all z values to distance

            # Combine x, y, z for the current distance and append to all_points
            points = np.column_stack((x, y, z))
            all_points.append(points)

        # Combine all points from all distances into one array
        all_points = np.vstack(all_points)
        return all_points
    

    def transform_points(self, points, from_frame, to_frame):
        transformed_points = []
        for (x, y, z) in points:
            point_stamped = PointStamped()
            point_stamped.header.frame_id = from_frame
            point_stamped.header.stamp = rospy.Time.now()
            point_stamped.point.x = x
            point_stamped.point.y = y
            point_stamped.point.z = z
            
            transformation = self.transformations.get(from_frame)
            if transformation is None:
                rospy.logerr("No transformation available for frame %s", from_frame)
                continue  # Skip this point or handle it according to your application's needs

            transformed_point = tf2_geometry_msgs.do_transform_point(point_stamped, transformation)
            transformed_points.append((transformed_point.point.x, transformed_point.point.y, transformed_point.point.z))

        return transformed_points


    def points_to_pointcloud_message(self, points, frame_id):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id
        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        
        return pc2.create_cloud(header, fields, points)

if __name__ == '__main__':
    processor = UltrasonicSensorProcessor()
    rospy.spin()
