#!/usr/bin/env python3

import rospy
import rospkg
import cv2
import os
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import matplotlib.pyplot as plt

class YOLOV8:
    def __init__(self):
        
        # Initialize the ROS node
        rospy.init_node('yolo', anonymous=True)

        # Setup a bridge to convert ROS image messages to OpenCV format
        self.bridge = CvBridge()

        # Set the directory where the models are stored
        self.model_dir = os.path.join(rospkg.RosPack().get_path("object_detection"), "models")
        
        # Initialize different YOLO models for different tasks
        self.detection_model = YOLO(os.path.join(self.model_dir, "yolov8n.pt"))
        self.segmentation_model = YOLO(os.path.join(self.model_dir, "lane_v2.pt"))
        self.pose_model = YOLO(os.path.join(self.model_dir, "yolov8n-pose.pt"))

        # Choose mode
        self.mode = None
        self.update_mode()

        # Setup timer to update parameters every 10 seconds
        rospy.Timer(rospy.Duration(10), lambda e: self.update_mode())

        # Setup publisher for compressed images
        self.pub_compressed = rospy.Publisher("/yolo/results", CompressedImage, queue_size=1)

        # Setup subscriber to receive compressed images
        self.subscriber = rospy.Subscriber("/camera/color/image_jpeg", CompressedImage, self.callback, queue_size=1)

    def callback(self, data):
        try:
            # Convert the ROS compressed image to an OpenCV image
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
            # Process the image using the current model
            results = self.model(cv_image)

            # Publish the processed image
            self.publish_jpeg_compressed(data.header.stamp.to_sec(), results[0].plot())
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def publish_jpeg_compressed(self, timestamp, image):
        try:
            success, compressed_image = cv2.imencode('.jpg', image)
            if success:
                jpeg_msg = CompressedImage()
                jpeg_msg.header.stamp = rospy.Time.from_sec(timestamp)
                jpeg_msg.format = "jpeg"
                jpeg_msg.data = np.array(compressed_image).tobytes()
                self.pub_compressed.publish(jpeg_msg)
        except Exception as e:
            rospy.logerr(f"Error publishing image: {e}")

    def update_mode(self):
        try:
            # Re-read the mode parameter
            mode = rospy.get_param('/yolo/mode', 'detection')
            if mode not in ['segmentation', 'detection', 'pose']:
                rospy.logwarn("Invalid mode specified. Valid modes are: segmentation, detection, pose")
                return

            # Update the model based on the mode
            if mode != self.mode:
                self.mode = mode
                if mode == 'detection':
                    self.model = self.detection_model
                elif mode == 'segmentation':
                    self.model = self.segmentation_model
                elif mode == 'pose':
                    self.model = self.pose_model
                rospy.loginfo(f"Switched to {mode} mode.")

        except rospy.ROSException as e:
            rospy.logwarn(f"Failed to retrieve parameter: {e}")

if __name__ == '__main__':
    # Instantiate the YOLOv8 object
    yolo = YOLOV8()
    # Keep the node running
    rospy.spin()
