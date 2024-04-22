#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from std_msgs.msg import Float32

class LaneDetection:
    def __init__(self, visualize = False):
        rospy.init_node('lane_detection', anonymous=True)

        # Read parameters
        self.read_parameters()

        # Setup subscriber for compressed image
        self.subscriber = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.callback, queue_size=1)

        # Create a publisher on the topic '/lane_detection/results'
        self.pub_results = rospy.Publisher('/lane_detection/results', CompressedImage, queue_size=3)

        # Setup a bridge from ROS to OpenCV
        self.bridge = CvBridge()

        # Setup timer to update parameters every 2 seconds
        rospy.Timer(rospy.Duration(2), self.update_parameters)

        # Offset between vehicle and lane center, normalized to image width for resolution independence.
        # E.g., an offset of 0.2 corresponds to 0.2 * image_width in pixels.
        self.offset_msg = Float32()

        # Publishes normalized vehicle-lane offset to maintain a consistent lateral position.
        self.pub_offset = rospy.Publisher('/lane_center_offset', Float32, queue_size=1)


        # Setup
        self.stop_threshold = 3 # Specifies the maximum number of consecutive frames without lane line detection before the vehicle stops.
        self.L_INDEX, self.R_INDEX = 0, 1 # Indexes for left (0) and right (1) lane columns.
        self.visualize = visualize  # Set to True to enable visual display of sliding window results for debugging or demonstration purposes.
        self.state = 'CALIBRATION'
        self.L_COLOR, self.R_COLOR = (0, 125, 255), (255, 125, 0)

    def callback(self, data):
        try:
            # Convert the ROS compressed image to an OpenCV image
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        else:
            # Call the detect function with the converted image
            self.detect(cv_image)

    def detect(self, image):

        self.im_preprocessed = self.preprocess_image(image)

        try:

            # Convert the OpenCV image to a ROS compressed image
            result_image = self.bridge.cv2_to_compressed_imgmsg(image, dst_format='jpeg')

            # Publish the compressed image
            self.pub_results.publish(result_image)

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            

    def validate_parameters(self):

        # Make sure that kernel is >= 3 and that it is an odd number
        self.kernel = max(3, self.kernel + (self.kernel % 2 == 0))
        rospy.set_param('kernel', self.kernel)

        # Ensure win_xleft stays within the left half of the image
        self.win_xleft = min(self.win_w // 2, max(self.win_xleft, self.image_w // 2 - self.win_w // 2))
        rospy.set_param('win_xleft', self.win_xleft)

        # Ensure win_xright is set correctly within the right half of the image
        self.win_xrigth = min(self.image_w // 2 + self.win_w // 2, max(self.win_xright, self.image_w - self.win_w // 2))
        rospy.set_param('win_xrigth', self.win_xrigth) 

    def read_parameters(self, event):
        try:
            # Re-read parameters to see if they have been changed
            self.win_h = rospy.get_param('~win_h')
            self.win_w = rospy.get_param('~win_w')
            self.win_xleft = rospy.get_param('~win_xleft')
            self.win_xright = rospy.get_param('~win_xright')
            self.win_y = rospy.get_param('~win_y')
            self.image_w = rospy.get_param('~image_w')
            self.image_h = rospy.get_param('~image_h')
            self.num_windows = rospy.get_param('~num_windows')
            self.canny_min = rospy.get_param('~canny_min')
            self.canny_max = rospy.get_param('~canny_max')
            self.kernel = rospy.get_param('~kernel')
            self.setup_done = rospy.get_param('~setup_done')

        except rospy.ROSException as e:
            rospy.logwarn(f"Failed to retrieve parameter: {e}")
        
        self.validate_parameters()

        self.limit_left = int(0.03 * self.image_w)
        self.limit_right = int(0.97 * self.image_w)
        self.cam_center = self.image_w // 2




if __name__ == '__main__':
    try:
        lane_det = LaneDetection()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
