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
        self.subscriber = rospy.Subscriber("/camera/color/image_jpeg", CompressedImage, self.callback, queue_size=1)

        # Create a publisher on the topic '/lane_detection/results'
        self.pub_results = rospy.Publisher('/lane_detection/results', CompressedImage, queue_size=3)

        # Setup a bridge from ROS to OpenCV
        self.bridge = CvBridge()

        # Setup timer to update parameters every 2 seconds
        rospy.Timer(rospy.Duration(2), lambda e: self.read_parameters())

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

        # List to track detection success for each lane line: [left, right] per frame, True if detected, False otherwise
        self.success_list = []

        # List to track center coordinates of detected lane lines for each frame: [left center, right center]
        self.center_list = []

        self.first_image = None

    def callback(self, data):
        try:
            # Convert the ROS compressed image to an OpenCV image
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
            if self.first_image is None:
                self.first_image = cv_image
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        else:
            # Call the detect function with the converted image
            self.detect(cv_image)

    def detect(self, image):

        self.im_preprocessed = self.preprocess_image(image)

        self.im_out = cv2.cvtColor(self.im_preprocessed, cv2.COLOR_GRAY2RGB)
        
        if self.state == "CALIBRATION":
            success_left, center_left = self.detect_left_line(image, self.win_xleft)
            success_right, center_right = self.detect_right_line(image, self.win_xright)

            if success_left and success_right:
                self.lane_width = center_right - center_left

        elif self.state == "BOTH_VISIBLE":
            success_left, center_left = self.detect_left_line(image)
            success_right, center_right = self.detect_right_line(image)

        elif self.state == "ONLY_LEFT_VISIBLE":
            success_right, center_right = False, self.center_list[-1][self.R_INDEX]

            success_left, center_left = self.detect_left_line(image)
            
            if success_left:
                imag_center_right = center_left + self.lane_width # imaginary center
                if imag_center_right < self.limit_right:
                    success_right, center_right = self.detect_right_line(image, imag_center_right)

        elif self.state == "ONLY_RIGHT_VISIBLE":
            success_left, center_left = False, self.center_list[-1][self.L_INDEX]

            success_right, center_right = self.detect_right_line(image)
            
            if success_right:
                imag_center_left = center_right - self.lane_width # imaginary center
                if imag_center_left > self.limit_left:
                    success_left, center_left = self.detect_left_line(image, imag_center_left)

        self.success_list.append([success_left, success_right])
        self.center_list.append([center_left, center_right])

        self.update_state()
        self.calculate_offset(image)

        try:

            # Convert the OpenCV image to a ROS compressed image
            result_image = self.bridge.cv2_to_compressed_imgmsg(image, dst_format='jpeg')

            # Publish the compressed image
            self.pub_results.publish(result_image)

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def preprocess_image(self, image):

        # Convert to grayscale
        self.im_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur to the grayscale image
        im_blur = cv2.GaussianBlur(self.im_gray, self.kernel, 0)

        # Apply Canny edge detection to the preprocessed image
        im_preprocessed = cv2.Canny(im_blur, self.canny_min, self.canny_max) # [0 or 255]
        
        return im_preprocessed


    def update_state(self):

        if not self.setup_done:
            return
        
        if len(self.success_list) < self.stop_threshold: return 

        success_left = any(row[self.L_INDEX] for row in self.success_list[-self.stop_threshold:])
        success_right = any(row[self.R_INDEX] for row in self.success_list[-self.stop_threshold:])

        if success_left and success_right:
            self.state = 'BOTH_VISIBLE'
        elif success_left:
            self.state ='ONLY_LEFT_VISIBLE'
        elif success_right:
            self.state = 'ONLY_RIGHT_VISIBLE'
        else:
            self.state = 'CALIBRATION'
            rospy.set_param('setup_done', False)

    def detect_left_line(self, image, center = None):
        success_left, center_left = self.sliding_window(image, self.L_INDEX, self.L_COLOR, center)
        return success_left, center_left

    def detect_right_line(self, image, center = None):
        success_right, center_right = self.sliding_window(image, self.R_INDEX, self.R_COLOR, center)
        return success_right, center_right

    def sliding_window(self, image, index, color, center=None):
        """
        Tracks the lane lines in an image using a sliding window approach.
        
        :param image: The input image on which to perform lane tracking.
        :param index: Column index in center_list used to select the initial center for tracking.
        :param color: The color used for visualization of windows (if visualize is True).
        :param center: Optional initial center position; defaults to the last known center.
        :return: A tuple (success flag, final lane center position).
        """

        self.path = []
        center = center or self.center_list[-1][index] # Use the center from the last frame if no center is provided.

        success, updated_center, next_center = self.find_center(image, center, self.win_y, color)

        if not success:
            return False, updated_center

        # Apply the sliding window technique if num_windows is greater than one. 
        # Iterate to track the lane line by moving the window up, adjusting vertically by win_h each step.
        cy = self.win_y
        for _ in range(self.num_windows - 1):
            cy -= self.win_h
            success, _, next_center = self.find_center(image, next_center, cy, color)

            if not success:
                break

        return True, updated_center


    def find_center(self, image, cx, cy, color):
        """
        Identifies the center of the lane line in a specific window of the image.
        
        :param image: The image in which to find the lane line center.
        :param cx: X-coordinate of the center of the search window.
        :param cy: Y-coordinate of the center of the search window.
        :param color: The color used to draw the search window (if visualize is True).
        :return: A tuple (success flag, updated center, next center).
        """
        # Calculate the search window coordinates, ensuring they are within image bounds
        x1 = max(0, int(cx - self.win_w / 2))
        y1 = max(0, int(cy - self.win_h / 2))
        x2 = min(self.image_w, x1 + self.win_w)
        y2 = min(self.image_h, y1 + self.win_h)

        # Draw the search window if debugging
        if self.visualize:
            self.draw_box(image, x1, y1, x2, y2, color)
        
        # Extract the relevant image patch and identify non-zero pixels (lane line pixels)
        patch = self.im_preprocessed[y1:y2, x1:x2]
        _, xs = np.nonzero(patch)
        
        # Check if any lane line pixels are detected
        # If not, return False along with the current center
        if len(xs) == 0:
            return False, cx, None

        # Calculate the new center based on the mean of the detected pixels
        updated_center = int(x1 + np.mean(xs))
        self.path += [updated_center]

        # Verify that the new center is within the acceptable range
        if updated_center < self.limit_left or updated_center > self.limit_right:
            return False, updated_center, None

        # If num_windows is set to one, skip searching for the center of the next window and return the final center.
        if self.num_windows == 1:
            return True, updated_center, None

        # Calculate the horizontal shift for the next center based on detected path
        if len(self.path) > 1:
            dx = self.path[-1] - self.path[-2]
        else:
            # Calculate the next position by determining the slope of white pixels in the first window.
            values = np.array([(np.mean(np.nonzero(row)), self.win_h - y) for y, row in enumerate(patch) if np.count_nonzero(row) > 0])
            dx = values[0,0] - values[-1,0]

        next_center = updated_center + dx

        return True, updated_center, next_center
    
    def validate_parameters(self):

        # Make sure that kernel is >= 3 and that it is an odd number
        k = max(3, self.k + (self.k % 2 == 0))
        self.kernel = (k, k)
        rospy.set_param('lane_detection/kernel', k)

        # Ensure win_xleft stays within the left half of the image
        self.win_xleft = max(self.win_w // 2, min(self.win_xleft, self.image_w // 2 - self.win_w // 2))
        rospy.set_param('lane_detection/win_xleft', self.win_xleft)

        # Ensure win_xright is set correctly within the right half of the image
        self.win_xright = max(self.image_w // 2 + self.win_w // 2, min(self.win_xright, self.image_w - self.win_w // 2))
        rospy.set_param('lane_detection/win_xright', self.win_xright) 

    def read_parameters(self):
        try:
            # Re-read parameters to see if they have been changed
            self.win_h = rospy.get_param('lane_detection/win_h')
            self.win_w = rospy.get_param('lane_detection/win_w')
            self.win_xleft = rospy.get_param('lane_detection/win_xleft')
            self.win_xright = rospy.get_param('lane_detection/win_xright')
            self.win_y = rospy.get_param('lane_detection/win_y')
            self.image_w = rospy.get_param('lane_detection/image_w')
            self.image_h = rospy.get_param('lane_detection/image_h')
            self.num_windows = rospy.get_param('lane_detection/num_windows')
            self.canny_min = rospy.get_param('lane_detection/canny_min')
            self.canny_max = rospy.get_param('lane_detection/canny_max')
            self.k = rospy.get_param('lane_detection/kernel')
            self.setup_done = rospy.get_param('lane_detection/setup_done')

        except rospy.ROSException as e:
            rospy.logwarn(f"Failed to retrieve parameter: {e}")
        
        self.validate_parameters()

        self.limit_left = int(0.03 * self.image_w)
        self.limit_right = int(0.97 * self.image_w)
        self.cam_center = self.image_w // 2

    # ---------------------------------
    # Controlling
    # ---------------------------------

    def calculate_offset(self, image):

        center_left, center_right = self.center_list[-1]

        if self.state == 'ONLY_LEFT_VISIBLE':
            self.veh_center = center_left + self.lane_width // 2
        elif self.state == 'ONLY_RIGHT_VISIBLE':
            self.veh_center = center_right - self.lane_width // 2
        else: # 'CALIBRATION' or 'BOTH_VISIBLE'
            self.veh_center = (center_left + center_right) // 2

        # Calculate and normalize the offset in pixels relative to the image position
        self.offset = (self.cam_center - self.veh_center) / self.image_w  # Positive for offset (+) turn left, negative for offset (-) turn right
        
        if self.state != 'CALIBRATION':
          self.offset_msg.data = self.offset
          self.pub_offset.publish(self.offset_msg)
        
        if self.visualize:
            self.plot_position(image)

    # ---------------------------------
    # Visualization
    # ---------------------------------
  
    def draw_vertical_line(self, image, x, y, color = (0, 255, 255), thickness = 1, length = 10):
        cv2.line(image, (x, y - length), (x, y + length), color, thickness)

    def draw_horizontal_line(self, image, x_start, x_end, y, color = (0, 255, 255), thickness = 1):
        cv2.line(image, (x_start, y), (x_end, y), color, thickness)
    
    def draw_horizontal_arrow(self, image, x_start, x_end, y, color = (0, 0, 255), thickness = 1, arrow_length=50):
        # Draw a vertical line at the end of the arrow
        self.draw_vertical_line(image, x_start, y, color, thickness)
    
        # Draw the horizontal arrow
        arrow_length *= np.sign(x_start - x_end)
        cv2.arrowedLine(image, (int(x_start + arrow_length), y), (x_start, y), color, thickness, tipLength=0.3)
    
    
    def draw_box(self, image, x1, y1, x2, y2, color=(0, 255, 0), thickness = 2):

        # Draw the box on the image
        cv2.rectangle(image, (x1, y1), (x2, y2), color=color, thickness=thickness)

        # Extract the patch from the grayscale image
        patch = self.im_out[y1:y2, x1:x2, :]

        # Replace the corresponding region in the RGB image with the patch
        image[y1:y2, x1:x2, :] = patch

        return image

    def plot_position(self, image):
        success_left, success_right = self.success_list[-1]
        center_left, center_right = self.center_list[-1]
        
        x1 = center_left if success_left else 0
        x2 = center_right if success_right else self.image_w
    
        self.draw_horizontal_line(image, x1, x2, self.win_y)
        
        for ctr, suc in zip([center_left, center_right], [success_left, success_right]):
            if suc:
                self.draw_vertical_line(image, ctr, self.win_y)
        
        
        self.draw_vertical_line(image, self.veh_center, self.win_y)
        
        self.draw_horizontal_arrow(image, self.cam_center, self.veh_center, self.win_y -35)
        
        cv2.putText(image, f'Offset: {self.offset:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)


if __name__ == '__main__':
    try:
        lane_det = LaneDetection(visualize = True)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
