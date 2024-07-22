#!/usr/bin/env python

import rospy
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Bool, Float32MultiArray
from nav_msgs.msg import Path
import time
from vehicle_control.srv import ChangeStatus, ChangeStatusResponse
pit_ride = False
mode_ad = None 
laststatus_joy = None

class AckermannToVesc:
    def __init__(self, dynamic_update = False, interval = 10):
        # Load configuration parameters for mapping and mode settings
        self.load_params()
        #self.class_names_str = rospy.get_param('/class_names_param')
        #self.class_names_str = self.class_names_str.strip('{}')
        #self.class_names_list = self.class_names_str.split(', ')
        #self.class_names_dict = {}
        #for item in self.class_names_list:
        #    key, value = item.split(': ')
        #    self.class_names_dict[int(key)] = value.strip("'")

        # Initialize modes as None indicating no mode is set initially
        self.mode = None
        
        # Initialize messages for publishing speed and servo position
        self.erpm_msg = Float64()
        self.servo_msg = Float64()
        
        # Initialize brake message
        self.brake_msg = Float64()
        self.brake_msg.data = self.brake_amps

        # Safety check parameters and subscriber
        self.speed_values = []  # Stores speed values for safety check
        n_seconds = 8  # Duration for the safety check in seconds
        hz = 40  # Expected number of speed values per second
        self.min_values = n_seconds * hz  # Minimum number of values for a valid safety check
        self.safety_sub = rospy.Subscriber('/rc/ackermann_cmd', AckermannDriveStamped, self.safety_check)
        
        # Driving command subscribers, initially not active
        self.rc_sub = None	 			# Messages from RC
        self.ad_sub = None	 			# Messages for autonomous driving
        self.speed_acc = 1000	 			# initialize speed for acc
        self.tl_green = False	 			# traffic light is not red
        self.tldet = False	 			# traffic light not detected
        self.pitin = False 	 			# parallelparking false
        self.pitout = False
        self.pathbox = Path()				# initialize path in box
        self.pathtrack = Path()			# initialize path on track
        self.last_timebox = rospy.Time.now()		# initialize timer for last received message box 
        self.last_timetrack = rospy.Time.now()	# initialize timer for last received message track
        
        # Joystick subscriber for mode updates
        self.joy_sub = rospy.Subscriber('/rc/joy', Joy, self.update_mode)

        # Mode Subscriber for mode updates within selfdriving mode
        self.pdc_mode_sub = rospy.Subscriber('/space_detected', Bool, self.update_self_driving_mode)
        
        # Subscriber for Pathplanning
        self.speed_acc_sub = rospy.Subscriber('/speed_acc', Float64, self.callback_acc)			# get speed from acc. If acc has not detected car, speed is infinite
        self.multar = rospy.Subscriber("/yolo/multi_array", Float32MultiArray, self.callbackmultar)		# get multiarray vom object detection
        # Subscribe to the /path topic
        self.pathbox_sub = rospy.Subscriber('/pathbox', Path, self.pathplanning)				# get the path of the box (from pathplanning)  (atm not available)
        self.pathtrack_sub = rospy.Subscriber('/pathtrack', Path, self.pathplanning)			# get the path of the track (from pathplanning)  (atm not available)
        
        # Publish Path to Stanley 
        self.path_pub = rospy.Publisher('/path', Path, queue_size=5)	# send the path to the controller
        self.pit_ride_pub = rospy.Publisher('/pit_ride', Bool, queue_size=1)					

        # Publishers for motor speed and servo position
        self.erpm_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
        self.servo_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=1) 
        self.brake_pub = rospy.Publisher('/commands/motor/brake', Float64, queue_size=1) 
        
        # Inform user about safety check procedure at the start of booting
        info_msg = ("Please activate 'Deadman' mode. Do not touch the throttle or steering for {} seconds. "
                    "Safety check ends when speed stays at 0 m/s during this time.").format(n_seconds)
        rospy.loginfo(info_msg)

        # Update params with rospy.Timer real-time adjustments via Foxglove.
        if dynamic_update:
            # Using lambda to ignore TimerEvent arg from rospy.Timer callback
            rospy.Timer(rospy.Duration(interval), lambda e: self.load_params())

    def callbackmultar(self, msg):   #Multiarray subscription from object detection
        global pit_ride
        self.multar = self.parse_multiarr(msg)
        self.class_names_str = rospy.get_param('/class_names_param') 						# Extract parameters from object detection
        self.class_names_str = self.class_names_str.strip('{}')
        self.class_names_list = self.class_names_str.split(', ')
        self.class_names_dict = {}
        for item in self.class_names_list:
            key, value = item.split(': ')
            self.class_names_dict[int(key)] = value.strip("'")							# Extract Infromation from multiarray
        if self.multar is not None:											# if the array is filled wih parameters
            self.multar = self.replace_class_ids_with_names(self.multar, self.class_names_dict)			# replace IDs with names
            for item in self.multar:											# Search multiarray and set booleans
                class_name = item.get('class_name')									# Important parameters: class name and confidence (security feature)
                confidence = item.get('confidence', 0)

                if class_name in ['green', 'orange'] and confidence > 0.8:						# detect if traffic light is green
                    self.tl_green = True
                
                if class_name in ['no_lights', 'orange', 'green', 'red', 'red_orange'] and confidence > 0.8:	# detect if traffic light is detected
                    self.tldet = True
                
                if class_name == 'pit_in' and confidence > 0.8:							# set pitride true
                    self.pitin = True
                    self.pitout = False
                    pit_ride = True
                    msg = Bool()
                    msg.data = True
                    self.pit_ride_pub.publish(msg)
                
                if class_name == 'pit_out' and confidence > 0.8:							# set pitride false
                    self.pitout = True
                    self.pitin = False
                    pit_ride = False
                    msg = Bool()
                    msg.data = False
                    self.pit_ride_pub.publish(msg)
                    
                if class_name in ['red', 'orange'] and confidence > 0.8:					# Reset tl_green if red or orange is detected						
                    self.tl_green = False 
                print("tl_green: {}, tldet: {}, pitin: {}, pitout: {}, pit_ride: {}".format(self.tl_green, self.tldet, self.pitin, self.pitout, pit_ride))
    
    
    def callback_acc(self, msg):											# set the subscibed data in self.speed_acc
        self.speed_acc = msg.data            
    
    def callback_tl(self, msg): 
    	self.tl_green = msg.data
    	
    def callback_tldet(self, msg): 
    	self.tldet = msg.data    	
    	
    def callback_boxcp(self, msg): 
    	self.boxcp = msg.data    	
    	
    def callback_boxpp(self, msg): 
    	self.boxpp = msg.data        	
    
    def callback_pathbox(self, msg): 							# set the data coming from the pathbox subscriber and put it into self.pathplanning for stanley 
    	self.pathbox = msg.data
    	self.last_timebox = rospy.Time.now()
    	self.pathplanning() 								# calling the pathpalanning function to generate a path msg to publish to stanley
    	
    def callback_pathtrack(self, msg): 						# set the data coming from the pathtrack subscriber and put it into self.pathplanning for stanley
    	self.pathtrack = msg.data
    	self.last_timetrack = rospy.Time.now()
    	self.pathplanning() 								# calling the pathpalanning function to generate a path msg to publish to stanley    	
    
    def replace_class_ids_with_names(self, multiarray, class_names_dict):
        for item in multiarray:
            class_id = item.get('class_id')
            if class_id in class_names_dict:
                item['class_name'] = class_names_dict[class_id]
        return multiarray
    
    def parse_multiarr(self, msg):							# Extract the dimensions from the message
        if msg.layout.dim:
            rows = msg.layout.dim[0].size
            cols = msg.layout.dim[1].size
        else:
            return

        data = []
        for i in range(rows):
            start_index = i * cols
            end_index = start_index + cols
            row = msg.data[start_index:end_index]
            class_id, x1, y1, x2, y2, confidence = row
            entry = {
                'class_id': int(class_id),
                'x1': int(x1),
                'y1': int(y1),
                'x2': int(x2),
                'y1': int(y1),
                'confidence': float(confidence)}
        
            data.append(entry)
    
        return data
    	
    def is_path_empty(self, path):							# Cecking of the subscribed path is empty
        if path is None or not path.poses:
            return True
        return False
    
    
    def pathplanning(self): 								# generate publisher data for stanley  
        if pit_ride:
            if self.is_path_empty(self.pathbox):					# if messsage for pitride path is empty, th car stays on the track
                path = self.pathtrack
            else:
                path = self.pathbox
        else: 										# if pitride is false, so no signs are deteced and there is a track of the box, the car drives into the bo 
            if self.is_path_empty(self.pathtrack):
                path = self.pathbox
            else:
                path = self.pathtrack							# otherwise the car drives on the track
            
        
            
        self.path_pub.publish(path)    
    
    
    def signal_calibration_complete(self):
        # Publish simple sinus sweep to indicate that the calibration was complete

        rate = rospy.Rate(40)
        
        amplitude = 0.2
        frequency = 3.0
        vertical_shift = 0.5

        t = np.linspace(0, np.pi, 60)
        values = amplitude * np.sin(frequency * t) + vertical_shift
        
        for value in values:
            self.servo_pub.publish(Float64(value))
            rate.sleep()
            
    
    def initialize_subscribers(self):							# Initialize subscribers for manual and autonomous driving commands, so these go to the callback.
        if self.rc_sub is None and self.ad_sub is None:
            self.rc_sub = rospy.Subscriber('/rc/ackermann_cmd', AckermannDriveStamped, lambda x: self.callback(x, self.manu_val))
            self.ad_sub = rospy.Subscriber('/autonomous/ackermann_cmd', AckermannDriveStamped, lambda x: self.callback(x, self.auto_val))
            self.pdc_sub = rospy.Subscriber('/pdc/ackermann_cmd', AckermannDriveStamped, lambda x: self.callback(x, self.pdc_val))

    def brake(self):									# Publishes the brake message hz times in rapid succession, causing a blocking effect.
        hz = 420
        
        rate = rospy.Rate(hz)  

        for _ in range(hz):  # Publish 1s
            self.brake_pub.publish(self.brake_msg)
            rate.sleep()
                
    def update_mode(self, msg):							# Update the driving mode based on joystick input
        global laststatus_joy
        global mode_ad
        new_mode = msg.buttons[self.mode_btn]						# New mode is set to button mode					 
        laststatus_joy = new_mode							# laststaus is also set to button mode
        if new_mode != self.mode: 							# if the mode changes the car brakes
            self.brake() # emergency brake
            
            self.mode = new_mode							# mode is set to the new mode
            if self.mode == self.dead_val: 						# if mode is deadman the autonmous driving mode is set to none
                mode_ad = None
            
            mode_name = {self.dead_val: "Deadman", self.auto_val: "Autonomous", self.manu_val: "Manual"}.get(self.mode, "Unknown")	
            rospy.loginfo("Mode changed to: {}".format(mode_name))			# Send changed mode to terminal
            
    def update_self_driving_mode(self, msg):						# Update the driving mode based on the selfdriving input (PDC or Autonomous)
        global mode_ad	
        global laststatus_joy
        space_detected = msg.data  
         
        if space_detected != True:					# after leaving the parking space the space_detected is set to false, so the autonomous driving mode is set to the last status
            mode_ad = laststatus_joy   
            return        
        new_mode = self.pdc_val					# if space detected is true the new mode is PDC, so the Parking team can continue their parking
        
        if new_mode != mode_ad: 
            self.brake() # emergency brake				# if the mode changes the car brakes
            #mode_ad = new_mode    					# setting the autonomous mode to the new mode
            #mode_name = {self.pdc_val: "PDC"}.get(self.mode)
            #rospy.loginfo("Mode changed to: {}".format(mode_name))	# Send changed mode to terminal
            
    def safety_check(self, ackermann_msg):
        """Perform safety checks before enabling driving commands."""
        if self.mode != self.dead_val:
            return
        
        speed = ackermann_msg.drive.speed				# Track vehicle speed to ensure it remains at 0 during the safety check
        self.speed_values.append(speed)
        if len(self.speed_values) > self.min_values:
            self.speed_values.pop(0)
            if max(self.speed_values) == 0 and min(self.speed_values) == 0:
                rospy.loginfo("Calibration complete!")
                self.safety_sub.unregister()  			# Unsubscribe after passing the safety check
                self.signal_calibration_complete() 			# Publish simple sinus sweep to indicate that the calibration was complete
                self.initialize_subscribers()  			# Activate driving command subscribers
                
                
    def callback(self, ackermann_msg, target):								#Process received driving commands based on the current mode.
        global mode_ad  
        global pit_ride 
        
        if self.mode == self.manu_val and (mode_ad == self.manu_val or mode_ad is None):			# Option 1: The targeted mode is manu_val 
        	if target == self.auto_val or target == self.pdc_val:
        		return
        		
        if self.mode == self.auto_val and (mode_ad == self.auto_val or mode_ad is None): 			# Option 2: The targeted mode is auto_val
        	if target == self.manu_val or target == self.pdc_val:
        		return
        		
        if self.mode == self.manu_val and mode_ad == self.pdc_val: 						# Option 3: The targeted mode is pdc_val with mode_rc = manu_val
        	if target == self.auto_val or target == self.manu_val:
        		return
        		
        if self.mode == self.auto_val and mode_ad == self.pdc_val: 						# Option 4: The targeted mode is pdc_val with mode_rc = auto_val
        	if target == self.auto_val or target == self.manu_val:
        		return
        		
        if self.mode == self.dead_val: 									# Option 5: The targeted mode is deadman
        	return
        #print("Modus: ",self.mode)
        #print("Modusad: ",mode_ad)		
        # Convert Ackermann message to VESC-compatible commands
        steering_angle = ackermann_msg.drive.steering_angle
        speed = ackermann_msg.drive.speed
        if pit_ride:      # because pathplanner and object detection are not working at the same time. for demonstration purposes
            if speed >= 0.5: 
                speed = 0.5 
        
        
        # change the speed according to acc or stanley. The priority is on the stanley speed. If the speed_acc is bigger than the speed of the stanley. The speed is stanley speed.
        if self.speed_acc <= speed:
        	speed = self.speed_acc
        
        # Change the speed, if traffic light is red
        if self.tl_green != True and self.tldet == True: 
            speed = 0
            self.brake()
            
        
        		
        erpm = self.speed_to_erpm_gain * speed
        servo_value = self.servo_mid + steering_angle * self.steer_to_servo_gain

        # Ensure servo commands are within limits
        self.servo_msg.data = max(min(servo_value, self.servo_max), self.servo_min)
        self.erpm_msg.data = erpm

        # Publish commands to VESC
        self.servo_pub.publish(self.servo_msg)
        

        if abs(erpm) < self.erpm_min:
            self.brake_pub.publish(self.brake_msg)
        else:
            self.erpm_pub.publish(self.erpm_msg)

    def load_params(self):
        self.servo_mid = rospy.get_param("/servo_mid")
        self.servo_max = rospy.get_param("/servo_max")
        self.servo_min = rospy.get_param("/servo_min")
        
        self.erpm_min = rospy.get_param("/erpm_min")
        self.brake_amps = rospy.get_param("/brake_amps")
        
        self.speed_to_erpm_gain = rospy.get_param("/speed_to_erpm_gain")
        self.steer_to_servo_gain = rospy.get_param("/steer_to_servo_gain")
        
        self.dead_val = rospy.get_param("/rc_dead_value")  # Deadman switch
        self.auto_val = rospy.get_param("/rc_auto_value")  # Drive autonomously
        self.manu_val = rospy.get_param("/rc_manu_value")  # Drive manually
        self.pdc_val = rospy.get_param("/rc_pdc_value")  # Drive pdc
        
        self.mode_btn = rospy.get_param("/rc_mode_button")
        
    def monitor_pathbox(self):					# if no new data is published by path.box the data is set to none after five seconds, 
        while not rospy.is_shutdown():				# to guarantee no old data when leaving the box
            current_time = rospy.Time.now()
            elapsed_time = current_time - self.last_timebox
            

            if elapsed_time.to_sec() > 5:
                self.pathbox = None      
                	
    def monitor_pathtrack(self):					# if no new data is published by path.track the data is set to none after five seconds, 
        while not rospy.is_shutdown():				# to guarantee no old data when leaving the box
            current_time = rospy.Time.now()
            elapsed_time = current_time - self.last_timetrack
            

            if elapsed_time.to_sec() > 5:
                self.pathtrack = None
                
                
def handle_change_status(req):
    global pit_ride
    global mode_ad
    global laststatus_joy
    rospy.loginfo("Received request from sender: {}, status: {}".format(req.sender, req.status))
    # Check if status is true, sender is "parking", and self.pitt_ride is true
    if req.status and req.sender == "parking" and pit_ride:						# if allowed to drive into the box and sender is set to 'parking' change mode_ad to pdc. 
        print("dgdfg")											# This enables the parking team to gain access to ackermann.
        success = True
        mode_ad = 3
        AckermannToVesc().brake()
        print("Modusad", mode_ad)
    else:
        success = False										# else: mode_ad will be set to last status
        mode_ad = laststatus_joy
        
    return ChangeStatusResponse(success)
#rosservice call /change_status "{status: true, sender: 'parking'}"


if __name__ == '__main__':
    rospy.init_node('ackermann_to_vesc', anonymous=True)
    # to initialize server
    s = rospy.Service('change_status', ChangeStatus, handle_change_status)
    rospy.loginfo("Ready to change status.")
   
    A2V = AckermannToVesc(dynamic_update = False)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ackermann_to_vesc")
