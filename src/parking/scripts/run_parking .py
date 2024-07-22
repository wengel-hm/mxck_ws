#!/usr/bin/env python
# -*- coding: utf-8 -*-

import string
import rospy
import rospkg
from std_msgs.msg import Int16MultiArray, Float32MultiArray
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Int32
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import time
import rosbag
import os
from std_msgs.msg import Bool
from statistics import median
from collections import deque
from vehicle_control.srv import ChangeStatus, ChangeStatusRequest




class ParkingSpace:

    def __init__(self):
        self.set_variables()
        self.multar = rospy.Subscriber("/yolo/multi_array", Float32MultiArray, self.callbackmultar)
        
        
    def callbackmultar(self, msg):   #Multiarray subscription from object detection
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

                if class_name == 'cross_parking' and confidence > 0.9:						
                    self.type = 2
                
                
                if class_name == 'parallel_parking' and confidence > 0.9:							
                    self.type = 1
                 
                self.detect_type_change()
    
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



    def set_variables(self):
        self.length = None
        self.type = 0 # 0: no sign detected; 1: parallel sign detected; 2: cross sign detected
        self.type_old = 0
        self.ready = False 
        self.detected = False


    def detect_type_change(self):
        if self.type == self.type_old:
            pass
        else:
            print("parking type", self.type)
            self.type_old = self.type      



class Parker:

    def __init__(self):

        base_dir = rospkg.RosPack().get_path("parking")
        bag_dir = base_dir + "/bagfiles"

        self.r = rospy.Rate(25)

        self.path_cross = bag_dir + "/cross_inparking_manuvre_fix.bag"
        self.path_parallel = bag_dir + "/parallel_inparking_manuvre_fix.bag"
        self.path_ausparken_cross = bag_dir + "/cross_outparking_manuvre_fix.bag"
        self.path_ausparken_parallel = bag_dir + "/parallel_outparking_manuvre_fix.bag"
        self.ackermann_pub = rospy.Publisher("/pdc/ackermann_cmd", AckermannDriveStamped, queue_size=10)
        self.ackMsg = AckermannDriveStamped()
        self.space_detected_pub = rospy.Publisher("/space_detected", Bool, queue_size=1)
        self.pdc_sub = rospy.Subscriber("uss_values", Int16MultiArray, self.parking_callback)
        self.pdc_pub_0 = rospy.Publisher("/pdc_pub_0", Int32, queue_size=1)
        self.pdc_pub_1 = rospy.Publisher("/pdc_pub_1", Int32, queue_size=1)
        self.set_variables()

        self.PARALLEL = 1
        self.CROSS = 2
        self.NONE = 0
        self.STEERING_OFFSET = -0.1
        self.PARKING_SPEED = 0.5

        self.filter_window_size = 5
        self.distance0_filtered = 0
        self.distance1_filtered = 0
        self.distance0_puff = deque(maxlen=self.filter_window_size)
        self.distance1_puff = deque(maxlen=self.filter_window_size)


    def set_variables(self):
        self.state = 0
        self.parkingspace_measured = 0
        self.e_previous = 0
        self.derivative = 0
        self.time = 0
        self.time_float = 0
        self.time_save_start = 0
        self.time_save_start_float = 0
        self.time_save_end = 0
        self.time_save_end_float = 0
        self.delta_time_save = 0
        self.such_reset = 0
        self.time_safe = 0
        self.time_safe_float = 0
        self.distance0_1 = []
        self.distance1_1 = []
        self.distance0_2 = []
        self.distance1_2 = []
        self.distance0 = []
        self.distance1 = []
        self.timestamps = []
        self.parkspace = ParkingSpace()
        self.t_previous = rospy.Time.now()
        self.parkspace.detected = False
        msg = Bool()
        msg.data = False

    def parking_callback(self, data):

        self.distance0_puff.append(data.data[0])
        self.distance1_puff.append(data.data[1])

        self.distance0_puff = deque([40 if x < 0 else x for x in self.distance0_puff], maxlen=self.filter_window_size)
        self.distance1_puff = deque([40 if x < 0 else x for x in self.distance1_puff], maxlen=self.filter_window_size)

        self.distance0_filtered = median(self.distance0_puff)
        self.distance1_filtered = median(self.distance1_puff)
        
        if self.parkspace.type != self.NONE:
            if self.parkspace.ready == True:
                if self.parkspace.type == self.PARALLEL:
                    self.parking_parallel()
                elif self.parkspace.type == self.CROSS:
                    self.parking_cross()
            elif self.parkspace.detected == True:
                self.go_to_startposition()
            else:
                self.parksearch()
        
        dt = 0.3
        delta = self.e_previous - self.distance0_filtered
        self.derivative = delta / dt

        self.e_previous = self.distance0_filtered
        
        distance0_filtered_to_publish = Int32()
        distance1_filtered_to_publish = Int32()
        
        distance0_filtered_to_publish.data = self.distance0_filtered
        distance1_filtered_to_publish.data = self.distance1_filtered
        
        self.pdc_pub_0.publish(distance0_filtered_to_publish)
        self.pdc_pub_1.publish(distance1_filtered_to_publish)

    def parksearch(self):

        self.time = rospy.Time.now()
        self.time_float = self.time.to_sec()

        if self.time_float > 1:

            if self.derivative < -100 and self.time_save_start_float ==0:
                self.time_save_start = rospy.Time.now()
                self.time_save_start_float = self.time_save_start.to_sec()
                print("Parkingspace start found")
            elif self.derivative > 100 and self.time_save_start_float !=0:
                self.time_save_end = rospy.Time.now()
                self.time_save_end_float = self.time_save_end.to_sec()
                print("Parkingspace end found")
            elif self.time_save_start_float != 0 and self.time_save_end_float !=0 and self.delta_time_save == 0:
                self.delta_time_save = (self.time_save_end_float - self.time_save_start_float)
                print("Passed time: ",self.delta_time_save)
            elif self.delta_time_save !=0:
                self.parkspace_calculator()
            elif self.time_save_start_float !=0:
                print("Waiting for parkingspace end")
                self.time_safe = rospy.Time.now()
                self.time_safe_float = self.time_safe.to_sec()
                self.such_reset = self.time_safe_float - self.time_save_start_float
                if self.such_reset > 5:
                    self.time_save_start = 0
                    print("No ending detected - Timeout")
                    self.time_save_start_float = 0
                    self.time_save_start = 0
                    self.time_save_end_float = 0
                    self.time_save_end = 0
                else:
                    pass
            else:
                print("Searching for possible parkingspaces")
        else:
            pass


    def parkspace_calculator(self): #Calculation for 0.5 m/s
        self.delta_time_save = self.delta_time_save

        if self.delta_time_save > 0.6: #v=d/t d=v*t t=d/v 70cm
            if self.delta_time_save < 2: #100cm
                print("!!!Parallelparking detected!!!")
                self.parkspace.length = (0.35 * self.delta_time_save) * 100
                print("Parkingspace length: ",self.parkspace.length)
                self.parkingspace_measured = 1
                self.parkspace_positiv()
            else:
                print("!!!Faulty parkingspace!!! - Parallelparking expected")
                self.time_save_start_float = 0
                self.time_save_end_float = 0
                self.delta_time_save = 0
        elif self.delta_time_save > 0.15: #40cm
            if self.delta_time_save < 0.8: #60cm
                print("!!!Crossparking detected!!!")
                self.parkspace.length = (0.35 * self.delta_time_save) * 100
                print("Parkingspace length: ",self.parkspace.length)
                self.parkingspace_measured = 2
                self.parkspace_positiv()
            else:
                print("!!!Faulty Parkingspace!!! - Crossparking expected")
                self.time_save_start_float = 0
                self.time_save_end_float = 0
                self.delta_time_save = 0
        else:
            print("That wasn't a Parkingspace")
            self.time_save_start_float = 0
            self.time_save_end_float = 0
            self.delta_time_save = 0



    def parkspace_positiv(self):
        
        if self.parkingspace_measured == self.parkspace.type:
            print("requesting Service call")
            req = ChangeStatusRequest()
        
            change_status = rospy.ServiceProxy('change_status', ChangeStatus)
            response = change_status(True, "parking")
            rospy.loginfo("Service call successful. Response: {}".format(response))    
            if response.success:
                print("Measured and Prediction match - Service call successful")
            
            
                self.parkspace.detected = True
            
            else:
                print("No Permission obtained")
                self.time_save_start_float = 0
                self.time_save_end_float = 0
                self.delta_time_save = 0
        else:
            print("Measured and Prediction don't match - continue searching")
            self.time_save_start_float = 0
            self.time_save_end_float = 0
            self.delta_time_save = 0


    def go_to_startposition(self):

        self.r = rospy.Rate(30)

        print("Driving to startposition") #85cm forward t ~ 1.7s

        for _ in range(40):
            self.ackMsg.drive.steering_angle = self.STEERING_OFFSET
            self.ackMsg.drive.speed = self.PARKING_SPEED
            self.ackermann_pub.publish(self.ackMsg)
            self.r.sleep()

        for _ in range(50):
            self.ackMsg.drive.steering_angle = self.STEERING_OFFSET
            self.ackMsg.drive.speed = self.PARKING_SPEED
            self.ackermann_pub.publish(self.ackMsg)
            self.r.sleep()
        
        for _ in range(30):
            self.ackMsg.drive.steering_angle = 0
            self.ackMsg.drive.speed = 0
            self.ackermann_pub.publish(self.ackMsg)
            self.r.sleep()

        print("reached Startposition")

        self.parkspace.ready=True


    def parking_parallel(self):

        print("Starting parallel parking process")
        self.r2 = rospy.Rate(30)

        for _ in range(27):
            self.ackMsg.drive.steering_angle = self.STEERING_OFFSET
            self.ackMsg.drive.speed = -self.PARKING_SPEED
            self.ackermann_pub.publish(self.ackMsg)
            self.r2.sleep()


        for index, (topic, msg, t) in enumerate(rosbag.Bag(self.path_parallel).read_messages(topics=["/autonomous/ackermann_cmd"])):
            if index == 15:
                for _ in range(45):
                    self.ackMsg.drive.steering_angle = 0
                    self.ackMsg.drive.speed = -self.PARKING_SPEED
                    self.ackermann_pub.publish(self.ackMsg)
                    self.r2.sleep()

            elif index > 20 and index < 210:
                if index < 40:
                    self.ackMsg.drive.steering_angle = 0.4
                    self.ackMsg.drive.speed = -self.PARKING_SPEED
                    self.ackermann_pub.publish(self.ackMsg)
                    self.r2.sleep()
                else:
                    self.ackMsg.drive.steering_angle = -0.4
                    self.ackMsg.drive.speed = -self.PARKING_SPEED + 0.1
                    self.ackermann_pub.publish(msg)
                    self.r2.sleep()

        for index in range (10):
            self.ackMsg.drive.steering_angle = self.STEERING_OFFSET
            self.ackMsg.drive.speed = self.PARKING_SPEED
            self.ackermann_pub.publish(self.ackMsg)
            self.r2.sleep()

        for index in range (10):
            self.ackMsg.drive.steering_angle = 0.4
            self.ackMsg.drive.speed = self.PARKING_SPEED
            self.ackermann_pub.publish(self.ackMsg)
            self.r2.sleep()

        for index in range (8):
            self.ackMsg.drive.steering_angle = -0.4
            self.ackMsg.drive.speed = -self.PARKING_SPEED
            self.ackermann_pub.publish(self.ackMsg)
            self.r2.sleep()

        for index in range (1):
            self.ackMsg.drive.steering_angle = 0.4
            self.ackMsg.drive.speed = self.PARKING_SPEED
            self.ackermann_pub.publish(self.ackMsg)
            self.r2.sleep()

        rospy.sleep(10)

        print("Getting groceries and leaving parkingspace in 10 seconds")

        for _ in range(300):
            self.r.sleep()

        for index in range(230):
            if index < 30:
                self.ackMsg.drive.steering_angle = 0.4
                self.ackMsg.drive.speed = -(self.PARKING_SPEED)
                self.ackermann_pub.publish(self.ackMsg)
                self.r2.sleep()
            elif index < 73:
                self.ackMsg.drive.steering_angle = -0.44
                self.ackMsg.drive.speed = (self.PARKING_SPEED)
                self.ackermann_pub.publish(self.ackMsg)
                self.r2.sleep()
            elif index < 110:
                self.ackMsg.drive.steering_angle = 0
                self.ackMsg.drive.speed = -(self.PARKING_SPEED)
                self.ackermann_pub.publish(self.ackMsg)
                self.r2.sleep()
            elif index < 170:
                self.ackMsg.drive.steering_angle = -0.44
                self.ackMsg.drive.speed = (self.PARKING_SPEED)
                self.ackermann_pub.publish(self.ackMsg)
                self.r2.sleep()
            elif index < 172:
                self.ackMsg.drive.steering_angle = self.STEERING_OFFSET
                self.ackMsg.drive.speed = (self.PARKING_SPEED)
                self.ackermann_pub.publish(self.ackMsg)
                self.r2.sleep()
            elif index < 220:
                self.ackMsg.drive.steering_angle = 0.44
                self.ackMsg.drive.speed = (self.PARKING_SPEED)
                self.ackermann_pub.publish(self.ackMsg)
                self.r2.sleep()
            else:
                self.ackMsg.drive.steering_angle = self.STEERING_OFFSET
                self.ackMsg.drive.speed = (self.PARKING_SPEED)
                self.ackermann_pub.publish(self.ackMsg)
                self.r2.sleep()
        
        for _ in range(60):
            self.r.sleep()

        print("Successful Parkmanuvre")
        
        req = ChangeStatusRequest()
        
        change_status = rospy.ServiceProxy('change_status', ChangeStatus)
        response = change_status(False, "parking")
        rospy.loginfo("Service call successful. Response: {}".format(response))  
        self.set_variables()
        self.parkspace.set_variables()



    def parking_cross(self):

        print("Starting cross parking process")
        self.r2 = rospy.Rate(30)

        for _ in range(25):
            self.ackMsg.drive.steering_angle = self.STEERING_OFFSET
            self.ackMsg.drive.speed = -self.PARKING_SPEED
            self.ackermann_pub.publish(self.ackMsg)
            self.r2.sleep()


        for index, (topic, msg, t) in enumerate(rosbag.Bag(self.path_cross).read_messages(topics=["/autonomous/ackermann_cmd"])):
            if index == 55:
                for _ in range(18):
                    self.ackMsg.drive.steering_angle = 0
                    self.ackMsg.drive.speed = -self.PARKING_SPEED
                    self.ackermann_pub.publish(self.ackMsg)
                    self.r2.sleep()

            elif index > 55 and index < 165:
                self.ackermann_pub.publish(msg)
                self.r2.sleep()
            elif index >= 166:
                if index < 198:
                    self.ackMsg.drive.steering_angle = 0.44
                    self.ackMsg.drive.speed = -self.PARKING_SPEED
                    self.ackermann_pub.publish(self.ackMsg)
                    self.r2.sleep()
                elif index < 202:
                    self.ackMsg.drive.steering_angle = self.STEERING_OFFSET
                    self.ackMsg.drive.speed = -self.PARKING_SPEED
                    self.ackermann_pub.publish(self.ackMsg)
                else:
                    self.ackMsg.drive.steering_angle = self.STEERING_OFFSET
                    self.ackMsg.drive.speed = 0
                    self.ackermann_pub.publish(self.ackMsg)
        
        rospy.sleep(10)
        
        print("Getting groceries and leaving parkingspace in 10 seconds")

        rospy.sleep(10)

        for index, (topic, msg, t) in enumerate(rosbag.Bag(self.path_ausparken_cross).read_messages(topics=["/autonomous/ackermann_cmd"])):
            msg.drive.speed = self.PARKING_SPEED
            self.ackermann_pub.publish(msg)
            self.r2.sleep()
        
        for _ in range(60):
            self.r.sleep()
            
        print("Successful Parkmanuvre")
        
        req = ChangeStatusRequest()
        
        change_status = rospy.ServiceProxy('change_status', ChangeStatus)
        response = change_status(False, "parking")
        rospy.loginfo("Service call successful. Response: {}".format(response))   
        self.set_variables()
        self.parkspace.set_variables()


if __name__ == "__main__":
    rospy.init_node("parking", anonymous = False)

    parking = Parker()
    parkingspace = ParkingSpace()

    rospy.spin()
