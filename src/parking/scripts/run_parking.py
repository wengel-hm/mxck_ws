#!/usr/bin/env python
# -*- coding: utf-8 -*-

import string
import rospy
import rospkg
from std_msgs.msg import Int16MultiArray
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




class ParkingSpace:

    def __init__(self):
        self.object_sub = rospy.Subscriber("/vorhersage_topic", Int32, self.detect_type)
        self.set_variables()



    def set_variables(self):
        self.length = None
        self.type = 2 # 0: keine Luecke; 1: parallel; 2: cross
        self.ready = False 
        self.detected = False
        self.object_array = np.zeros(5)
        self.verification_counter = 0


    def detect_type(self, data):
        if self.type == 0:
            self.object_array[self.verification_counter % 5] = data.data
            if self.verification_counter >= 4:
                if np.sum(self.object_array)/5 == 1:
                    self.type = 1
                elif np.sum(self.object_array)/5 == 2:
                    self.type = 2
            self.verification_counter += 1
        else:
            self.object_array = np.zeros(5)
            self.verification_counter = 0         



class Parker:

    def __init__(self):

        base_dir = rospkg.RosPack().get_path("parking")
        bag_dir = base_dir + "/bagfiles"

        self.r = rospy.Rate(25)

        self.path_cross = bag_dir + "/cross_inparking_manuvre_fix.bag"
        self.path_parallel = bag_dir + "/parallel_inparking_manuvre_fix.bag"
        self.path_ausparken_cross = bag_dir + "/cross_outparking_manuvre_fix.bag"
        self.path_ausparken_parallel = bag_dir + "/parallel_outparking_manuvre_fix.bag"
        #self.ackermann_pub = rospy.Publisher("/pdc/ackermann_cmd", AckermannDriveStamped, queue_size=10)
        self.ackermann_pub = rospy.Publisher("/autonomous/ackermann_cmd", AckermannDriveStamped, queue_size=10)
        self.ackMsg = AckermannDriveStamped()
        self.space_detected_pub = rospy.Publisher("/space_detected", Bool, queue_size=1)
        self.pdc_sub = rospy.Subscriber("uss_values", Int16MultiArray, self.parking_callback)
        self.pdc_pub_0 = rospy.Publisher("/pdc_pub_0", Int32, queue_size=1)
        self.pdc_pub_1 = rospy.Publisher("/pdc_pub_1", Int32, queue_size=1)
        self.set_variables()
        #self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)

        self.PARALLEL = 1
        self.CROSS = 2
        self.STRAIGHT = 1
        self.DECREASE = 2
        self.INCREASE = 3
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
        self.space_detected_pub.publish(msg)


    # def joy_callback(self,msg):
    #     self.dead_btn = msg.buttons[0]



    def parking_callback(self, data):
        #self.distance0 = data.data[0]
        #self.distance1 = data.data[1]

        self.distance0_puff.append(data.data[0])
        self.distance1_puff.append(data.data[1])

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
                # self.parkspace.ready = True #nur zum debuggen
            else:
                #self.states()
                #self.positioning()
                self.parksuche()
        

        #t = rospy.Time.now()
        #dt = (t - self.t_previous).to_sec()
        dt = 0.2
        delta = self.e_previous - self.distance0_filtered
        self.derivative = delta / dt

        #self.t_previous = t
        self.e_previous = self.distance0_filtered
        
        distance0_filtered_to_publish = Int32()
        distance1_filtered_to_publish = Int32()
        
        distance0_filtered_to_publish.data = self.distance0_filtered
        distance1_filtered_to_publish.data = self.distance1_filtered
        
        self.pdc_pub_0.publish(distance0_filtered_to_publish)
        self.pdc_pub_1.publish(distance1_filtered_to_publish)


    def states(self): #irrelevant

        if self.distance1_filtered > 20:
            self.state = self.DECREASE
        elif self.distance1_filtered < 20:
            self.state = self.INCREASE
        else:
            if self.distance0_filtered == self.distance1_filtered:
                self.state = self.STRAIGHT
            elif self.distance0_filtered > self.distance1_filtered:
                self.state = self.DECREASE
            else:
                self.state = self.INCREASE


    def positioning(self): #irrelevant

        self.ackMsg.drive.speed = 0.5

        if self.state == self.STRAIGHT:
            self.ackMsg.drive.steering_angle = 0
            print("Fahre geradeaus")
        elif self.state == self.DECREASE:
            self.ackMsg.drive.steering_angle = 0.2
            print("Verringere den Abstand")
        elif self.state == self.INCREASE:
            self.ackMsg.drive.steering_angle = -0.2
            print("Vergroessere den Abstand")
        else:
            self.ackMsg.drive.steering_angle = 0
            self.ackMsg.drive.speed = 0
            print("Fehlerhafter Positioningsstate")

        self.ackermann_pub.publish(self.ackMsg)


    def parksuche(self):

        self.time = rospy.Time.now()
        self.time_float = self.time.to_sec()

        if self.time_float > 1:

            if self.derivative < -100 and self.time_save_start_float ==0:
                self.time_save_start = rospy.Time.now()
                self.time_save_start_float = self.time_save_start.to_sec()
                print("Parklueckenanfang gefunden")
            elif self.derivative > 100 and self.time_save_start_float !=0:
                self.time_save_end = rospy.Time.now()
                self.time_save_end_float = self.time_save_end.to_sec()
                print("Parklueckenende gefunden")
            elif self.time_save_start_float != 0 and self.time_save_end_float !=0 and self.delta_time_save == 0:
                self.delta_time_save = (self.time_save_end_float - self.time_save_start_float)
                print("Benoetigte Zeit: ",self.delta_time_save)
            elif self.delta_time_save !=0:
                self.parkplatz_rechner()
            elif self.time_save_start_float !=0:
                print("Warte auf Parklueckenende")
                self.time_safe = rospy.Time.now()
                self.time_safe_float = self.time_safe.to_sec()
                self.such_reset = self.time_safe_float - self.time_save_start_float
                if self.such_reset > 5:
                    self.time_save_start = 0
                    print("kein Ende gefunden - Zeitueberschreitung")
                    self.time_save_start_float = 0
                    self.time_save_start = 0
                    self.time_save_end_float = 0
                    self.time_save_end = 0
                else:
                    pass
            else:
                print("Suche nach moeglichen Parkluecken")
        else:
            pass


    def parkplatz_rechner(self): #Rechnung für 0.5 m/s
        self.delta_time_save = self.delta_time_save

        if self.delta_time_save > 1.4: #v=d/t d=v*t t=d/v 70cm
            if self.delta_time_save < 2: #100cm
                print("!!!Parallelparking erkannt!!!")
                self.parkspace.length = (0.35 * self.delta_time_save) * 100
                print("Parklueckenlaenge: ",self.parkspace.length)
                self.parkingspace_measured = 1
                self.parkluecke_positiv()
            else:
                print("!!!Fehlerhafte Parkluecke erkannt!!! - Parallelparking erwartet")
                self.time_save_start_float = 0
                self.time_save_end_float = 0
                self.delta_time_save = 0
        elif self.delta_time_save > 0.15: #40cm
            if self.delta_time_save < 0.8: #60cm
                print("!!!Crossparking erkannt!!!")
                self.parkspace.length = (0.35 * self.delta_time_save) * 100
                print("Parklueckenlaenge: ",self.parkspace.length)
                self.parkingspace_measured = 2
                self.parkluecke_positiv()
            else:
                print("!!!Fehlerhafte Parkluecke erkannt!!! - Crossparking erwartet")
                self.time_save_start_float = 0
                self.time_save_end_float = 0
                self.delta_time_save = 0
        else:
            print("Das war keine Parkluecke")
            self.time_save_start_float = 0
            self.time_save_end_float = 0
            self.delta_time_save = 0



    def parkluecke_positiv(self):
        
        if self.parkingspace_measured == self.parkspace.type:
            print("Measured stimmt mit Vorhersage überein - Startposition wird angefahren")
            self.parkspace.detected = True

            msg = Bool()
            msg.data = True
            self.space_detected_pub.publish(msg)
        else:
            print("Measured stimmt NICHT mit Vorhersage überein")
            self.time_save_start_float = 0
            self.time_save_end_float = 0
            self.delta_time_save = 0


    def go_to_startposition(self):

        self.r = rospy.Rate(30)

        print("Fahre Startposition an") #85cm vorfahren t ~ 1.7s

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

        print("Startposition erreicht")

        self.parkspace.ready=True


    def parking_parallel(self):

        print("Parke parallel ein")
        self.r2 = rospy.Rate(30)

        for index, (topic, msg, t) in enumerate(rosbag.Bag(self.path_parallel).read_messages(topics=["/autonomous/ackermann_cmd"])):
            if index == 18:
                for _ in range(20):
                    self.ackMsg.drive.steering_angle = 0
                    self.ackMsg.drive.speed = -self.PARKING_SPEED
                    self.ackermann_pub.publish(self.ackMsg)
                    self.r2.sleep()

            elif index > 20 and index < 163:
                if msg.drive.steering_angle == 0:
                    msg.drive.steering_angle = self.STEERING_OFFSET
                msg.drive.speed = -self.PARKING_SPEED + 0.1
                self.ackermann_pub.publish(msg)
                self.r2.sleep()
        #Alternativ: Trajektorie

        rospy.sleep(10) # sleep 10 seconds

        print("Parke in 10 Sekunden aus")

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

        # for index, (topic, msg, t) in enumerate(rosbag.Bag(self.path_ausparken_parallel).read_messages(topics=["/autonomous/ackermann_cmd"])):
        #     if index > 30:
        #         msg.drive.speed = self.PARKING_SPEED
        #         self.ackermann_pub.publish(msg)
        #         self.r2.sleep()
        
        for _ in range(60):
            self.r.sleep()

        print("Erfolgreiches Parkmaneuvre")
        
        self.set_variables()
        self.parkspace.set_variables()



    def parking_cross(self):

        print("Parke cross ein")
        self.r2 = rospy.Rate(30)




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
                
        #Alternativ: Trajektorie
        
        rospy.sleep(10)
        
        print("Parke in 10 Sekunden aus")

        rospy.sleep(10)

        for index, (topic, msg, t) in enumerate(rosbag.Bag(self.path_ausparken_cross).read_messages(topics=["/autonomous/ackermann_cmd"])):
            msg.drive.speed = self.PARKING_SPEED
            self.ackermann_pub.publish(msg)
            self.r2.sleep()
        
        for _ in range(60):
            self.r.sleep()

        self.set_variables()
        self.parkspace.set_variables()


if __name__ == "__main__":
    rospy.init_node("parking", anonymous = False)

    parking = Parker()
    parkingspace = ParkingSpace()

    rospy.spin()
