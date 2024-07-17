#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
import transformations
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
import rospkg
import sys
import os
from qpsolvers import *
import time
##!!!! IDEE mit 8 zu skalieren morgen fortsetzen !!!!
# The following line of code dynamically modifies the Python search path for modules at runtime by appending a specific directory.
# It uses the `rospkg` module, which provides an interface to ROS package locations, to find the directory of the 'mpc' package.
# Specifically, it appends the 'include' directory within the 'mpc' package to the `sys.path` list. This is necessary because the
# 'include' directory is where some Python modules or packages are located, but it's not a standard location that Python automatically
# searches for modules. By appending this directory to `sys.path`, it allows Python scripts to import modules from this non-standard location.
# In this case, after modifying `sys.path`, the script imports the `SupportFilesCar` module from the 'include' directory of the 'mpc' package.
sys.path.append(os.path.join(rospkg.RosPack().get_path('mpc'), 'include'))
from support_files_car_general import SupportFilesCar
#roslaunch mpc run_mpc.launch

# Node that subscribes to /path topic
class PathSubscriberNode(object):
    def __init__(self): 
        rospy.init_node('path_subscriber_node')


        # Subscribe to the /path topic
        self.path_sub = rospy.Subscriber('/path', Path, self.callback)

        # publish ackermann messages to VESC
        self.ackermann_pub = rospy.Publisher('/autonomous/ackermann_cmd', AckermannDriveStamped, queue_size=1) 

        # define messages
        self.ackMsg = AckermannDriveStamped()
        
        # subscribe acc
        self.object_detect_sub = rospy.Subscriber('/detected_relevant_object', bool, self.callback)
        self.distance_sub = ropy.Subscriber('/distance_to_vehicle', Float32, self.callback)
        self.relative_speed_sub = rospy.Subscriber("/relative_speed_to_vehicle", Float32, self.callback)
        #self.parking_sign = JOSEFSUBSCRIBER
        #self.traffic_light = JOSEFSUBSRIBER
        
        #Define MPC constants
        self.support=SupportFilesCar()
        self.constants=self.support.constants
        self.Ts=self.constants['Ts']
        self.outputs=self.constants['outputs'] # number of outputs (psi, Y)
        self.hz = self.constants['hz'] # horizon prediction period
        self.time_length=self.constants['time_length'] # duration of the manoeuvre irrelva
        self.inputs=self.constants['inputs'] # zahl
        self.x_lim=self.constants['x_lim']
        self.y_lim=self.constants['y_lim']
        self.trajectory=self.constants['trajectory'] # art der traj
        self.x_dot_dot=0.
        self.y_dot_dot=0.
        self.psi_dot_dot=0.
        self.x_dot=1.#*8 #x_dot_ref[0]
        self.y_dot=0. #y_dot_ref[0]
        self.psi=0.#psi_ref[0]
        self.psi_dot=0.
        self.X=0.  #X_ref[0]
        self.Y=0.    #Y_ref[0]
        self.x_dot_dot=0.
        self.y_dot_dot=0.
        self.psi_dot_dot=0.
        self.a=0
        self.states=np.array([self.x_dot,self.y_dot,self.psi,self.psi_dot,self.X,self.Y])
        self.U1=0 # Input at t = -0.02 s (steering wheel angle in rad (delta))
        self.U2=0 # Input at t = -0.02 s (acceleration in m/s^2 (a))
        self.acc_enable = True


    def get_trajectory(self, msg):
        x_coords, y_coords, z_rotations = [], [], []

        for pose_stamped in msg.poses[1:]: # skip the first one since its the base
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
        
        return x_coords, y_coords, z_rotations

    def callback(self, msg):self.traffic_light

        start = time.time()
        X_ref, Y_ref, psi_ref = self.get_trajectory(msg)
        print("get_trajectory: %.2f" % (time.time()-start))
        
        #X_ref = [x * 8 for x in X_ref]
        #Y_ref = [y * 8 for y in Y_ref]
        #psi_ref = [psi * 8 for psi in psi_ref]

	# Vector of x and y changes per sample time
        #dX=X_ref[1:len(X_ref)]-X_ref[0:len(X_ref)-1]
        #dY=Y_ref[1:len(Y_ref)]-Y_ref[0:len(Y_ref)-1]
        # Convert lists to NumPy arrays
        X_ref_array = np.array(X_ref)
        Y_ref_array = np.array(Y_ref)

          # Calculate differences
        dX = X_ref_array[1:] - X_ref_array[:-1]
        dY = Y_ref_array[1:] - Y_ref_array[:-1]
                # Define the reference yaw angles
        psi=np.zeros(len(X_ref))
        psiInt=psi
        psi[0]=np.arctan2(dY[0],dX[0])
        psi[1:len(psi)]=np.arctan2(dY[0:len(dY)],dX[0:len(dX)])

        # We want the yaw angle to keep track the amount of rotations
        dpsi=psi[1:len(psi)]-psi[0:len(psi)-1]
        psiInt[0]=psi[0]
        for i in range(1,len(psiInt)):
            if dpsi[i-1]<-np.pi:
                psiInt[i]=psiInt[i-1]+(dpsi[i-1]+2*np.pi)
            elif dpsi[i-1]>np.pi:
                psiInt[i]=psiInt[i-1]+(dpsi[i-1]-2*np.pi)
            else:
                psiInt[i]=psiInt[i-1]+dpsi[i-1]
        
        x_dot_ref = np.ones_like(X_ref)
        x_dot_ref = x_dot_ref# * 8
        y_dot_ref = np.zeros_like(X_ref)
        
        refSignals=np.zeros(len(X_ref)*self.outputs)
        k=0
        
        for i in range(0,len(refSignals),self.outputs):
            refSignals[i]=x_dot_ref[k]
            refSignals[i+1]=psiInt[k]
            refSignals[i+2]=X_ref[k]
            refSignals[i+3]=Y_ref[k]
            k=k+1
        print("Ref_Signals",refSignals)
        #if self.a == 0:
            #self.states=np.array([self.x_dot,self.y_dot,self.psi,self.psi_dot,self.X,self.Y])########states muss noch initialisiert werden
            #self.a=1
        print("states aktuell",self.states)
        #statesTotal=np.zeros((len(t),len(states))) # It will keep track of all your states during the entire manoeuvre
        #statesTotal[0][0:len(states)]=states



        accelerations=np.array([self.x_dot_dot,self.y_dot_dot,self.psi_dot_dot])
        #accelerations_total=np.zeros((len(t),len(accelerations)))
        du=np.zeros((self.inputs*self.hz,1)) # zwei lösungen
          # Load the initial Inputs
        #U1=0 # Input at t = -0.02 s (steering wheel angle in rad (delta))
        #U2=0 # Input at t = -0.02 s (acceleration in m/s^2 (a))
        #UTotal=np.zeros((len(t),2)) # To keep track all inputs over time
        #UTotal[0][0]=U1
        #UTotal[0][1]=U2
          # Generate the discrete state space matrices
        Ad,Bd,Cd,Dd=self.support.state_space(self.states,self.U1,self.U2)

          # Generate the augmented current state and the reference vector
        x_aug_t=np.transpose([np.concatenate((self.states,[self.U1,self.U2]),axis=0)])
        states_predicted = np.zeros_like(x_aug_t)
        r = refSignals

        Hdb,Fdbt,Cdb,Adc,G,ht,states_predicted=self.support.mpc_simplification(Ad,Bd,Cd,Dd,self.hz,x_aug_t,du)
        print("Pred states",states_predicted)
        ft=np.matmul(np.concatenate((np.transpose(x_aug_t)[0][0:len(x_aug_t)],r),axis=0),Fdbt)
        
        start = time.time()
        try:
            du=solve_qp(Hdb,ft,G,ht,solver="clarabel") #clarabel cvxopt highs
            du=np.transpose([du])
            print("du",du)
            # exit()
        except ValueError as ve:
            print("Test",Hdb)
            print(ft)
            print(G)
            print(ht)
            print(Adc)
            print(x_aug_t)
            print(du)
            print(i)
            #break
        print("solve_qp: %.2f" % (time.time()-start))

        self.U1=self.U1+du[0][0]  #steeringwheel angle  du ist die aenderung
        self.U2=self.U2+du[1][0]  # acceleration
        print("Lenkwinkel:", self.U1)
        print("Beschl",self.U2)
        #UTotal[i+1][0]=U1
        #UTotal[i+1][1]=U2 
        
        self.states,x_dot_dot,y_dot_dot,psi_dot_dot=self.support.open_loop_new_states(self.states,self.U1,self.U2) # evtl nicht mgl
        #statesTotal[i+1][0:len(self.states)]=states #wsl nicht mgl
        accelerations=np.array([x_dot_dot,y_dot_dot,psi_dot_dot])
        #accelerations_total[i+1][0:len(accelerations)]=accelerations
        # evtl einfuegen von x punkt, y punkt ueber beschl sensor, sowie psi dot 
        # vlt reicht berechnung aus, mal sehen, muss beratschlagen 
        print("States",self.states)
        
        
        #ACC Integration
        
        #if self.parking_sign == True
        #	self.acc_enable = False
        #if self.traffic_light == True
        #	self.acc_enable = True
        
        if self.acc_enable == True
        	if self.object_detect_sub == True
        		d = 2
        		delta_distance = self.distance_sub - d 
        		if delta_distance >= 0.2:  # Wenn zu weit weg
        			speed = #U2 Integriert
    			elif delta_distance <= -0.2:  # Wenn zu nah
        			speed = U2 Itegriert halbe
    			else:  # In einem Fenster von 0.2 bis -0.2
        			speed = U2 integriert - self.relative_speed_sub

        
        
        # Nachricht an VESC senden
        steering_angle = self.U1  #psi_ref[1] / 4 # platzhalter
        speed = 1.0 # die sinus kurve wurde mit 1 m/s generiert


        self.ackMsg.header.stamp = rospy.Time.now()
        self.ackMsg.drive.steering_angle = steering_angle
        self.ackMsg.drive.speed = speed

        self.ackermann_pub.publish(self.ackMsg)

    def spin(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        node = PathSubscriberNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
