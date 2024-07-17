#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
import tf
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
import support_files_car_general as sfc_g
from qpsolvers import *
#from std msgs.msg import Float32
#from std msgs.msg import Float32MultiArray

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

        #Define MPC constants
        self.support=sfc_g.SupportFilesCar()
        self.constants=support.constants
        self.Ts=constants['Ts']
        self.outputs=constants['outputs'] # number of outputs (psi, Y)
        self.hz = constants['hz'] # horizon prediction period
        self.time_length=constants['time_length'] # duration of the manoeuvre irrelva
        self.inputs=constants['inputs'] # zahl
        self.x_lim=constants['x_lim']
        self.y_lim=constants['y_lim']
        self.trajectory=constants['trajectory'] # art der traj
        self.x_dot_dot=0.
        self.y_dot_dot=0.
        self.psi_dot_dot=0.
        self.a=0

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
            euler = tf.transformations.euler_from_quaternion(quaternion)

            # z-axis rotation in radians
            z_rotation = euler[2]
            z_rotations.append(z_rotation)
        
        return x_coords, y_coords, z_rotations

    def callback(self, msg):

        X_ref, Y_ref, psi_ref = self.get_trajectory(msg)
        x_dot_ref = np.ones_like(X_ref)
        y_dot_ref = np.zeros_like(X_ref)
        #############
        # DEIN CODE #
        #############
        refSignals=np.zeros(len(X_ref)*outputs)
        k=0
        for i in range(0,len(refSignals),outputs):
            refSignals[i]=x_dot_ref[k]
            refSignals[i+1]=psi_ref[k]
            refSignals[i+2]=X_ref[k]
            refSignals[i+3]=Y_ref[k]
            k=k+1
        
        if a == 0: 
            x_dot=x_dot_ref[0]
            y_dot=y_dot_ref[0]
            psi=psi_ref[0]
            psi_dot=0.
            X=0.  #X_ref[0]
            Y=0.    #Y_ref[0]

        states=np.array([x_dot,y_dot,psi,psi_dot,X,Y])
        statesTotal=np.zeros((len(t),len(states))) # It will keep track of all your states during the entire manoeuvre
        statesTotal[0][0:len(states)]=states



        accelerations=np.array([x_dot_dot,y_dot_dot,psi_dot_dot])
        accelerations_total=np.zeros((len(t),len(accelerations)))

          # Load the initial Inputs
        U1=0 # Input at t = -0.02 s (steering wheel angle in rad (delta))
        U2=0 # Input at t = -0.02 s (acceleration in m/s^2 (a))
        UTotal=np.zeros((len(t),2)) # To keep track all inputs over time
        UTotal[0][0]=U1
        UTotal[0][1]=U2
          # Generate the discrete state space matrices
        Ad,Bd,Cd,Dd=support.state_space(states,U1,U2)

          # Generate the augmented current state and the reference vector
        x_aug_t=np.transpose([np.concatenate((states,[U1,U2]),axis=0)])
        r = refSignals

        Hdb,Fdbt,Cdb,Adc,G,ht=support.mpc_simplification(Ad,Bd,Cd,Dd,hz,x_aug_t,du)
        ft=np.matmul(np.concatenate((np.transpose(x_aug_t)[0][0:len(x_aug_t)],r),axis=0),Fdbt)
        
        try:
            du=solve_qp(Hdb,ft,G,ht,solver="cvxopt")
            du=np.transpose([du])
            print(du)
            # exit()
        except ValueError as ve:
            print(Hdb)
            print(ft)
            print(G)
            print(ht)
            print(Adc)
            print(x_aug_t)
            print(du)
            print(i)
            #break

        U1=U1+du[0][0]  #steeringwheel angle  du ist die aenderung
        U2=U2+du[1][0]  # acceleration
        print(U2)
        UTotal[i+1][0]=U1
        UTotal[i+1][1]=U2 
        
        states,x_dot_dot,y_dot_dot,psi_dot_dot=support.open_loop_new_states(states,U1,U2) # evtl nicht mgl
        statesTotal[i+1][0:len(states)]=states #wsl nicht mgl
        accelerations=np.array([x_dot_dot,y_dot_dot,psi_dot_dot])
        accelerations_total[i+1][0:len(accelerations)]=accelerations
        # evtl einfuegen von x punkt, y punkt ueber beschl sensor, sowie psi dot 
        # vlt reicht berechnung aus, mal sehen, muss beratschlagen 
        
        
        
        
        
        # Nachricht an VESC senden
        steering_angle = U1  #psi_ref[1] / 4 # platzhalter
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
