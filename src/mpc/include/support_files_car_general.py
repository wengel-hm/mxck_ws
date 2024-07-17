#!/usr/bin/env python3

import numpy as np
#import matplotlib.pyplot as plt

class SupportFilesCar:
    ''' The following functions interact with the main file'''
    def __init__(self):
        ''' Load the constants that do not change'''
        g=9.81
        m=6.161 #Masse faktor 0,0041
        Iz=0.2 #Moment of inertia
        Cf=8 #Front tire cornering stiffness (N/rad).
        Cr=10 #Rear tire cornering stiffness (N/rad).
        lf=0.195#Distance from the center of gravity to the front axle (m).
        lr=0.165#Distance from the center of gravity to the rear axle (m).
        Ts=0.08 #fuer 1m/s    #0.16  #Sampling time for the MPC controller (s). wichtig fuer die abstaende der Punkte bei nicht const geschw evtl mit formel
        mju=0.1  # equivalent to asphalt friction coefficient
        ####################### Lateral control #################################

        outputs=4 # number of outputs
        inputs=2 # number of inputs
        hz = 8  #10 # horizon period, can be changed

        trajectory=3 # Choose 1, 2 or 3, nothing else
        version=2 # This is only for trajectory 3 (Choose 1 or 2)

        if trajectory==3 and version==2:
            # Weights for trajectory 3, version 2
            Q=np.matrix('10 0 0 0;0 2000 0 0;0 0 10 0;0 0 0 10') # weights for outputs (all samples, except the last one)
            S=np.matrix('10 0 0 0;0 2000 0 0;0 0 10 0;0 0 0 10') # weights for the final horizon period outputs
            R=np.matrix('10 0;0 1') # weights for inputs
      #By specifying weights in Q, you are specifying how much importance or priority each state
      #variable has in the cost function. In your example, Q is a diagonal matrix, indicating that each
      #state variable is weighted independently of the others. For example, the weight for the first
      #state variable is 100, the weight for the second state variable is 20000, and so on. A higher
      #weight indicates that the controller should prioritize minimizing the deviation of that state
      #variable from its reference or desired value.
        elif trajectory==3:
            # Weights for trajectory 3, version 1
            Q=np.matrix('10000 0 0 0;0 2000000 0 0;0 0 100000 0;0 0 0 100000') # weights for outputs (all samples, except the last one)
            S=np.matrix('10000 0 0 0;0 2000000 0 0;0 0 100000 0;0 0 0 100000') # weights for the final horizon period outputs
            R=np.matrix('10000 0;0 100') # weights for inputs
        else:
            # Weights for trajectories 1 & 2
            Q=np.matrix('10 0 0 0;0 200 0 0;0 0 10 0;0 0 0 10') # weights for outputs (all samples, except the last one)
            S=np.matrix('100 0 0 0;0 200 0 0;0 0 10 0;0 0 0 10') # weights for the final horizon period outputs
            R=np.matrix('10 0;0 0.1') # weights for inputs
        # Please do not modify the time_length!
        delay=0
        if trajectory==1:
           time_length = 60.
           x_lim=1000
           y_lim=1000
        elif trajectory ==2:
           time_length = 140.
           x_lim=1000
           y_lim=1000
        elif trajectory == 3:
           if version==1:
                x_lim=170
                y_lim=160
           else:
                x_lim=170*version
                y_lim=160*version
           first_section=14
           other_sections=14
           time_length=first_section+other_sections*10
           delay=np.zeros(12)
           for dly in range(1,len(delay)):
               delay[dly]=first_section+(dly-1)*other_sections
           # print(delay)
           # exit()
        else:
           print("trajectory: 1,2 or 3; version: 1 or 2")

        self.constants={'g':g,'m':m,'Iz':Iz,'Cf':Cf,'Cr':Cr,'lf':lf,'lr':lr,\
        'Ts':Ts,'mju':mju,'Q':Q,'S':S,'R':R,'outputs':outputs,'inputs':inputs,\
        'hz':hz,'delay':delay,'time_length':time_length,'trajectory':trajectory,\
        'version':version,'x_lim':x_lim,'y_lim':y_lim}

        

    def trajectory_generator(self, msg):
        '''This method creates the trajectory for a car to follow'''
        # Arrays zur Speicherung der x- und y-Koordinaten der Wegpunkte
        x_ref = []
        y_ref = []
        # Schleife durch alle Wegpunkte im empfangenen Pfad
        for pose in msg.poses:
        # x- und y-Koordinaten des aktuellen Wegpunkts in die Arrays speichern
            x_ref.append(pose.pose.position.x)
            y_ref.append(pose.pose.position.y)

            # Ausgabe der Koordinaten jedes Wegpunkts
        print("Waypoint: x={}, y={}".format(pose.pose.position.x, pose.pose.position.y))
        #Maxgeschw basierend auf kreummung
        # Ableitungen mit finiten Differenzen approximieren
        dx = np.gradient(x_ref)
        dy = np.gradient(y_ref)
        ddx = np.gradient(dx)
        ddy = np.gradient(dy)
        radius = [abs(num) for num in radius]
        radius = np.maximum(radius, 0.1)
        # Begrenze den Radius auf den maximalen zulaessigen Wert
        radius = np.minimum(radius, 2.5) #geschw begrenzung
        # Kruemmungsradius berechnen
        radius = (dx * ddy - ddx * dy) / (dx**2 + dy**2)**(3/2)
        max_acceleration = 3
        max_acceleration_reshaped = np.full_like(radius, max_acceleration)

        # Perform element-wise multiplication and take square root
        x_punkt = np.sqrt(max_acceleration_reshaped * radius)
        print("Maximale zulaessige Geschwindigkeit:", x_punkt, "m/s")

        # Vector of x and y changes per sample time
        dX=x_ref[1:len(x_ref)]-x_ref[0:len(x_ref)-1]
        dY=y_ref[1:len(y_ref)]-y_ref[0:len(y_ref)-1]

        #X_dot=dX/Ts Muss nicht immer so berechnet werden
        #Y_dot=dY/Ts
        #X_dot=np.concatenate(([X_dot[0]],X_dot),axis=0)
        #Y_dot=np.concatenate(([Y_dot[0]],Y_dot),axis=0)
        #x_punkt = [1]* 5
        y_punkt = [0]* 5

        # Define the reference yaw angles
        psi=np.zeros(len(x_ref))
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

       # Rckgabe der Arrays mit den Wegpunkt-Koordinaten
        return x_punkt, y_punkt, psiInt, x_ref, y_ref
    
    def state_space(self,states,delta,a):
        '''This function forms the state space matrices and transforms them in the discrete form'''

        # Get the necessary constants
        g=self.constants['g']
        m=self.constants['m']
        Iz=self.constants['Iz']
        Cf=self.constants['Cf']
        Cr=self.constants['Cr']
        lf=self.constants['lf']
        lr=self.constants['lr']
        Ts=self.constants['Ts']
        mju=self.constants['mju']

        # Get the necessary states
        x_dot=states[0]
        y_dot=states[1]
        psi=states[2]

        # Get the state space matrices for the control
        A11=-mju*g/x_dot #Beitrag der Geschwindigkeit x zur aenderung der Laengsbeschleunigung des Fahrzeugs
        A12=Cf*np.sin(delta)/(m*x_dot) #Beitrag des Lenkwinkels δ zur aenderung der Laengsbeschleunigung des Fahrzeugs
        A14=Cf*lf*np.sin(delta)/(m*x_dot)+y_dot #Beitrag des seitlichen Verschiebungsgeschwindigkeitsfehlers y zur aenderung der Laengsbeschleunigung des Fahrzeugs.
        A22=-(Cr+Cf*np.cos(delta))/(m*x_dot) #Beitrag des Lenkwinkels δ zur aenderung der seitlichen Beschleunigung des Fahrzeugs
        A24=-(Cf*lf*np.cos(delta)-Cr*lr)/(m*x_dot)-x_dot #Beitrag des seitlichen Verschiebungsgeschwindigkeitsfehlers y zur aenderung der seitlichen Beschleunigung des Fahrzeugs
        A34=1 #Gierwinkelgeschwindigkeit r zur aenderung der seitlichen Verschiebungsgeschwindigkeit des Fahrzeugs
        A42=-(Cf*lf*np.cos(delta)-lr*Cr)/(Iz*x_dot) #Beitrag des Lenkwinkels δ zur aenderung der Gierwinkelgeschwindigkeit des Fahrzeugs.
        A44=-(Cf*lf**2*np.cos(delta)+lr**2*Cr)/(Iz*x_dot) #Beitrag des Lenkwinkels δ zur aenderung der Gierwinkelbeschleunigung des Fahrzeugs
        A51=np.cos(psi) #Beziehung zwischen der aktuellen Fahrzeugorientierung (gemessen durch den Kurswinkel ψ) und der aenderung der Laengs- und Quergeschwindigkeiten des Fahrzeugs
        A52=-np.sin(psi)
        A61=np.sin(psi) #Beitrag des Sinus des Gierwinkels (ψ) zur aenderung der Laengsgeschwindigkeit des Fahrzeugs
        A62=np.cos(psi)#Beitrag des Kosinus des Gierwinkels (ψ) zur aenderung der Laengsgeschwindigkeit des Fahrzeugs. 

        B11=-1/m*np.sin(delta)*Cf #laterale Beschleunigung des Fahrzeugs abnimmt, wenn der Lenkwinkel zunimmt
        B12=1 #die aenderung der lateralen Geschwindigkeit des Fahrzeugs direkt von sich selbst abhaengt
        B21=1/m*np.cos(delta)*Cf # longitudinale Beschleunigung zunimmt, wenn der Lenkwinkel zunimmt
        B41=1/Iz*np.cos(delta)*Cf*lf #bedeutet, dass die Gierwinkelgeschwindigkeit zunimmt, wenn der Lenkwinkel zunimmt

        ### Zustandsraumdarstellung ###
        A=np.array([[A11, A12, 0, A14, 0, 0],[0, A22, 0, A24, 0, 0],[0, 0, 0, A34, 0, 0],\
        [0, A42, 0, A44, 0, 0],[A51, A52, 0, 0, 0, 0],[A61, A62, 0, 0, 0, 0]]) ##Beitrag einer Zustandsgroeße zur zeitlichen aenderung 6x6 bedeutet 6 Zustandsgroeßen
        B=np.array([[B11, B12],[B21, 0],[0, 0],[B41, 0],[0, 0],[0, 0]]) #Eingangsmatrix und beschreibt, wie die Eingangsgroeßen das System beeinflussen 6x2 bedeutet 2 eingangsgroeßen
        C=np.array([[1, 0, 0, 0, 0, 0],[0, 0, 1, 0, 0, 0],[0, 0, 0, 0, 1, 0],[0, 0, 0, 0, 0, 1]]) #Ausgangsmatrix: beschreibt, wie die Zustandsvariablen des Systems in die Ausgangsgroeßen      uebersetzt werden
        D=np.array([[0, 0],[0, 0],[0, 0],[0, 0]])#Direktuebertragungsmatrix und beschreibt den direkten Einfluss der Eingangsvariablen auf die Ausgangsvariablen

        # Discretise the system (forward Euler)
        Ad=np.identity(np.size(A,1))+Ts*A ## diskrete Zustandsuebergangsmatrix von zeit auf weg
        Bd=Ts*B # diskrete Eingangsmatrix 
        Cd=C #unveraendert, da die Ausgangsgroeßen nicht diskretisiert
        Dd=D #eh 0 

        return Ad, Bd, Cd, Dd
    
    ##Umwandlung in erweiterte Zustandsraummatrizen fuer die Zustandsraumregelung###
    def augmented_matrices(self, Ad, Bd, Cd, Dd):

        A_aug=np.concatenate((Ad,Bd),axis=1)# aneinanderhaengen der spalten von Ad und Bd 
        temp1=np.zeros((np.size(Bd,1),np.size(Ad,1)))
        temp2=np.identity(np.size(Bd,1))#erzeugen einheitsmatrix
        temp=np.concatenate((temp1,temp2),axis=1)

        A_aug=np.concatenate((A_aug,temp),axis=0)#zusammenfuegen von A und B mit einer einheitsmatrix 
        B_aug=np.concatenate((Bd,np.identity(np.size(Bd,1))),axis=0)# bd mit einer einehitsmatrix concat
        C_aug=np.concatenate((Cd,np.zeros((np.size(Cd,0),np.size(Bd,1)))),axis=1)#matrix mit der groeße von nullen angehangen 
        D_aug=Dd

        return A_aug, B_aug, C_aug, D_aug

    def mpc_simplification(self, Ad, Bd, Cd, Dd, hz, x_aug_t, du):
        '''This function creates the compact matrices for Model Predictive Control'''
        # db - double bar
        # dbt - double bar transpose
        # dc - double circumflex

        A_aug, B_aug, C_aug, D_aug=self.augmented_matrices(Ad, Bd, Cd, Dd)

        Q=self.constants['Q']
        S=self.constants['S']
        R=self.constants['R']
        Cf=self.constants['Cf']
        g=self.constants['g']
        m=self.constants['m']
        mju=self.constants['mju']
        lf=self.constants['lf']
        inputs=self.constants['inputs']

        ############################### Constraints ############################# 
        d_delta_max=np.pi/150  #150  #200 maximaler aenderungswinkel #300
        d_a_max=0.5#0.5            #200 max beschl, evtl anpassen	0.1 
        d_delta_min=-np.pi/150 #150	#300
        d_a_min=-0.5#-0.5		#-0.1

        ub_global=np.zeros(inputs*hz)#legen grenzen fuer steuereingang fest
        lb_global=np.zeros(inputs*hz)

        # Only works for 2 inputs zuoerdnung der constraints
        for i in range(0,inputs*hz):
            if i%2==0:
                ub_global[i]=d_delta_max
                lb_global[i]=-d_delta_min
            else:
                ub_global[i]=d_a_max
                lb_global[i]=-d_a_min

        ub_global=ub_global[0:inputs*hz]
        lb_global=lb_global[0:inputs*hz]
        ublb_global=np.concatenate((ub_global,lb_global),axis=0) #zusammenfueren beider matrizen

        I_global=np.eye(inputs*hz)
        I_global_negative=-I_global
        I_mega_global=np.concatenate((I_global,I_global_negative),axis=0)# wird verwendet um grenzen der steuereingaenge zu formulieren

        y_asterisk_max_global=[]
        y_asterisk_min_global=[]

        C_asterisk=np.matrix('1 0 0 0 0 0 0 0;\
                        0 1 0 0 0 0 0 0;\
                        0 0 0 0 0 0 1 0;\
                        0 0 0 0 0 0 0 1')

        C_asterisk_global=np.zeros((np.size(C_asterisk,0)*hz,np.size(C_asterisk,1)*hz))

        #########################################################################

        CQC=np.matmul(np.transpose(C_aug),Q)
        CQC=np.matmul(CQC,C_aug)#costmatrix mit der grenzmatrix kombinieren, hilft dem MPC-Algorithmus, die Zustaende in Bezug auf die costmatrix zu beruecksichtigen

        CSC=np.matmul(np.transpose(C_aug),S)
        CSC=np.matmul(CSC,C_aug)#steuerungen in Bezug auf die costmarix zu beruecksichtigen

        QC=np.matmul(Q,C_aug)
        SC=np.matmul(S,C_aug)# Diese Matrizen repraesentieren die Kosten, die durch die Wechselwirkung zwischen Zustaenden und Steuerungen entstehen

        Qdb=np.zeros((np.size(CQC,0)*hz,np.size(CQC,1)*hz))
        Tdb=np.zeros((np.size(QC,0)*hz,np.size(QC,1)*hz))
        Rdb=np.zeros((np.size(R,0)*hz,np.size(R,1)*hz))
        Cdb=np.zeros((np.size(B_aug,0)*hz,np.size(B_aug,1)*hz))
        Adc=np.zeros((np.size(A_aug,0)*hz,np.size(A_aug,1))) #alles nullmatrizen fuer spaeteren gebrauch

        ######################### Advanced LPV ##################################
        A_product=A_aug
        states_predicted_aug=x_aug_t #werden initialisiert, um die aktuelle Systemdynamik und den aktuellen erweiterten Zustand zu speichern
        A_aug_collection=np.zeros((hz,np.size(A_aug,0),np.size(A_aug,1)))#Sammlungen von Matrizen, die für die Aufzeichnung der Systemdynamik in verschiedenen Zeitschritten verwendet werden
        B_aug_collection=np.zeros((hz,np.size(B_aug,0),np.size(B_aug,1)))
        #########################################################################

        for i in range(0,hz):
            if i == hz-1:
                Qdb[np.size(CSC,0)*i:np.size(CSC,0)*i+CSC.shape[0],np.size(CSC,1)*i:np.size(CSC,1)*i+CSC.shape[1]]=CSC #qudratische Kostenfunktion
                Tdb[np.size(SC,0)*i:np.size(SC,0)*i+SC.shape[0],np.size(SC,1)*i:np.size(SC,1)*i+SC.shape[1]]=SC #Mischkostenfunktion zw zustaenden und steuerungen
            else:
                Qdb[np.size(CQC,0)*i:np.size(CQC,0)*i+CQC.shape[0],np.size(CQC,1)*i:np.size(CQC,1)*i+CQC.shape[1]]=CQC
                Tdb[np.size(QC,0)*i:np.size(QC,0)*i+QC.shape[0],np.size(QC,1)*i:np.size(QC,1)*i+QC.shape[1]]=QC

            Rdb[np.size(R,0)*i:np.size(R,0)*i+R.shape[0],np.size(R,1)*i:np.size(R,1)*i+R.shape[1]]=R#Kostenfunktion Steuerung

            ########################### Advanced LPV ############################
            Adc[np.size(A_aug,0)*i:np.size(A_aug,0)*i+A_aug.shape[0],0:0+A_aug.shape[1]]=A_product# dynamische entwicklung der Zustandsmartix
            A_aug_collection[i][:][:]=A_aug # speichern der Werte
            B_aug_collection[i][:][:]=B_aug
            #####################################################################

            ######################## Constraints ################################ max lenkwinkel und geschw
            x_dot_max=10
            if 0.17*states_predicted_aug[0][0] < 3:
                y_dot_max=0.17*states_predicted_aug[0][0]
            else:
                y_dot_max=5
            delta_max=0.42  #0.42 #np.pi/6
            Fyf=Cf*(states_predicted_aug[6][0]-states_predicted_aug[1][0]/states_predicted_aug[0][0]-lf*states_predicted_aug[3][0]/states_predicted_aug[0][0])
            a_max=1+(Fyf*np.sin(states_predicted_aug[6][0])+mju*m*g)/m-states_predicted_aug[3][0]*states_predicted_aug[1][0]
            x_dot_min=0.2
            if -0.17*states_predicted_aug[0][0] > -3:
                y_dot_min=-0.17*states_predicted_aug[0][0]
            else:
                y_dot_min=-5
            delta_min=-0.42 #-np.pi/6
            a_min=-4+(Fyf*np.sin(states_predicted_aug[6][0])+mju*m*g)/m-states_predicted_aug[3][0]*states_predicted_aug[1][0]

            y_asterisk_max=np.array([x_dot_max,y_dot_max,delta_max,a_max])
            y_asterisk_min=np.array([x_dot_min,y_dot_min,delta_min,a_min])

            y_asterisk_max_global=np.concatenate((y_asterisk_max_global,y_asterisk_max),axis=0)
            y_asterisk_min_global=np.concatenate((y_asterisk_min_global,y_asterisk_min),axis=0)

            C_asterisk_global[np.size(C_asterisk,0)*i:np.size(C_asterisk,0)*i+C_asterisk.shape[0],np.size(C_asterisk,1)*i:np.size(C_asterisk,1)*i+C_asterisk.shape[1]]=C_asterisk #aktualisiert, um die Beziehung zwischen den Ausgangsvariablen und den Zustaenden ueber den Vorhersagehorizont hinweg zu beruecksichtigen


            #####################################################################

            ######################### Advanced LPV ############################## vorhersage der Zustände
            if i<hz-1:
                du1=du[inputs*(i+1)][0]
                du2=du[inputs*(i+1)+inputs-1][0]
                states_predicted_aug=np.matmul(A_aug,states_predicted_aug)+np.matmul(B_aug,np.transpose([[du1,du2]]))#Vorhersage der nächsten states
                states_predicted=np.transpose(states_predicted_aug[0:6])[0]
                delta_predicted=states_predicted_aug[6][0]#predicted lw 
                a_predicted=states_predicted_aug[7][0]#predictet beschl
                Ad, Bd, Cd, Dd=self.state_space(states_predicted,delta_predicted,a_predicted)
                A_aug, B_aug, C_aug, D_aug=self.augmented_matrices(Ad, Bd, Cd, Dd)
                A_product=np.matmul(A_aug,A_product)

        for i in range(0,hz):
            for j in range(0,hz):
                if j<=i:
                    AB_product=np.eye(np.shape(A_aug)[0])
                    for ii in range(i,j-1,-1):
                        if ii>j:
                            AB_product=np.matmul(AB_product,A_aug_collection[ii][:][:])
                        else:
                            AB_product=np.matmul(AB_product,B_aug_collection[ii][:][:])
                    Cdb[np.size(B_aug,0)*i:np.size(B_aug,0)*i+B_aug.shape[0],np.size(B_aug,1)*j:np.size(B_aug,1)*j+B_aug.shape[1]]=AB_product#beziehungen zwischen den eingaengen und zustaenden ueber ganzn hz
        
        #########################################################################

        ####################### Constraints #####################################

        Cdb_constraints=np.matmul(C_asterisk_global,Cdb)
        Cdb_constraints_negative=-Cdb_constraints
        Cdb_constraints_global=np.concatenate((Cdb_constraints,Cdb_constraints_negative),axis=0)#constrauntsmatix fuer steuerung

        Adc_constraints=np.matmul(C_asterisk_global,Adc)#const matrix zustandsraum
        Adc_constraints_x0=np.transpose(np.matmul(Adc_constraints,x_aug_t))[0]
        y_max_Adc_difference=y_asterisk_max_global-Adc_constraints_x0
        y_min_Adc_difference=-y_asterisk_min_global+Adc_constraints_x0
        y_Adc_difference_global=np.concatenate((y_max_Adc_difference,y_min_Adc_difference),axis=0)

        G=np.concatenate((I_mega_global,Cdb_constraints_global),axis=0)#zusammenstellung gesamter constraints matrix
        ht=np.concatenate((ublb_global,y_Adc_difference_global),axis=0)#rechte seite constraints leichung

        #######################################################################

        Hdb=np.matmul(np.transpose(Cdb),Qdb)
        Hdb=np.matmul(Hdb,Cdb)+Rdb#Hesse-Matrix des mpc

        temp=np.matmul(np.transpose(Adc),Qdb)
        temp=np.matmul(temp,Cdb)

        temp2=np.matmul(-Tdb,Cdb)
        Fdbt=np.concatenate((temp,temp2),axis=0)#enthaelt Informationen darueber, wie sich die Zustands- und Steuerungsvariablen im Verlauf der Zeit entwickeln, waehrend die Constraints eingehalten werden
        #print(A_product)
        return Hdb,Fdbt,Cdb,Adc,G,ht,states_predicted
    def open_loop_new_states(self,states,delta,a):
        '''This function computes the new state vector for one sample time later'''

        # Get the necessary constants
        g=self.constants['g']
        m=self.constants['m']
        Iz=self.constants['Iz']
        Cf=self.constants['Cf']
        Cr=self.constants['Cr']
        lf=self.constants['lf']
        lr=self.constants['lr']
        Ts=self.constants['Ts']
        mju=self.constants['mju']

        current_states=states
        new_states=current_states
        x_dot=current_states[0]
        y_dot=current_states[1]
        psi=current_states[2]
        psi_dot=current_states[3]
        X=current_states[4]
        Y=current_states[5]

        sub_loop=30  #Chops Ts into 30 pieces
        for i in range(0,sub_loop):

            # Compute lateral forces
            Fyf=Cf*(delta-y_dot/x_dot-lf*psi_dot/x_dot)
            Fyr=Cr*(-y_dot/x_dot+lr*psi_dot/x_dot)

            # Compute the the derivatives of the states
            x_dot_dot=a+(-Fyf*np.sin(delta)-mju*m*g)/m+psi_dot*y_dot
            y_dot_dot=(Fyf*np.cos(delta)+Fyr)/m-psi_dot*x_dot
            psi_dot=psi_dot
            psi_dot_dot=(Fyf*lf*np.cos(delta)-Fyr*lr)/Iz
            X_dot=x_dot*np.cos(psi)-y_dot*np.sin(psi)
            Y_dot=x_dot*np.sin(psi)+y_dot*np.cos(psi)

            # Update the state values with new state derivatives
            x_dot=x_dot+x_dot_dot*Ts/sub_loop
            y_dot=y_dot+y_dot_dot*Ts/sub_loop
            psi=psi+psi_dot*Ts/sub_loop
            psi_dot=psi_dot+psi_dot_dot*Ts/sub_loop
            X=X+X_dot*Ts/sub_loop
            Y=Y+Y_dot*Ts/sub_loop

        # Take the last states
        new_states[0]=x_dot
        new_states[1]=y_dot
        new_states[2]=psi
        new_states[3]=psi_dot
        new_states[4]=0. #X
        new_states[5]=0. #Y
        #print("newstates",new_states)

        return new_states,x_dot_dot,y_dot_dot,psi_dot_dot
        
    def open_loop_new_states_pred(self,states,delta,a):
        '''This function computes the new state vector for one sample time later'''

        # Get the necessary constants
        g=self.constants['g']
        m=self.constants['m']
        Iz=self.constants['Iz']
        Cf=self.constants['Cf']
        Cr=self.constants['Cr']
        lf=self.constants['lf']
        lr=self.constants['lr']
        Ts=self.constants['Ts']
        mju=self.constants['mju']

        current_states=states
        new_states=current_states
        x_dot=current_states[0]
        y_dot=current_states[1]
        psi=current_states[2]
        psi_dot=current_states[3]
        X=current_states[4]
        Y=current_states[5]

        sub_loop=30  #Chops Ts into 30 pieces
        for i in range(0,sub_loop):

            # Compute lateral forces
            Fyf=Cf*(delta-y_dot/x_dot-lf*psi_dot/x_dot)
            Fyr=Cr*(-y_dot/x_dot+lr*psi_dot/x_dot)

            # Compute the the derivatives of the states
            x_dot_dot=a+(-Fyf*np.sin(delta)-mju*m*g)/m+psi_dot*y_dot
            y_dot_dot=(Fyf*np.cos(delta)+Fyr)/m-psi_dot*x_dot
            psi_dot=psi_dot
            psi_dot_dot=(Fyf*lf*np.cos(delta)-Fyr*lr)/Iz
            X_dot=x_dot*np.cos(psi)-y_dot*np.sin(psi)
            Y_dot=x_dot*np.sin(psi)+y_dot*np.cos(psi)


            # Update the state values with new state derivatives
            x_dot=x_dot+x_dot_dot*Ts/sub_loop
            y_dot=y_dot+y_dot_dot*Ts/sub_loop
            psi=psi+psi_dot*Ts/sub_loop
            psi_dot=psi_dot+psi_dot_dot*Ts/sub_loop
            X=X+X_dot*Ts/sub_loop
            Y=Y+Y_dot*Ts/sub_loop

        # Take the last states
        new_states[0]=x_dot
        new_states[1]=y_dot
        new_states[2]=psi
        new_states[3]=psi_dot
        new_states[4]=X
        new_states[5]=Y
        #print("newstates",new_states)

        return new_states,x_dot_dot,y_dot_dot,psi_dot_dot
