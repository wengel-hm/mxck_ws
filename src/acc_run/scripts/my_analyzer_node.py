#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud, Image, CompressedImage, CameraInfo
from geometry_msgs.msg import Point, Point32, PoseStamped, TransformStamped
from std_msgs.msg import Bool, Float32
from nav_msgs.msg import Path
import math
import time
import transformations
import cv2
from cv_bridge import CvBridge, CvBridgeError
# from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
import tf2_geometry_msgs
from scipy.optimize import curve_fit

class AnalyzerNode:
    def __init__(self):
        rospy.init_node("analyzer_node", anonymous=True)
        rospy.loginfo("ROS Lidar Analyzer started!")
        
        #
        # Subscriber
        #
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)

        self.path_sub = rospy.Subscriber('/path', Path, self.boundary_callback) # Mittellinie sub -> muss noch zu seiten umgeschrieben werden 
        self.camera_sub = rospy.Subscriber("/camera/color/image_jpeg", CompressedImage, self.camera_callback, queue_size=1)
        #self.camera_sub = rospy.Subscriber("yolo/results", PointCloud, self.camera_callback) # Bekommt die Kameradaten mit erkannten objekten
        #self.camera_sub_raw = rospy.Subscriber("/camera/color/image_jpeg", PointCloud, self.camera_callback) # Bekommt die Kameradaten in Rohfassung
        #self.boundary_sub = rospy.Subscriber("/track_boundaries", PointCloud, self.boundary_callback)  # Bekommt die Streckenbegrenzung (gibts noch nicht)
        #self.own_speed_sub = rospy.Subscriber("/vehicle_speed", float, self.speed_callback)  # Bekommt die Geschwindigkeit des eigenen Fahrzeugs (gibts noch nicht)

        #
        # Publisher
        #
        self.distance_pub = rospy.Publisher("/distance_to_vehicle", Float32, queue_size=1)
        self.speed_front_vehicle_pub = rospy.Publisher("/speed_of_front_vehicle", Float32, queue_size=1) # Kann als soll-v verwendet werden
        self.best_cluster_scan_pub = rospy.Publisher("/best_cluster_scan", LaserScan, queue_size=1)
        self.image_pub = rospy.Publisher("/projected_lidar_image", Image, queue_size=1)
        self.camera_info_pub = rospy.Publisher("/camera/color/camera_info", CameraInfo, queue_size=1)  # CameraInfo Publisher

        #self.middle_line_pub = rospy.Publisher("/middle_line", Path, queue_size=1)
        #self.left_boundary_pub = rospy.Publisher("/left_boundary", Path, queue_size=1)
        #self.right_boundary_pub = rospy.Publisher("/right_boundary", Path, queue_size=1)

        #self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        #
        # Data placeholders und co
        #

        #self.middle_line = []  # Mittellinie
        #self.left_boundary = []  # Linke Streckenbegrenzung
        #self.right_boundary = []  # Rechte Streckenbegrenzung

        # Fake Werte für Mittellinie zum Testen
        #if False:
        #    self.x_ref = np.array([-0.3025, -0.3275, -0.3525, -0.3775, -0.4025, -0.4275, -0.4525, -0.4775, -0.5025, -0.5275])
        #    self.y_ref = np.array([0.06395297, 0.06784961, 0.07163364, 0.07530504, 0.07886383, 0.08231, 0.08564354, 0.08886447, 0.09197278, 0.09496847])
        #    self.update_boundaries(self.x_ref, self.y_ref)

        # Kameramatrix
        self.K = np.array([[454.7581481933594, 0.0, 322.9541015625],
                           [0.0, 454.8823547363281, 181.71121215820312],
                           [0.0, 0.0, 1.0]])
        
        self.track_boundaries = []  # Dynamisch aktualisierte Streckenbegrenzungen

        self.own_speed_sub = 0.0  # Fake Wert zum testen - Eigene Geschwindigkeit in m/s

        self.real_lidar_data = None
        self.last_lidar_receive_time = None  # Time when the last lidar data was received
        
        self.camera_image = None
        self.bridge = CvBridge()

        self.previous_distance = None
        self.previous_time = None

        self.front_vehicle_speed_filtered = 0.0
        self.filter_constant = 0.5  # PT1-Filter Konstante

        self.timer = rospy.Timer(rospy.Duration(10), self.timer_callback) # Checkt alle 10 Sekunden ob Funktion noch läuft
        self.camera_info_timer = rospy.Timer(rospy.Duration(1), self.publish_camera_info)  # Timer for CameraInfo

        # Neuer Kram für Fusion
        # Beispielhafte experimentelle Daten (Entfernungen in Metern und gemessene Offsets)
        self.known_distances = np.array([1, 2, 3, 5, 7, 10, 15])  # Entfernungen in Metern
        self.observed_offsets = np.array([0.8, 0.5, 0.35, 0.25, 0.2, 0.15, 0.1])  # Offsets in Metern

        # Kurvenanpassung durchführen
        self.scaling_function = self.calibrate_scaling_function(self.known_distances, self.observed_offsets)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Start the static transform broadcaster
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.publish_static_transform()

        self.accuracy_data = []

##########################################################


    #
    # Callback Funktionen
    #

    # Callback des Timers
    def timer_callback(self, event):
        rospy.loginfo("ROS Lidar Analyzer still alive!")

    # Callback für track boundaries - bekommt die Punkte der Streckenbegrenzung
    def boundary_callback(self, data):
        #rospy.loginfo("Boundry Callback aufgerufen!")
        x_ref, y_ref, _ = self.get_trajectory(data)  # psi ist irrelevant
        self.update_boundaries(x_ref, y_ref)
        '''
        x_ref, y_ref, _ = self.get_trajectory(data)  # psi ist irrelevant

        left_boundary = []
        right_boundary = []

        for x, y in zip(x_ref, y_ref):
            left_x = x - 0.5
            left_y = y
            right_x = x + 0.5
            right_y = y

            left_boundary.append(Point32(left_x, left_y, 0))
            right_boundary.append(Point32(right_x, right_y, 0))

        self.track_boundaries = left_boundary + right_boundary
        '''
        #self.track_boundaries = [Point32(pt.x, pt.y, pt.z) for pt in data.points]
        #rospy.loginfo("Track boundaries updated.")

    # Callback der eigenen Geschwindigkeit
    def speed_callback(self, data):
        self.own_speed_sub = data

    # Callback des Lidars - setzt zudem die Zeit des letzten Lidaraufrufs
    def lidar_callback(self, data):
        self.real_lidar_data = data
        self.last_lidar_receive_time = rospy.Time.now()
        self.lidar_frame_id = data.header.frame_id  # Hier extrahieren wir den frame_id

    # Callback der Kamera - setzt die x,y,z Koordinaten der Kamera
    def camera_callback(self, data):
        #rospy.loginfo("Kamera Callback aufgerufen!")
        try:
            self.camera_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            #rospy.loginfo(f"CvBridge Error: {e}")
            rospy.logerr(f"CvBridge Error: {e}")
    
##########################################################

    #
    # Kamerainfo
    #

    def publish_camera_info(self, event):
        camera_info = CameraInfo()
        camera_info.header.stamp = rospy.Time.now()
        camera_info.header.frame_id = "camera_frame"
        camera_info.height = 360
        camera_info.width = 640
        camera_info.distortion_model = "plumb_bob"
        camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_info.K = [454.7581481933594, 0.0, 322.9541015625,
                         0.0, 454.8823547363281, 181.71121215820312,
                         0.0, 0.0, 1.0]
        camera_info.R = [1.0, 0.0, 0.0,
                         0.0, 1.0, 0.0,
                         0.0, 0.0, 1.0]
        camera_info.P = [454.7581481933594, 0.0, 322.9541015625, 0.0,
                         0.0, 454.8823547363281, 181.71121215820312, 0.0,
                         0.0, 0.0, 1.0, 0.0]
        self.camera_info_pub.publish(camera_info)


##########################################################

    
    #
    # Für die Lidar-Punkt-Projektion
    #

    def scaling_model(self, distance, a, b, c):
        return a * distance ** 2 + b * distance + c
    
    def calibrate_scaling_function(self, distances, offsets):
        # Führe die Kurvenanpassung durch
        params, _ = curve_fit(self.scaling_model, distances, offsets)
        return lambda d: self.scaling_model(d, *params)
    
    def project_to_image_plane(self, point):
        depth = point[2]
        scaling_factor = 1 + self.scaling_function(depth)  # dynamischer Skalierungsfaktor
        pixel_coordinates = self.K @ (point / scaling_factor)
        pixel_coordinates /= pixel_coordinates[2]
        return pixel_coordinates[:2]

    def apply_lidar_camera_offset(self, point):
        offset_x = 0.00
        offset_y = 0.00
        offset_z = 0.00
        point[0] -= offset_x
        point[1] -= offset_y
        point[2] -= offset_z
        return point

    def publish_static_transform(self):
        
        static_transform_stamped = TransformStamped()

        static_transform_stamped.header.stamp = rospy.Time.now()
        static_transform_stamped.header.frame_id = "camera_frame"
        static_transform_stamped.child_frame_id = "laser"

        # Set translation
        static_transform_stamped.transform.translation.x = 0.065  # Replace with actual x offset
        static_transform_stamped.transform.translation.y = 0.02  # Replace with actual y offset
        static_transform_stamped.transform.translation.z = 0.0  # Replace with actual z offset

        # Set rotation (90 degrees around x-axis to align Lidar's xy-plane with Camera's z direction)
        #quat = quaternion_from_euler(math.pi / 2, math.pi / 2, 0)
        quat = transformations.quaternion_from_euler(math.pi / 2, math.pi / 2, 0)
        static_transform_stamped.transform.rotation.x = quat[0] # 0.0
        static_transform_stamped.transform.rotation.y = quat[1] # 0.0
        static_transform_stamped.transform.rotation.z = quat[2] # 0.0
        static_transform_stamped.transform.rotation.w = quat[3] # 1.0


        self.static_broadcaster.sendTransform(static_transform_stamped)

    def transform_point(self, point, from_frame, to_frame):
        point_stamped = tf2_geometry_msgs.PointStamped()
        point_stamped.header.frame_id = from_frame
        point_stamped.header.stamp = rospy.Time(0)
        point_stamped.point.x = point[0]
        point_stamped.point.y = point[1]
        point_stamped.point.z = point[2]
        
        try:
            transform = self.tf_buffer.lookup_transform(to_frame, from_frame, rospy.Time(0), rospy.Duration(1.0))
            transformed_point = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
            return np.array([transformed_point.point.x, transformed_point.point.y, transformed_point.point.z])
        except tf2_ros.LookupException as e:
            rospy.logerr(f"Transform lookup error: {e}")
            return None
        except tf2_ros.ExtrapolationException as e:
            rospy.logerr(f"Transform extrapolation error: {e}")
            return None

    def transform_and_project_lidar_points(self, lidar_data):
        projected_points = []
        for i, distance in enumerate(lidar_data.ranges):
            if distance > 0.5 and distance < 20.0:
                angle = lidar_data.angle_min + i * lidar_data.angle_increment
                object_x = distance * math.cos(angle)
                object_y = distance * math.sin(angle)
                object_z = 0

                # Filtere Punkte hinter dem Fahrzeug aus
                if object_x > 0:
                    continue

                lidar_point = np.array([object_x, object_y, object_z])
                camera_point = self.transform_point(lidar_point, self.lidar_frame_id, "camera_frame")
                if camera_point is not None:
                    pixel_coords = self.project_to_image_plane(camera_point)
                    if not np.any(np.isinf(pixel_coords)) and not np.any(np.isnan(pixel_coords)):
                        projected_points.append(pixel_coords)
        return projected_points
    
    
##########################################################

    # 
    # Für Mittellinienerkennung
    #

    # Funktion zur Aktualisierung der Grenzen
    def update_boundaries(self, x_ref, y_ref):
        self.middle_line = []
        self.left_boundary = []
        self.right_boundary = []
        middle_path = Path()
        left_path = Path()
        right_path = Path()

        # Set the header for the Path messages
        middle_path.header.frame_id = "map"
        middle_path.header.stamp = rospy.Time.now()
        left_path.header.frame_id = "map"
        left_path.header.stamp = rospy.Time.now()
        right_path.header.frame_id = "map"
        right_path.header.stamp = rospy.Time.now()

        # Punkte entlang der Mittellinie hinzufügen
        for x, y in zip(x_ref, y_ref):
            self.middle_line.append((-x, y))

        # Punkte entlang der Mittellinie extrapolieren
        for _ in range(150):  # 50 zusätzliche Punkte für 5 Meter
            x_diff = self.middle_line[-1][0] - self.middle_line[-2][0]
            y_diff = self.middle_line[-1][1] - self.middle_line[-2][1]
            new_x = self.middle_line[-1][0] + x_diff
            new_y = self.middle_line[-1][1] + y_diff
            self.middle_line.append((new_x, new_y))

        # Linke und rechte Begrenzungslinien basierend auf der Mittellinie erstellen
        for x, y in self.middle_line:
            left_y = y - 1.5 #TODO: Wieder auf 0.25 oder so ändern (auch y)
            right_y = y + 1.5

            self.left_boundary.append((x, left_y))
            self.right_boundary.append((x, right_y))

            # Mittellinie
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0
            middle_path.poses.append(pose_stamped)

            # Linke Begrenzung
            left_pose_stamped = PoseStamped()
            left_pose_stamped.header.frame_id = "map"
            left_pose_stamped.header.stamp = rospy.Time.now()
            left_pose_stamped.pose.position.x = x
            left_pose_stamped.pose.position.y = left_y
            left_pose_stamped.pose.position.z = 0
            left_path.poses.append(left_pose_stamped)

            # Rechte Begrenzung
            right_pose_stamped = PoseStamped()
            right_pose_stamped.header.frame_id = "map"
            right_pose_stamped.header.stamp = rospy.Time.now()
            right_pose_stamped.pose.position.x = x
            right_pose_stamped.pose.position.y = right_y
            right_pose_stamped.pose.position.z = 0
            right_path.poses.append(right_pose_stamped)

        # Publish the boundaries and the middle line
        self.left_boundary_pub.publish(left_path)
        self.right_boundary_pub.publish(right_path)
        self.middle_line_pub.publish(middle_path)
    
    def get_trajectory(self, msg):
        x_coords, y_coords, z_rotations = [], [], []

        for pose_stamped in msg.poses: #für pathplanning
        #for pose_stamped in msg.poses[1:]:
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
            #euler = euler_from_quaternion(quaternion)
            euler = transformations.euler_from_quaternion(quaternion)

            # z-axis rotation in radians
            z_rotation = euler[2]
            z_rotations.append(z_rotation)
        
        return np.array(x_coords), np.array(y_coords), np.array(z_rotations)
    

##########################################################

    #
    # Überprüfen, ob Objekt innerhalb der Streckenbegrenzung liegt
    #

    def detect_object_within_boundaries(self, object_distance, object_angle):
        object_x = object_distance * math.cos(math.radians(object_angle))
        object_y = object_distance * math.sin(math.radians(object_angle))

        is_within_boundaries = (-20 <= object_x <= 0) and (-1 <= object_y <= 1) and (object_angle <= -120 or object_angle >= 120)# Hier werden die Fake Begrenzungen gecheckt

        return is_within_boundaries
        # Im Testmodus werden die Streckenbegrenzungen simuliert:
        if self.test_mode:
            object_x = object_distance * math.cos(math.radians(object_angle))
            object_y = object_distance * math.sin(math.radians(object_angle))

            is_within_boundaries = (-4 <= object_x <= 0) and (-1.5 <= object_y <= 1.5) # Hier werden die Fake Begrenzungen gecheckt

            return is_within_boundaries
        
        # Wenn Linienerkennung verwendet wird:
        else:
            # Berechne die Objektkoordinaten im Lidar-Koordinatensystem
            object_x = object_distance * math.cos(math.radians(object_angle))
            object_y = object_distance * math.sin(math.radians(object_angle))

            # Überprüfe, ob die Koordinaten innerhalb der Streckenbegrenzung liegen
            # TODO: Kann sein dass ich hier nur die x Koordinate brauche und mir y einfach paar meter puffer gebe
            is_within_boundaries = any((boundary_point.x <= object_x <= boundary_point.x + 0.4 and boundary_point.y <= object_y <= boundary_point.y + 0.4) for boundary_point in self.track_boundaries)

            return is_within_boundaries
            '''
            if not self.camera_points:
                rospy.loginfo("Keine Kamerapunkte verfügbar.")
                return False

            # Transformation der Lidar-Daten in ein Koordinatensystem, das für die Kameraprojektion verwendet wird
            object_x = object_distance * math.cos(math.radians(object_angle))
            object_y = object_distance * math.sin(math.radians(object_angle))
            object_z = 0  # Angenommen, die z-Koordinate ist auf Straßenniveau


            # Projektion überprüfen
            camera_point = np.array([object_x, object_y, object_z])
            camera_point = self.apply_lidar_camera_offset(camera_point)  # Offset berücksichtigen
            pixel_coords = self.project_to_image_plane(camera_point)

            # Überprüfen, ob die projizierten Koordinaten innerhalb der Begrenzung liegen
            is_within_boundaries = False  # Standardwert

            for boundary_point in self.track_boundaries:
                boundary_pixel = self.project_to_image_plane(np.array([boundary_point.x, boundary_point.y, boundary_point.z, 1]))
                if np.linalg.norm(boundary_pixel - pixel_coords) < 5:  # Schwellwert zum Abgleichen von Punkten
                    is_within_boundaries = True
                    break

            return is_within_boundaries
            '''


##########################################################

    #
    # Fusion von Lidar- und Kameradaten und Überprüfung der erkannten Objekte
    #
    '''
    def fuse_lidar_camera_data(self, object_distance, object_angle):
        if not self.camera_points:
            rospy.loginfo("Keine Kamerapunkte verfügbar.")
            return False

        # Transformation der Lidar-Daten in ein Koordinatensystem, das für die Kameraprojektion verwendet wird
        object_x = object_distance * math.cos(math.radians(object_angle))
        object_y = object_distance * math.sin(math.radians(object_angle))
        object_z = 0  # Angenommen, die z-Koordinate ist auf Straßenniveau

        camera_point = np.array([object_x, object_y, object_z])
        camera_point = self.apply_lidar_camera_offset(camera_point)  # Offset berücksichtigen

        # Überprüfen, ob die projizierten Koordinaten innerhalb der durch die Kamera erkannten Objekte liegen
        is_within_object = False  # Standardwert

        for detected_object in self.camera_points:
            projected_camera_point = self.project_to_image_plane(camera_point)
            projected_detected_object = self.project_to_image_plane(detected_object)

            if (abs(projected_camera_point[0] - projected_detected_object[0]) < 5 and
                abs(projected_camera_point[1] - projected_detected_object[1]) < 5):
                is_within_object = True
                break

        return is_within_object
    '''
##########################################################
    #
    # Berechnen der Geschwindigkeit des vorausfahrenden Fahrzeugs
    #

    def calculate_front_vehicle_speed(self, current_distance):
        current_time = time.time()

        if self.previous_distance is not None and self.previous_time is not None:
            time_diff = current_time - self.previous_time

            # Verhindert Instabilitäten in der Berechnung
            if time_diff < 0.001:
                return 0.0  

            distance_diff = current_distance - self.previous_distance
            relative_speed = distance_diff / time_diff

            # Überprüfe, ob ein Clusterwechsel stattgefunden hat
            if self.is_cluster_change(current_distance, self.previous_distance):
                self.previous_distance = current_distance
                self.previous_time = current_time
                return 0.0

            # Berechnung der absoluten Geschwindigkeit des vorausfahrenden Fahrzeugs
            # TODO: Überprüfen ob das so stimmt
            if current_distance == 999 or self.previous_distance == 999:
                front_vehicle_absolute_speed = 0.0
            else:
                front_vehicle_absolute_speed = self.own_speed_sub + relative_speed 

            # PT1-Filter anwenden
            self.front_vehicle_speed_filtered = (self.filter_constant * front_vehicle_absolute_speed +
                                                 (1 - self.filter_constant) * self.front_vehicle_speed_filtered)

            self.previous_distance = current_distance
            self.previous_time = current_time

            return self.front_vehicle_speed_filtered
        else:
            self.previous_distance = current_distance
            self.previous_time = current_time
            return 0.0
    

    def is_cluster_change(self, current_distance, previous_distance, tolerance=1.0):
        return abs(current_distance - previous_distance) > tolerance
##########################################################

    #
    # Funktionen zum visualisieren
    #

    def publish_best_cluster_as_laserscan(self, best_cluster):
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = self.lidar_frame_id  # Setze das korrekte Frame ID
        
        scan.angle_min = self.real_lidar_data.angle_min
        scan.angle_max = self.real_lidar_data.angle_max
        scan.angle_increment = self.real_lidar_data.angle_increment
        scan.time_increment = self.real_lidar_data.time_increment
        scan.scan_time = self.real_lidar_data.scan_time
        scan.range_min = self.real_lidar_data.range_min
        scan.range_max = self.real_lidar_data.range_max

        # Initialisiere alle Ranges mit "Inf" (keine Messung)
        scan.ranges = [float('inf')] * len(self.real_lidar_data.ranges)

        # Füge die Punkte des besten Clusters hinzu
        for distance, angle, _, _ in best_cluster:
            index = int((angle - scan.angle_min) / scan.angle_increment)
            scan.ranges[index] = distance

        self.best_cluster_scan_pub.publish(scan)

##########################################################
##########################################################

    #
    # Hauptfunktion zum Entscheidungen Treffen
    #

    def process_data(self):

        #self.update_boundaries(self.x_ref, self.y_ref)
        
        # Abbrechen falls keine Lidardaten vorhanden sind
        if self.real_lidar_data is None:
            rospy.loginfo("No Lidar data available")
            return
        
        # Abbrechen wenn zu lange keine Lidar Daten empfangen wurden
        if (rospy.Time.now() - self.last_lidar_receive_time).to_sec() > 1.0:  # TODO: Kürzer?
            rospy.loginfo("Waiting for new Lidar data...")
            return


        ###################################################

        #
        # Kamera-Lidar-Fusion
        #
        '''
        if self.camera_image is None:
            rospy.loginfo("Waiting for Camera data...")
        else:
            projected_points = self.transform_and_project_lidar_points(self.real_lidar_data)
            image_with_lidar = self.camera_image.copy()

            for point in projected_points:
                x, y = int(point[0]), int(point[1])
                if 0 <= x < image_with_lidar.shape[1] and 0 <= y < image_with_lidar.shape[0]:
                    cv2.circle(image_with_lidar, (x, y), 5, (0, 0, 255), -1)

            try:
                image_msg = self.bridge.cv2_to_imgmsg(image_with_lidar, "bgr8")
                self.image_pub.publish(image_msg)
            except CvBridgeError as e:
                rospy.logerr(f"CvBridge Error: {e}")
        '''
        ###################################################


        # Hier wird jetzt alles verarbeitet

        object_detected = False  # Flag, um festzustellen, ob ein Objekt erkannt wurde
            
        valid_points = []
        best_cluster = []

        for i, distance in enumerate(self.real_lidar_data.ranges):
            if 0.5 < distance < 20.0:
                angle = self.real_lidar_data.angle_min + i * self.real_lidar_data.angle_increment
                object_x = distance * math.cos(angle)
                object_y = distance * math.sin(angle)

                # Filtere Punkte hinter dem Fahrzeug aus
                if object_x > 0:
                    continue

                if self.detect_object_within_boundaries(distance, math.degrees(angle)):
                    #rospy.loginfo(f'Angle: {math.degrees(angle):.2f} deg')
                    #rospy.loginfo(f'Distance: {distance:.2f} m')

                    #rospy.loginfo(f'x: {object_x:.2f} ')
                    #rospy.loginfo(f'y: {object_y:.2f} ')
                    valid_points.append((distance, angle, object_x, object_y))

        # Clusterbildung
        clusters = []
        cluster = []

        for point in valid_points:
            if not cluster:
                cluster.append(point)
            else:
                last_point = cluster[-1]
                if (abs(point[2] - last_point[2]) <= 0.3) and (abs(point[3] - last_point[3]) <= 0.3):
                    cluster.append(point)
                else:
                    clusters.append(cluster)
                    cluster = [point]
        
        if cluster:
            clusters.append(cluster)

        # Bestes Cluster wählen (das am nächsten zur Mitte liegt)
        if clusters:
            best_cluster = min(clusters, key=lambda cl: abs(sum(pt[3] for pt in cl) / len(cl))) #TODO: Statt beste cluster zur mittellinie könnte hier das cluster verwendet werden, welches von der kamera als ein fahrzeug erkannt wurde
            
            current_distance = np.mean([pt[0] for pt in best_cluster])
            front_vehicle_speed = self.calculate_front_vehicle_speed(current_distance)
            self.previous_distance = current_distance
            self.previous_time = time.time()

            self.distance_pub.publish(Float32(current_distance))
            self.speed_front_vehicle_pub.publish(Float32(front_vehicle_speed))
            rospy.loginfo(f'Current distance to the vehicle in front: {current_distance:.2f} meters')
            rospy.loginfo(f'Current speed of the vehicle in front: {front_vehicle_speed:.2f} m/s')
            object_detected = True

            # Bestes Cluster veröffentlichen
            self.publish_best_cluster_as_laserscan(best_cluster)

            ##########################################

            # Transformation 2.0 
            if self.camera_image is not None:
                projected_points = self.transform_and_project_lidar_points(self.real_lidar_data)
                image_with_lidar = self.camera_image.copy()

                for point in projected_points:
                    x, y = int(point[0]), int(point[1])
                    if 0 <= x < image_with_lidar.shape[1] and 0 <= y < image_with_lidar.shape[0]:
                        cv2.circle(image_with_lidar, (x, y), 3, (0, 0, 0), -1)  # Schwarz für andere Punkte

                for point in best_cluster:
                    camera_point = self.transform_point(np.array([point[2], point[3], 0]), self.lidar_frame_id, "camera_frame")
                    if camera_point is not None:
                        pixel_coords = self.project_to_image_plane(camera_point)
                        if not np.any(np.isinf(pixel_coords)) and not np.any(np.isnan(pixel_coords)):
                            x, y = int(pixel_coords[0]), int(pixel_coords[1])
                            if 0 <= x < image_with_lidar.shape[1] and 0 <= y < image_with_lidar.shape[0]:
                                cv2.circle(image_with_lidar, (x, y), 5, (0, 0, 255), -1)  # Rot für bestes Cluster
                                # Genauigkeitsdaten sammeln
                                self.accuracy_data.append((x, y, point[2], point[3]))


                try:
                    image_msg = self.bridge.cv2_to_imgmsg(image_with_lidar, "bgr8")
                    self.image_pub.publish(image_msg)
                except CvBridgeError as e:
                    rospy.logerr(f"CvBridge Error: {e}")


            ##########################################

        if not object_detected:
            current_distance = 999
            front_vehicle_speed = 0.0  # Setze die Geschwindigkeit auf 0, wenn kein Objekt erkannt wurde
            self.distance_pub.publish(Float32(current_distance))
            self.speed_front_vehicle_pub.publish(Float32(front_vehicle_speed))
            rospy.loginfo(f'Current distance to the vehicle in front: {current_distance:.2f} meters')
            rospy.loginfo(f'Current speed of the vehicle in front: {front_vehicle_speed:.2f} m/s')

        #self.evaluate_accuracy()


    def evaluate_accuracy(self):
        if not self.accuracy_data:
            return

        pixel_errors = []
        for x, y, true_x, true_y in self.accuracy_data:
            pixel_errors.append(np.sqrt((x - true_x) ** 2 + (y - true_y) ** 2))

        mean_error = np.mean(pixel_errors)
        std_error = np.std(pixel_errors)

        rospy.loginfo(f"Mean projection error: {mean_error:.2f} pixels")
        rospy.loginfo(f"Standard deviation of projection error: {std_error:.2f} pixels")



##########################################################
##########################################################

    # Broadcast TF transform from laser to map
    def broadcast_transform(self, event):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "laser"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    #
    # Run Funktion - hier kann die Hz-Rate eingestellt werden!
    #

    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            self.process_data()
            rate.sleep()


#
# Funktionsaufruf main
#

if __name__ == '__main__':
    an_node = AnalyzerNode()
    an_node.run()
    #    rospy.spin()

