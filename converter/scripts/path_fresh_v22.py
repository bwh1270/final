#!/usr/bin/env python

import time
import numpy as np
import threading
import rospy
import math
from icuas23_competition.msg import poi
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, Int16
from std_msgs.msg import UInt32MultiArray, UInt8MultiArray
from converter.msg import xyz
from nav_msgs.msg import Odometry

class Path:
    def __init__(self, poi_len: int):

        self.scale = int(1 / 0.2) # scale of 0.2m
        self.absolute_location = np.array([0, 0, 0]) # in meters
        self.current_idx = np.array([0, 0])
        self.absolute_orientation = np.array([0, 0, 0, 0])
        self.set_point = 0
        self.current_score = 100
        self.total_poi = np.array([]) # total list of poi to go (do not change)
        self.poi_to_go = np.array([]) # poi left to go (changes)
        self.target_poi = np.array([]) # current poi to go (single poi in meters)
        self.target_poi_idx = np.array([]) # current poi to go (single poi in array)
        self.home_check = False
        self.home = np.array([0, 0, 0])  # home poi (single poi in meters)
        self.home_idx = np.array([0, 0]) # home poi (single poi in array)
        self.need_slam_flag = False
        self.poi_arrived = False
        self.time_limit = 0
        self.target_number = 0
        self.poi_check = False
        self.set_point_x = 0
        self.set_point_y = 0
        self.yaw_rad = 0
        self.set_point_list = []
        self.is_first_yaw = True
        
        self.map_x = 8.4
        self.map_y = 6

        # Timer return to home
        self.rth_timer_start = time.time()
        self.rth_time_limit_min = 7 # min
        self.rth_time_limit = self.rth_time_limit_min * 60
        self.is_rth = False
        self.absolute_yaw = 0
        self.threshold_yaw = math.pi / 36
        self.check_poi_threshold = 0.1

        self.time_break_threshold = 20 # s
        self.time_start_code = time.time()

        self.idx_map_x = int(self.map_x * self.scale + 1) # 10m x scale factor + 1
        self.idx_map_y = int(self.map_y * self.scale + 1) # 12m x scale factor + 1

        

        # Subscriber
        rospy.Subscriber('/carrot_team/voxel_grid_map_32', UInt32MultiArray, self.slam_map_callback)
        rospy.Subscriber('/falconblack/vrpn_client/estimated_odometry', Odometry, self.absolute_location_callback)
        rospy.Subscriber('/red/poi', poi, self.total_poi_callback)
        
        # Publisher
        self.pose_publisher = rospy.Publisher('/red/tracker/input_pose', PoseStamped, queue_size=10)
        self.pub_image = rospy.Publisher('/carrot_team/poi_idx', Int16, queue_size=10)

        self.pose_msg = PoseStamped()
        self.pose_msg.pose.position.x = 0
        self.pose_msg.pose.position.y = 0
        self.pose_msg.pose.position.z = 1

        self.pose_msg.pose.orientation.x = 0
        self.pose_msg.pose.orientation.y = 0
        self.pose_msg.pose.orientation.z = 0
        self.pose_msg.pose.orientation.w = 1


        # Subscribe each score map
        self.poi_len = poi_len
        target_topics = [f'/carrot_team/target{i}' for i in range(self.poi_len)]
        subscriber_list = [f'self.subscriber{i}' for i in range(self.poi_len)]
        #self.score_map_callback_list = [self.score_map_callback for _ in range(self.poi_len)]
        self.score_map = {}
        for i, topic in enumerate(target_topics):
            subscriber_list[i] = rospy.Subscriber(topic, UInt32MultiArray, lambda msg, index=i: self.score_map_callback(msg, index))
            #self.subscribers.append(self.subscriber)
            self.score_map[i] = np.ones((self.idx_map_x, self.idx_map_y), dtype=np.uint32)*7654321 # array of 10 * scale + 1 x 12 * scale + 1
        self.slam_map = np.zeros((self.idx_map_x, self.idx_map_y), dtype=np.uint32) # array of 10 * scale + 1 x 12 * scale + 1

        #self.score_map[0] = 
        #rospy.Subscriber('/carrot_team/target0', UInt32MultiArray, self.score_map_callback0)

        
    def quaternion2euler(self, orientation):
        x, y, z, w = orientation
        # PIby2 = (math.pi / 2)

        # # roll : rotation in x axis
        # roll_x = float(2 * (w * x + y * z))
        # roll_y = float(1 - 2 * (x * x + y * y))
        # roll = math.atan2(roll_x, roll_y);

        # # pitch : rotation in y axis
        # pitch_var = float(2 * (w * y - z * x))
        # if (abs(pitch_var) >= 1):
        # #In the event of out of range -> use 90 degrees
        #     pitch = math.copysign(PIby2, pitch_var);
        # else:
        #     pitch = math.asin(pitch_var);

        # yaw : rotation in z-axis
        yaw_x = 2 * (w * z + x * y);
        yaw_y = 1 - 2 * (y * y + z * z);
        yaw = math.atan2(yaw_x, yaw_y);
        
        # deg로 변환 - 주석하면 radian
        # roll  = math.degrees(roll)
        # pitch = math.degrees(pitch)
        # yaw   = math.degrees(yaw)

        # return roll, pitch, yaw
        return yaw
    
    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.

        Input
        :param roll: The roll (rotation around x-axis) angle in radians.
        :param pitch: The pitch (rotation around y-axis) angle in radians.
        :param yaw: The yaw (rotation around z-axis) angle in radians.

        Output
        :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]
    
    def idx2m(self, point_idx):
        idx_x, idx_y, _ = point_idx

        x = idx_x / self.scale - self.map_x / 2
        y = idx_y / self.scale - self.map_y / 2

        return x, y
        
    def m2idx(self, point):
        x, y, _ = point

        x_idx = np.round((x + self.map_x / 2) * self.scale)
        y_idx = np.round((y + self.map_y / 2) * self.scale)

        return x_idx, y_idx
        

    # callback for absolute location
    def absolute_location_callback(self, msg):
        xyz_tmp = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        or_tmp = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.z])
        

        self.absolute_location = xyz_tmp
        self.current_idx = np.array([self.m2idx(self.absolute_location)])
        #print(self.absolute_location)
        #print(self.current_idx)
        self.absolute_orientation = or_tmp
        # quaternion to euler
        self.absolute_yaw = self.quaternion2euler(self.absolute_orientation)

        self.is_drone_location = True

    # callback for score map
    def score_map_callback(self, msg, i):
        #for i in range(self.poi_len):
        self.score_map[i] = np.reshape(msg.data, (self.idx_map_x, self.idx_map_y))

    # callback for SLAM map
    def slam_map_callback(self, msg):
        self.slam_map = np.reshape(msg.data, (self.idx_map_x, self.idx_map_y))

    # callback for poi
    def total_poi_callback(self, msg):
        
        for i in range(len(msg.poi)):
            tmp = np.array([])
            tmp = np.append(tmp, msg.poi[i].x)
            tmp = np.append(tmp, msg.poi[i].y)
            tmp = np.append(tmp, msg.poi[i].z)
        
            self.total_poi = np.append(self.total_poi, tmp)
            
        self.total_poi = np.reshape(self.total_poi, (-1,3))
        #print(np.shape(self.total_poi))
        
        if self.poi_check == False:
            self.poi_to_go = self.total_poi
            self.poi_check = True
            print("poi_to_go saved successfully")


    # set yaw
    def set_yaw(self, yaw):
        q = self.get_quaternion_from_euler(0, 0, yaw)

        self.pose_msg.pose.orientation.x = q[0]
        self.pose_msg.pose.orientation.y = q[1]
        self.pose_msg.pose.orientation.z = q[2]
        self.pose_msg.pose.orientation.w = q[3]


    # set home and home_idx
    def set_home(self):
        self.pose_msg.pose.position.x, self.pose_msg.pose.position.y, self.pose_msg.pose.position.z = self.absolute_location
        self.home = self.absolute_location
        self.home_idx = np.array([self.m2idx(self.home)])
        X_ = np.round((self.home[0] + self.map_x / 2) * self.scale)
        Y_ = np.round((self.home[1] + self.map_y / 2) * self.scale)
        x_ = (self.home[0] + self.map_x / 2)
        y_ = (self.home[1] + self.map_y / 2)
        if (X_ > self.idx_map_x or X_ < 0) or (Y_ > self.idx_map_x or Y_ < 0) or (x_ < 0 or x_ > self.map_x) or (y_ < 0 or y_ > self.map_y):
            print("Drone position not in map")
        else:
            print("Home point saved successfully")


    def yaw_circle(self): # !!
        
        yaw_time = 2 # 5

        # send to yolo
        is_yolo = Int16()
        is_yolo.data = 1

        PI = math.pi
        for i in range(8):
            target_yaw = PI * i / 4 # rad

            ori_x, ori_y, ori_z, ori_w = self.get_quaternion_from_euler(0, 0, target_yaw)

            self.pose_msg.pose.orientation.x = ori_x
            self.pose_msg.pose.orientation.y = ori_y
            self.pose_msg.pose.orientation.z = ori_z
            self.pose_msg.pose.orientation.w = ori_w

            # publish orientation - Only orientation
            self.pose_publisher.publish(self.pose_msg)
            print(f"start Yawing {i}")

            # wait for yaw
            time.sleep(yaw_time)
            

            if self.is_first_yaw == False: # initial value is True
                self.pub_image.publish(is_yolo)
                print("Sending to YOLO")
        
        if self.is_first_yaw:
            self.is_first_yaw = False
    
    def is_arrived_at_poi(self):
        print(np.shape(self.poi_to_go))
        if (len(self.poi_to_go) == 0):
            print("There is no remained POI")
            while True:
                print("Racing End")
                sleep(10)

        # Check whether vehicle is placed at poi_idx
        poi_x , poi_y = self.target_poi_idx
        curr_x, curr_y = self.current_idx
        delta_idx_x = poi_x - curr_x;
        delta_idx_y = poi_y - curr_y;

        target_poi_xy = [poi_x, poi_y, 0] # value 0 will be throwing away
        poi_meter_x, poi_meter_y = self.idx2m(target_poi_xy) 
        updating_x, updating_y, _ = self.absolute_location # current x, y
        delta_meter_x = abs(updating_x) - abs(poi_meter_x)
        delta_meter_y = abs(updating_y) - abs(poi_meter_y)

        if ((delta_idx_x == 0) and (delta_idx_y == 0)) or ((delta_meter_x < 0.2) and (delta_meter_y < 0.2)):
            print("Arrived at Target POI")
            # Is Stopping vehicle needed???
            self.yaw_circle()
            self.set_target_poi()


    # set target_poi to the shortest poi
    def set_target_poi(self):
        
        dist = np.array([])
        idx = np.arange(len(self.poi_to_go))
        #print(idx)

        for poi_tmp in self.poi_to_go:
            #print(poi_tmp)
            poi_x, poi_y = poi_tmp[:2]
            dist = np.append(dist, abs(np.linalg.norm(np.array([poi_x, poi_y]) - np.array([self.absolute_location[0], self.absolute_location[1]]))))
        
        idx = idx[dist.argsort()]
        
        if len(idx) > 0:
            self.target_number = idx[0]
            self.target_poi = self.poi_to_go[idx[0]]
            self.target_poi_idx = np.array([np.round((self.target_poi[0] + self.map_x / 2) * self.scale), np.round((self.target_poi[1] + self.map_x / 2) * self.scale)])
            self.poi_to_go = np.delete(self.poi_to_go, idx[0], axis=0)
            # print('target poi:' , self.target_poi)
            #print(np.shape(self.poi_to_go), "del")
        else:
            print("Error! No POI")


    def set_set_point(self, flag):
        
        if flag == False :
            # make more path
            curr_x, curr_y = self.set_point_x, self.set_point_y
            # print("try making more path")
        else:
            curr_x, curr_y = self.current_idx
            # print("try going to one step")
            
        poi_x , poi_y = self.target_poi_idx
        #if self.target_number in self.score_map: 
        print( self.score_map[self.target_number])

        print("crurr_x :", curr_x)
        print("crurr_y :", curr_y)
        score_map_segment = self.score_map[self.target_number][max(0, curr_x - 1) : min(self.idx_map_x, curr_x + 2), max(0, curr_y - 1) : min(self.idx_map_y, curr_y + 2)]
        print("score_map segment : ")
        print(score_map_segment)

        if len(score_map_segment) == 0:
            print("Error! No score map segment")
        min_score = np.min(score_map_segment)
        self.current_score = min_score

        segment_idx = np.where(score_map_segment == min_score)

        dist = np.array([])

        for i in range(len(segment_idx[0])):
            temp = abs(np.linalg.norm(np.array([poi_x, poi_y]) - np.array([segment_idx[0][i] + curr_x - 1, segment_idx[1][i] + curr_y - 1])))
            dist = np.append(dist, temp)
        
        min_idx = np.argmin(dist)

        self.set_point_x = segment_idx[0][min_idx] + curr_x - 1
        self.set_point_y = segment_idx[1][min_idx] + curr_y - 1
        

        # set yaw angle, +x = 0deg
        if self.set_point_x - curr_x == 1:

            if self.set_point_y - curr_y == 0:
                self.yaw_rad = math.radians(0)
            elif self.set_point_y - curr_y == 1:
                self.yaw_rad = math.radians(-45)
            elif self.set_point_y - curr_y == -1:
                self.yaw_rad = math.radians(45)
        
        elif self.set_point_x - curr_x == 0:

            if self.set_point_y - curr_y == 1:
                self.yaw_rad = math.radians(-90)
            elif self.set_point_y - curr_y == -1:
                self.yaw_rad = math.radians(90)

        elif self.set_point_x - curr_x == -1:

            if self.set_point_y - curr_y == 0:
                self.yaw_rad = math.radians(180)
            elif self.set_point_y - curr_y == 1:
                self.yaw_rad = math.radians(-135)
            elif self.set_point_y - curr_y == -1:
                self.yaw_rad = math.radians(135)
                
        
        # check if the set_point is mapped with SLAM

        if self.slam_map[self.set_point_x, self.set_point_y] == 1:
            self.need_slam_flag = False
        
        elif self.slam_map[self.set_point_x, self.set_point_y] == 0:
            self.need_slam_flag = True         

        #else:
            #print("Invalid target_number:", self.target_number)
            #while True:
                #print("There is no target_number => score map")

        self.set_point_list.append([self.set_point_x, self.set_point_y, self.home[2], self.yaw_rad]) # for making trajectory


    def check_poi_arrived(self):
        
        poi_x , poi_y = self.target_poi_idx
        delta_x = poi_x - self.set_point_x
        delta_y = poi_y - self.set_point_y
        yaw = self.yaw_rad
        
        if ((delta_x == 0) and (delta_y == 0)):
            print("Can not make more path")
            return True
        
        elif (abs(yaw) - abs(math.radians(-45)) == 0):
            print("Can not make more path")
            return True
        
        else:
            print("Can make more path")
            return False
    
        

    def lower_controller(self, point):
        
        x_meter, y_meter, _, yaw_rad = self.idx2m(point) #!! changed
        
        self.pose_msg.pose.position.x = x_meter
        self.pose_msg.pose.position.y = y_meter
        self.pose_msg.pose.position.z = self.home[2]
        self.yaw_rad = yaw_rad # !! Added
        self.set_yaw(self.yaw_rad)
        
        self.pose_publisher.publish(self.pose_msg)
        print("Carrot Falcon starting moving now")

        while True:
            # Check arrived
            updating_x, updating_y, _ = self.absolute_location
            delta_x = abs(updating_x) - abs(x_meter)
            delta_y = abs(updating_y) - abs(y_meter)
            if (delta_x < 0.2) and (delta_y < 0.2):
                print("Arrived at desired one step")
                break;
            #sleep(0.5)


    def upper_controller(self): # !!

        trajectory_len = len(self.set_point_list)
        for step in range(trajectory_len):
            point = self.set_point_list[step]

            # check idx overflow
            if step < trajectory_len - 1:
                point_next = self.set_point_list[step + 1]
                if point[3] == point_next[3]:
                    continue
            else: # Last index
                pass

            self.lower_controller(self, point)
            break
        print("Arrived at all steps")

        # Initializing the self.set_point_list
        self.set_point_list = []


        
class PoiLen:
    def __init__(self):
        self.poi_len = 0
        rospy.Subscriber('/red/poi', poi, self.poi_callback)

    def poi_callback(self, msg):
            self.poi_len = len(msg.poi)
    
    def poi_return(self):
        return self.poi_len



if __name__ == '__main__':
    rospy.init_node('path', anonymous=True)
    Poi_len = PoiLen()
    while True:
        poi_len = Poi_len.poi_return()
        if (poi_len != 0):
            break

    path = Path(poi_len)
    planning_flag = True
    
    # save home point
    path.set_home()

    # yaw for slam
    path.yaw_circle()

    # set closest poi to target poi
    path.set_target_poi()
    

    while not rospy.is_shutdown():

        path.is_arrived_at_poi()
        path.set_set_point(planning_flag)
        
        # check whether next move is poi index
        planning_flag = path.check_poi_arrived()
        if planning_flag:  

            # control input and wait to arrrived
            path.upper_controller() # publishing
            
        else:
            # Make trajectory with adding step
            continue
        
        

