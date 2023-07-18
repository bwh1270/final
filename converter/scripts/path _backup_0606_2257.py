2#!/usr/bin/env python

import time
import numpy as np
import threading
import rospy
import math
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, Int16
from std_msgs.msg import UInt32MultiArray, UInt8MultiArray
from icuas23_competition.msg import poi

class Path:
    def __init__(self):
        
        
        self.absolute_location = np.array([0, 0, 0]) # in meters
        self.current_idx = np.array([0, 0, 0])
        self.absolute_orientation = np.array([0, 0, 0, 0])
        self.set_point = 0
        self.cmd_x = 0
        self.cmd_y = 0
        self.cmd_z = 0
        self.cmd_yaw = 0
        self.current_score = 100
        self.total_poi = np.array([]) # total list of poi to go (do not change)
        self.poi_to_go = np.array([]) # poi left to go (changes)
        self.target_poi = np.array([]) # current poi to go (single poi in meters)
        self.target_poi_idx = np.array([]) # current poi to go (single poi in array)
        self.home_check = False
        self.home = np.array([0, 0, 0])  # home poi (single poi in meters)
        self.home_idx = np.array([0, 0, 0]) # home poi (single poi in array)
        self.need_slam_flag = False
        self.poi_arrived = True
        self.time_limit = 0
        self.target_number = 0

        self.control_x = 0
        self.control_y = 0
        self.control_z = 0

        # Timer return to home
        self.rth_timer_start = time.time()
        self.rth_time_limit_min = 7 # min
        self.rth_time_limit = self.rth_time_limit_min * 60
        self.is_rth = False
        self.absolute_yaw = 0
        self.threshold_yaw = math.pi / 36

        self.time_break_threshold = 20 # s
        self.time_start_code = time.time()
       
        # Score map Resolution
        self.score_map_x = int(21)
        self.score_map_y = int(31)
        self.score_map_z = int(21)

        # Slam map initialize
        self.slam_map = np.zeros((self.score_map_x, self.score_map_y, self.score_map_z), dtype=np.uint32) # array of 21 x 31 x 21
        
        # Subscriber
        rospy.Subscriber('/carrot_team/voxel_grid_map_32', UInt32MultiArray, self.slam_map_callback)
        rospy.Subscriber('/carrot/pose', PoseStamped, self.absolute_location_callback)
        rospy.Subscriber('/poi', poi, self.total_poi_callback)
        
        # PUblisher
        self.pose_publisher = rospy.Publisher('/carrot_team/red/tracker/input_pose', PoseStamped, queue_size=10)
        self.pub_image = rospy.Publisher('/carrot_team/poi_idx', Int16, queue_size=10)
        
        # Subscribe each score map
        self.score_map = {}
        self.score_map[0] = np.ones((self.score_map_x, self.score_map_y, self.score_map_z), dtype=np.uint32)*7654321 # array of 21 x 31 x 21
        # self.score_map[1] = np.ones((self.score_map_x, self.score_map_y, self.score_map_z), dtype=np.unit32)*7654321 # array of 21 x 31 x 21
        # self.score_map[2] = np.ones((self.score_map_x, self.score_map_y, self.score_map_z), dtype=np.unit32)*7654321 # array of 21 x 31 x 21
        # self.score_map[3] = np.ones((self.score_map_x, self.score_map_y, self.score_map_z), dtype=np.unit32)*7654321 # array of 21 x 31 x 21
        # self.score_map[4] = np.ones((self.score_map_x, self.score_map_y, self.score_map_z), dtype=np.unit32)*7654321 # array of 21 x 31 x 21
        # self.score_map[5] = np.ones((self.score_map_x, self.score_map_y, self.score_map_z), dtype=np.unit32)*7654321 # array of 21 x 31 x 21
        # self.score_map[6] = np.ones((self.score_map_x, self.score_map_y, self.score_map_z), dtype=np.unit32)*7654321 # array of 21 x 31 x 21


        rospy.Subscriber('/carrot_team/target1', UInt32MultiArray, self.score_map_callback1)
        # rospy.Subscriber('/carrot_team/target2', UInt32MultiArray, self.score_map_callback2)
        # rospy.Subscriber('/carrot_team/target3', UInt32MultiArray, self.score_map_callback3)
        # rospy.Subscriber('/carrot_team/target4', UInt32MultiArray, self.score_map_callback4)
        # rospy.Subscriber('/carrot_team/target5', UInt32MultiArray, self.score_map_callback5)
        # rospy.Subscriber('/carrot_team/target6', UInt32MultiArray, self.score_map_callback6)
        # rospy.Subscriber('/carrot_team/target7', UInt32MultiArray, self.score_map_callback7)


        t_rth = threading.Thread(target = self.rth_timer_func)
        t_control_drone = threading.Thread(target = self.control_drone)
        t_rth.start()
        t_control_drone.start()
        
        
    
    



    def score_map_callback1(self, msg):
        print("Score map shape : ", np.shape(msg.data))
        self.score_map[0] = np.reshape(msg.data, (self.score_map_x, self.score_map_y, self.score_map_z))
    # def score_map_callback2(self, msg):
    #     self.score_map[1] = np.reshape(msg.data, (self.score_map_x, self.score_map_y, self.score_map_z))
    # def score_map_callback3(self, msg):
    #     self.score_map[2] = np.reshape(msg.data, (self.score_map_x, self.score_map_y, self.score_map_z))
    # def score_map_callback4(self, msg):
    #     self.score_map[3] = np.reshape(msg.data, (self.score_map_x, self.score_map_y, self.score_map_z))
    # def score_map_callback5(self, msg):
    #     self.score_map[4] = np.reshape(msg.data, (self.score_map_x, self.score_map_y, self.score_map_z))
    # def score_map_callback6(self, msg):
    #     self.score_map[5] = np.reshape(msg.data, (self.score_map_x, self.score_map_y, self.score_map_z))
    # def score_map_callback7(self, msg):
    #     self.score_map[6] = np.reshape(msg.data, (self.score_map_x, self.score_map_y, self.score_map_z))


    def slam_map_callback(self, msg):
        print(time.time() - self.time_start_code)
        print("Slam map shape : ", np.shape(msg.data))
        self.slam_map = np.reshape(msg.data, (self.score_map_x, self.score_map_y, self.score_map_z))
        
    def quaternion2euler(x, y, z, w):
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

    def absolute_location_callback(self, msg):
        xyz_tmp = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        or_tmp = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.z])
        
        
        if self.home_check == False:
            self.home = xyz_tmp
            self.home_idx = np.array([np.round(self.home[0] * 2), np.round(self.home[1] * 2), np.round(self.home[2] * 2)])
            self.home_check = True
    
        check = False
        for i in range(3):
            if self.absolute_location[i] != xyz_tmp[i]:
                check = True

        if check == True:
            self.time_is_map = time.time()

        self.absolute_location = xyz_tmp
        x, y, z = self.absolute_location
        self.current_idx = np.array([np.round(x * 2), np.round(y * 2), np.round(z * 2)])
        self.absolute_orientation = or_tmp
        # quaternion to euler
        self.absolute_yaw = self.quaternion2euler(self.absolute_orientation)

    '''
    quaternion to euler
    # # 쿼터니언을 오일러로 바꾸는 함수
    # def Quaternion2EulerAngles(w, x, y, z):
    #     PIby2 = (math.pi / 2)
    
    #     # roll : rotation in x axis
    #     roll_x = float(2 * (w * x + y * z))
    #     roll_y = float(1 - 2 * (x * x + y * y))
    #     roll = math.atan2(roll_x, roll_y);
    
    #     # pitch : rotation in y axis
    #     pitch_var = float(2 * (w * y - z * x))
    #     if (abs(pitch_var) >= 1):
    # 	#In the event of out of range -> use 90 degrees
    #         pitch = math.copysign(PIby2, pitch_var);
    #     else:
    #         pitch = math.asin(pitch_var);
    
    #     # yaw : rotation in z-axis
    #     yaw_x = 2 * (w * z + x * y);
    #     yaw_y = 1 - 2 * (y * y + z * z);
    #     yaw = math.atan2(yaw_x, yaw_y);
    
    #     # deg로 변환 - 주석하면 radian
    #     # roll  = math.degrees(roll)
    #     # pitch = math.degrees(pitch)
    #     # yaw   = math.degrees(yaw)
    
    #     return roll, pitch, yaw
    
    
    '''
    
    def total_poi_callback(self, msg):
        
        for i in range(len(msg.poi)):
            tmp = np.array([])
            tmp = np.append(tmp, msg.poi[i].x)
            tmp = np.append(tmp, msg.poi[i].y)
            tmp = np.append(tmp, msg.poi[i].z)
        
            self.total_poi = np.append(self.total_poi, tmp)
            
        self.total_poi = np.reshape(self.total_poi, (-1,3))
        self.poi_to_go = self.total_poi

    def set_target_poi(self):
        
        dist = np.array([])
        idx = np.arange(len(self.poi_to_go))

        for poi_tmp in self.poi_to_go:
            poi_x, poi_y, _ = poi_tmp
            dist = np.append(dist, abs(np.linalg.norm(np.array([poi_x, poi_y]) - np.array([self.absolute_location[0], self.absolute_location[1]]))))
        
        idx = idx[dist.argsort()]
        
        if len(idx) > 0:
            self.target_number = idx[0]

            self.target_poi = self.poi_to_go[idx[0]]
            self.target_poi_idx = np.array([np.round(self.target_poi[0] * 2), np.round(self.target_poi[1] * 2), np.round(self.target_poi[2] * 2)])
            self.poi_to_go = np.delete(self.poi_to_go, idx[0])
        else:
            print("Error! No POI")

    def yaw_circle(self):
        '''
        yaw in a circle for mapping
        '''

        # current orientation
        # self.absolute_orientation = 
        # self.absolute_yaw = 

        # Set yaw to 0, pi/2, pi, 3pi/2
        PI = math.pi
        for i in range(8):
            target_yaw = i * PI / 4
            self.cmd_yaw = target_yaw

            time_break_while = time.time()
            # Wait until yaw ends
            while abs(self.absolute_yaw - target_yaw) > self.threshold_yaw:
                # Set orientation of Drone
                self.cmd_yaw = target_yaw
                # pass
                
                if time.time() - time_break_while > self.time_break_threshold:
                    for i in range(10):
                        print("Error yaw time out")
                    break

            
            # Send image to Yolo
            is_yolo = Int16()
            is_yolo.data = 1
            self.pub_image.publish(is_yolo)
            time.sleep(2) # For safety


    def set_set_point(self):
        curr_x, curr_y, curr_z = self.current_idx
        poi_x , poi_y, poi_z = self.target_poi_idx
        score_map_segment = self.score_map[self.target_number][curr_x - 1 : curr_x + 2, curr_y - 1 : curr_y + 2, curr_z - 1 : curr_z + 2]

        min_score = np.min(score_map_segment)
        self.current_score = min_score

        segment_idx = np.where(score_map_segment == min_score)

        dist = np.array([])

        for i in range(len(segment_idx[0])):
            temp = abs(np.linalg.norm(np.array([poi_x, poi_y, poi_z]) - np.array([segment_idx[0][i] + curr_x - 1, segment_idx[1][i] + curr_x - 1, segment_idx[2][i] + curr_x - 1])))
            dist = np.append(dist, temp)
        
        min_idx = np.argmin(dist)

        set_point_x = segment_idx[0][min_idx] + curr_x - 1
        set_point_y = segment_idx[1][min_idx] + curr_y - 1
        set_point_z = segment_idx[2][min_idx] + curr_z - 1

        # set yaw angle, +x = 0deg
        if set_point_x - curr_x == 1:

            if set_point_y - curr_y == 0:
                yaw_deg = math.radians(0)
            elif set_point_y - curr_y == 1:
                yaw_deg = math.radians(-45)
            elif set_point_y - curr_y == -1:
                yaw_deg = math.radians(45)
        
        elif set_point_x - curr_x == 0:

            if set_point_y - curr_y == 1:
                yaw_deg = math.radians(-90)
            elif set_point_y - curr_y == -1:
                yaw_deg = math.radians(90)

        elif set_point_x - curr_x == -1:

            if set_point_y - curr_y == 0:
                yaw_deg = math.radians(180)
            elif set_point_y - curr_y == 1:
                yaw_deg = math.radians(-135)
            elif set_point_y - curr_y == -1:
                yaw_deg = math.radians(135)

        # check if the set_point is mapped with SLAM

        if self.slam_map[set_point_x, set_point_y, set_point_z] == 1:
            self.need_slam_flag = False
        
        elif self.slam_map[set_point_x, set_point_y, set_point_z] == 0:
            self.need_slam_flag = True

        self.cmd_x = set_point_x / 2
        self.cmd_y = set_point_y / 2
        self.cmd_z = set_point_z / 2
        self.cmd_yaw = yaw_deg
        
    
    def final_move_to_poi(self):
        
        return self.target_poi[0], self.target_poi[1], self.target_poi[2]
    
    def check_poi_arrived(self):

        if self.current_idx == self.target_poi_idx:
            self.poi_arrived = True
        else:
            self.poi_arrived = False

    
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

    def control_drone(self):
        pose_msg = PoseStamped()

        # pose_msg.header.frame_id = ""  # set the frame ID
        pose_msg.pose.position.x = 0.0  # set the x position
        pose_msg.pose.position.y = 0.0  # set the y position 
        pose_msg.pose.position.z = 0.0  # set the z position
        pose_msg.pose.orientation.x = 0.0  # set the x orientation
        pose_msg.pose.orientation.y = 0.0  # set the y orientation
        pose_msg.pose.orientation.z = 0.0  # set the z orientation
        pose_msg.pose.orientation.w = 1.0  # set the w orientation

        while True:

            q = self.get_quaternion_from_euler(0, 0, self.cmd_yaw)

            pose_msg.pose.orientation.x = q[0]  # set the x orientation
            pose_msg.pose.orientation.y = q[1]  # set the y orientation
            pose_msg.pose.orientation.z = q[2]  # set the z orientation
            pose_msg.pose.orientation.w = q[3]  # set the w orientation

            pose_msg.pose.position.x = self.control_x  # set the x position
            pose_msg.pose.position.y = self.control_y  # set the y position
            pose_msg.pose.position.z = self.control_z  # set the z position

            self.pose_publisher.publish(pose_msg)

            time.sleep(0.1)

    def rth_timer_func(self):
        while not self.is_rth:
            if time.time() - self.rth_timer_start > self.rth_time_limit:
                self.is_rth = True
                self.target_poi = self.home
                self.target_poi_idx = self.home_idx

                break

if __name__ == '__main__':
    rospy.init_node('path', anonymous=True)
    path = Path()

    # pose_publisher = rospy.Publisher('/red/tracker/input_pose', PoseStamped, queue_size=10)
    # pose_msg = PoseStamped()
    # # pose_msg.header.frame_id = ""  # set the frame ID
    # pose_msg.pose.position.x = 0.0  # set the x position
    # pose_msg.pose.position.y = 0.0  # set the y position 
    # pose_msg.pose.position.z = 0.0  # set the z position
    # pose_msg.pose.orientation.x = 0.0  # set the x orientation
    # pose_msg.pose.orientation.y = 0.0  # set the y orientation
    # pose_msg.pose.orientation.z = 0.0  # set the z orientation
    # pose_msg.pose.orientation.w = 1.0  # set the w orientation


    #t = threading.Thread(target = path.rospy_spin)
    #t.start()
    
    while not rospy.is_shutdown():

        if path.poi_arrived == True and path.is_rth == False:
            path.set_target_poi()
            path.yaw_circle()

        elif path.poi_arrived == False:
            path.set_set_point()

            if path.need_slam_flag == False:

                # q = path.get_quaternion_from_euler(0, 0, path.cmd_yaw)

                # pose_msg.pose.orientation.x = q[0]  # set the x orientation
                # pose_msg.pose.orientation.y = q[1]  # set the y orientation
                # pose_msg.pose.orientation.z = q[2]  # set the z orientation
                # pose_msg.pose.orientation.w = q[3]  # set the w orientation

                # pose_msg.pose.position.x = path.cmd_x  # set the x position
                # pose_msg.pose.position.y = path.cmd_y  # set the y position
                # pose_msg.pose.position.z = path.cmd_z  # set the z position
                
                path.control_x = path.cmd_x
                path.control_y = path.cmd_y
                path.control_z = path.cmd_z

            elif path.need_slam_flag == True:

                # q = path.get_quaternion_from_euler(0, 0, path.cmd_yaw)
                # pose_msg.pose.orientation.x = q[0]  # set the x orientation
                # pose_msg.pose.orientation.y = q[1]  # set the y orientation
                # pose_msg.pose.orientation.z = q[2]  # set the z orientation
                # pose_msg.pose.orientation.w = q[3]  # set the w orientation
                
                # pose_msg.pose.position.x = path.current_idx[0] / 2  # set the x position
                # pose_msg.pose.position.y = path.current_idx[1] / 2  # set the y position
                # pose_msg.pose.position.z = path.current_idx[2] / 2  # set the z position

                path.control_x = path.current_idx[0] / 2  # set the x position
                path.control_y = path.current_idx[1] / 2  # set the x position
                path.control_z = path.current_idx[2] / 2  # set the x position

        path.check_poi_arrived()
        
        # Need to add the publishing of x, y, z
        # Need to add return to home if time limit
        # Need to add path.yaw_circle()
