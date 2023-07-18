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
    def __init__(self):
        self.scale = int(1 / 0.2) # scale of 0.2m
        self.absolute_location = np.array([0, 0, 0]) # in meters
        self.current_idx = np.array([0, 0])
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
        self.home_idx = np.array([0, 0]) # home poi (single poi in array)
        self.need_slam_flag = False
        self.poi_arrived = False
        self.time_limit = 0
        self.target_number = 0
        self.poi_check = False

        self.control_x = 0
        self.control_y = 0
        self.control_z = 1
        
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

       
        # Score map Resolution
        self.score_map_x = int(self.map_x * self.scale + 1) # 10m x scale factor + 1
        self.score_map_y = int(self.map_y * self.scale + 1) # 12m x scale factor + 1

        # Slam map initialize
        self.slam_map = np.zeros((self.score_map_x, self.score_map_y), dtype=np.uint32) # array of 10 * scale + 1 x 12 * scale + 1

        # Check if drone location is recieved
        self.is_drone_location = False

        # Is point arrived
        self.is_point_arrived = False
        
        # Subscriber
        rospy.Subscriber('/carrot_team/voxel_grid_map_32', UInt32MultiArray, self.slam_map_callback)
        rospy.Subscriber('/falconblack/vrpn_client/estimated_odometry', Odometry, self.absolute_location_callback)
        rospy.Subscriber('/red/poi', poi, self.total_poi_callback)
        
        # Publisher
        self.pose_publisher = rospy.Publisher('/red/tracker/input_pose', PoseStamped, queue_size=10)
        self.pub_image = rospy.Publisher('/carrot_team/poi_idx', Int16, queue_size=10)
        
        # Subscribe each score map
        self.score_map = {}


        self.score_map[0] = np.ones((self.score_map_x, self.score_map_y), dtype=np.uint32)*7654321 # array of 10 * scale + 1 x 12 * scale + 1
        
        #self.score_map[1] = np.ones((self.score_map_x, self.score_map_y), dtype=np.uint32)*7654321 # array of 10 * scale + 1 x 12 * scale + 1
        #self.score_map[2] = np.ones((self.score_map_x, self.score_map_y), dtype=np.uint32)*7654321 # array of 10 * scale + 1 x 12 * scale + 1
        #self.score_map[3] = np.ones((self.score_map_x, self.score_map_y), dtype=np.uint32)*7654321 # array of 10 * scale + 1 x 12 * scale + 1
        #self.score_map[4] = np.ones((self.score_map_x, self.score_map_y), dtype=np.uint32)*7654321 # array of 10 * scale + 1 x 12 * scale + 1
        #self.score_map[5] = np.ones((self.score_map_x, self.score_map_y), dtype=np.uint32)*7654321 # array of 10 * scale + 1 x 12 * scale + 1
        # self.score_map[6] = np.ones((self.score_map_x, self.score_map_y), dtype=np.uint32)*7654321 # array of 10 * scale + 1 x 12 * scale + 1


        rospy.Subscriber('/carrot_team/target0', UInt32MultiArray, self.score_map_callback0)
        #rospy.Subscriber('/carrot_team/target1', UInt32MultiArray, self.score_map_callback1)
        #rospy.Subscriber('/carrot_team/target2', UInt32MultiArray, self.score_map_callback2)
        #rospy.Subscriber('/carrot_team/target3', UInt32MultiArray, self.score_map_callback3)
        #rospy.Subscriber('/carrot_team/target4', UInt32MultiArray, self.score_map_callback4)
        #rospy.Subscriber('/carrot_team/target5', UInt32MultiArray, self.score_map_callback5)
        # rospy.Subscriber('/carrot_team/target6', UInt32MultiArray, self.score_map_callback6)

        # Drone controller init function
        self.pose_msg = PoseStamped()
        self.control_drone_init()

        t_rth = threading.Thread(target = self.rth_timer_func)
        t_check_point_arrived = threading.Thread(target = self.check_point_arrrived)
        t_rth.start()
        t_check_point_arrived.start()
        


    def score_map_callback0(self, msg):
        print("Score map shape : ", np.shape(msg.data))
        self.score_map[0] = np.reshape(msg.data, (self.score_map_x, self.score_map_y))
    """
    def score_map_callback1(self, msg):
        self.score_map[1] = np.reshape(msg.data, (self.score_map_x, self.score_map_y))
    def score_map_callback2(self, msg):
        self.score_map[2] = np.reshape(msg.data, (self.score_map_x, self.score_map_y))
    def score_map_callback3(self, msg):
        self.score_map[3] = np.reshape(msg.data, (self.score_map_x, self.score_map_y))
    def score_map_callback4(self, msg):
        self.score_map[4] = np.reshape(msg.data, (self.score_map_x, self.score_map_y))
    def score_map_callback5(self, msg):
        self.score_map[5] = np.reshape(msg.data, (self.score_map_x, self.score_map_y))
    # def score_map_callback6(self, msg):
    #     self.score_map[6] = np.reshape(msg.data, (self.score_map_x, self.score_map_y))
    """

    def slam_map_callback(self, msg):
        print(time.time() - self.time_start_code)
        print("Slam map shape : ", np.shape(msg.data))
        self.slam_map = np.reshape(msg.data, (self.score_map_x, self.score_map_y))
        
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

    def absolute_location_callback(self, msg):
        xyz_tmp = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        or_tmp = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.z])
        
        
        if self.home_check == False:
            self.home = xyz_tmp
            self.home_idx = np.array([np.round(self.home[0] * self.scale), np.round(self.home[1] * self.scale)])
            self.home_check = True
    
        check = False
        for i in range(3):
            if self.absolute_location[i] != xyz_tmp[i]:
                check = True

        if check == True:
            self.time_is_map = time.time()

        self.absolute_location = xyz_tmp
        x, y, z = self.absolute_location
        self.current_idx = np.array([np.round(x * self.scale), np.round(y * self.scale)])
        self.absolute_orientation = or_tmp
        # quaternion to euler
        self.absolute_yaw = self.quaternion2euler(self.absolute_orientation)

        self.is_drone_location = True

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
            print(tmp)
        
            self.total_poi = np.append(self.total_poi, tmp)
            
        self.total_poi = np.reshape(self.total_poi, (-1,3))
        
        if self.poi_check == False:
            self.poi_to_go = self.total_poi
            self.poi_check = True

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
            self.target_poi_idx = np.array([np.round(self.target_poi[0] * self.scale), np.round(self.target_poi[1] * self.scale)])
            self.poi_to_go = np.delete(self.poi_to_go, idx[0])
        else:
            print("Error! No POI")

    def yaw_circle_init(self):
        resolution_yaw = 100
        total_yaw_time = 10 # s
    
        time_wait = total_yaw_time / resolution_yaw
        if time_wait <= 0:
            print("Error! wrong time")
            time_wait = 0.01

        # Set yaw
        PI = math.pi
        for i in range(resolution_yaw):
            target_yaw = (2 * PI * i) / resolution_yaw
            self.cmd_yaw = target_yaw
            
            time.sleep(time_wait)

        self.set_yaw(self.cmd_yaw)
        self.pose_publisher.publish(self.pose_msg)
        
    def yaw_circle(self): # Need to edit !! - Only check walls
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
                
                self.set_yaw(self.cmd_yaw)
                self.pose_publisher.publish(self.pose_msg)
                # pass
                
                if time.time() - time_break_while > self.time_break_threshold:
                    for i in range(10):
                        print("Error yaw time out")
                    break

            
            # Send image to Yolo
            is_yolo = Int16()
            is_yolo.data = 1
            self.pub_image.publish(is_yolo)
            time.sleep(1.5) # For safety


    def set_set_point(self):
        curr_x, curr_y = self.current_idx
        poi_x , poi_y = self.target_poi_idx
        score_map_segment = self.score_map[self.target_number][curr_x - 1 : curr_x + 2, curr_y - 1 : curr_y + 2]

        min_score = np.min(score_map_segment)
        self.current_score = min_score

        segment_idx = np.where(score_map_segment == min_score)

        dist = np.array([])

        for i in range(len(segment_idx[0])):
            temp = abs(np.linalg.norm(np.array([poi_x, poi_y]) - np.array([segment_idx[0][i] + curr_x - 1, segment_idx[1][i] + curr_y - 1])))
            dist = np.append(dist, temp)
        
        min_idx = np.argmin(dist)

        set_point_x = segment_idx[0][min_idx] + curr_x - 1
        set_point_y = segment_idx[1][min_idx] + curr_y - 1

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

        if self.slam_map[set_point_x, set_point_y] == 1:
            self.need_slam_flag = False
        
        elif self.slam_map[set_point_x, set_point_y] == 0:
            self.need_slam_flag = True

        self.cmd_x = (set_point_x / self.scale) - (self.map_x / 2)
        self.cmd_y = (set_point_y / self.scale) - (self.map_y / 2)
        # self.cmd_z = self.target_poi[2]
        self.cmd_yaw = yaw_deg
        
    
    def final_move_to_poi(self):
        self.cmd_x = self.target_poi[0]
        self.cmd_y = self.target_poi[1]
        # self.cmd_z = self.target_poi[2]
    
    def check_poi_arrived(self):
        poi_arrived = True
        for i in range(len(self.absolute_location)):
            if abs(self.absolute_location[i] - self.target_poi[i]) > self.check_poi_threshold:
                poi_arrived = False
        
        self.poi_arrived = poi_arrived

        # if self.current_idx == self.target_poi_idx:
        #     self.poi_arrived = True
        # else:
        #     self.poi_arrived = False

    def check_point_arrrived(self):
        abs_location = self.absolute_location
        control_input = [self.pose_msg.pose.position.x, pose_msg.pose.position.y]

        is_moved = True
        for i in range(2):
            if abs(self.absolute_location[i] - self.target_poi[i]) > self.check_poi_threshold:
                is_moved = False

        self.is_point_arrived = is_moved

    
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

    def control_drone_init(self):
        # pose_msg.header.frame_id = ""  # set the frame ID - We don't need this
        self.pose_msg.pose.position.x = 0.0  # set the x position
        self.pose_msg.pose.position.y = 0.0  # set the y position 
        self.pose_msg.pose.position.z = 1.0  # set the z position
        self.pose_msg.pose.orientation.x = 0.0  # set the x orientation
        self.pose_msg.pose.orientation.y = 0.0  # set the y orientation
        self.pose_msg.pose.orientation.z = 0.0  # set the z orientation
        self.pose_msg.pose.orientation.w = 1.0  # set the w orientation

        start_timer_drone_location = time.time()
        print("Waiting for drone location; controller")
        while not self.is_drone_location:
            if time.time() - start_timer_drone_location > 10:
                print("Error! No drone location")
                time.sleep(0.1)
        print("Drone location recieved")

        self.control_x = self.absolute_location[0]
        self.control_y = self.absolute_location[1]
        self.control_z = self.home[2]

        # while True:
        #     q = self.get_quaternion_from_euler(0, 0, self.cmd_yaw)

        #     pose_msg.pose.orientation.x = q[0]  # set the x orientation
        #     pose_msg.pose.orientation.y = q[1]  # set the y orientation
        #     pose_msg.pose.orientation.z = q[2]  # set the z orientation
        #     pose_msg.pose.orientation.w = q[3]  # set the w orientation

        #     pose_msg.pose.position.x = self.control_x  # set the x position
        #     pose_msg.pose.position.y = self.control_y  # set the y position
        #     pose_msg.pose.position.z = self.control_z  # set the z position

        #     self.pose_publisher.publish(self.pose_msg)

        #     time.sleep(2)
    
    def set_yaw(self, yaw):
        q = self.get_quaternion_from_euler(0, 0, yaw)

        self.pose_msg.pose.orientation.x = q[0]  # set the x orientation
        self.pose_msg.pose.orientation.y = q[1]  # set the y orientation
        self.pose_msg.pose.orientation.z = q[2]  # set the z orientation
        self.pose_msg.pose.orientation.w = q[3]  # set the w orientation
        

    def rth_timer_func(self):
        while not self.is_rth:
            if time.time() - self.rth_timer_start > self.rth_time_limit:
                self.is_rth = True
                self.target_poi = self.home
                self.target_poi_idx = self.home_idx

                break
    
    def return_home(self):
        self.target_poi = self.home
        self.target_poi_idx = self.home_idx

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
    
    # Check SLAM ready # !!
    # pass

    # Circle when takeoff
    path.yaw_circle_init()
    
    # Check POI # !!
    # pass

    path.set_target_poi()
    
    while not rospy.is_shutdown():
        # Not rth
        if path.is_rth == False:
            if path.poi_arrived == True:
                path.final_move_to_poi()
                path.set_target_poi()
                path.yaw_circle()

            # elif path.poi_arrived == False:
            else:
                path.set_set_point()

                # Move when mapped
                if path.need_slam_flag == False:
                    path.pose_msg.position.x = path.cmd_x
                    path.pose_msg.position.y = path.cmd_y
                    path.set_yaw(path.cmd_yaw)
                    # path.control_x = path.cmd_x
                    # path.control_y = path.cmd_y
                    # path.control_z = path.cmd_z

                # elif path.need_slam_flag == True:
                else:
                    path.pose_msg.position.x = path.current_idx[0] / path.scale  # set the x position
                    path.pose_msg.position.y = path.current_idx[1] / path.scale  # set the y position
                    path.set_yaw(path.cmd_yaw)
                    # path.control_z = path.absolute_location[2]  # set the z position
                
                # Publish !!
                path.pose_publisher.publish(path.pose_msg)
        else:
            path.return_home()
            path.set_set_point()
            
            # Move when mapped
            if path.need_slam_flag == False:
                path.pose_msg.position.x = path.cmd_x
                path.pose_msg.position.y = path.cmd_y
                path.set_yaw(path.cmd_yaw)
                # path.control_z = path.cmd_z

            # Yaw when not mapped
            # elif path.need_slam_flag == True:
            else:
                path.pose_msg.position.x = path.current_idx[0] / path.scale  # set the x position
                path.pose_msg.position.y = path.current_idx[1] / path.scale  # set the x position
                path.set_yaw(path.cmd_yaw)
                # path.control_z = path.absolute_location[2]  # set the x position
                # In case of infinitive loop

            # Publish !!
            path.pose_publisher.publish(path.pose_msg)
            
            # else:
            #     print("Error")


            # If end, hovering and print message
            if path.poi_arrived == True:
                print("RTH End")

        time_check = time.time()
        while not path.is_point_arrived:
            pass
            if time.time() - time_check > 5:
                print("Too long time")
                break

        path.check_poi_arrived()
        
        # Need to add the publishing of x, y, z
        # Need to add return to home if time limit
        # Need to add path.yaw_circle()
