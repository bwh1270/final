#!/usr/bin/env python

import time
import numpy as np
import threading
import rospy
import math
from geometry_msgs.msg import PoseStamped

class DummyController:
    def __init__(self):
        print("Test control code")
        self.points = np.array([])
        self.points = np.append(self.points, [0,0,1])
        self.points = np.append(self.points, [1,0,1])
        self.points = np.append(self.points, [1,1,1])
        self.points = np.append(self.points, [0,1,1])
        self.home_check = False

        self.absolute_location = np.array([0, 0, 0]) # in meters
        self.is_location = False
        self.current_idx = np.array([0, 0, 0])
        self.absolute_orientation = np.array([0, 0, 0, 0])
        self.cmd_x = 0
        self.cmd_y = 0
        self.cmd_z = 0
        self.cmd_yaw = 0

        self.threshold = 0.25
        
        # Subscriber
        rospy.Subscriber('/red/carrot/pose', PoseStamped, self.absolute_location_callback)
        
        # PUblisher
        self.pose_publisher = rospy.Publisher('/red/tracker/input_pose', PoseStamped, queue_size=10)          

        # Control Drone
        t_control_drone = threading.Thread(target = self.control_drone)
        t_control_drone.start()
    
    def quaternion2euler(self, orientation):
        # yaw : rotation in z-axis
        x, y, z, w = orientation
        yaw_x = 2 * (w * z + x * y);
        yaw_y = 1 - 2 * (y * y + z * z);
        yaw = math.atan2(yaw_x, yaw_y);
        
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
        
        self.is_location = True
    
    def set_set_point(self, where_to_go):
        time_start_drone = time.time()
        self.cmd_yaw = 0

        while True:
            # is_moved = True
            # curr_x, curr_y, curr_z = self.current_idx
            # if abs(curr_x - where_to_go[0]) > self.threshold:   
            #     is_moved = False
            # if abs(curr_y - where_to_go[1]) > self.threshold:   
            #     is_moved = False
            # if abs(curr_z - where_to_go[2]) > self.threshold:   
            #     is_moved = False
            
            # if is_moved == False:
            #     self.control_x = where_to_go[0]
            #     self.control_y = where_to_go[1]
            #     self.control_z = where_to_go[2]
            where_to_go = input('WASD')
            x, y, z = self.absolute_location
            print(self.absolute_location)
            if where_to_go == 'w' or where_to_go == 'W': # y + 1
                self.control_x = x
                self.control_y = y + 0.5
                self.control_z = z
            elif where_to_go == 'a' or where_to_go == 'A': # y + 1
                self.control_x = x - 0.5
                self.control_y = y
                self.control_z = z
            elif where_to_go == 's' or where_to_go == 'S':
                self.control_x = x
                self.control_y = y - 0.5
                self.control_z = z
            elif where_to_go == 'd' or where_to_go == 'D':
                self.control_x = x + 0.5
                self.control_y = y
                self.control_z = z
            else:
                print("Enter WASD")
                
    
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

        time_wait_location = time.time()
        print("Waiting for drone location")
        while not self.is_location:
            if time.time() - time_wait_location > 10:
                print("Error! Drone location Error!")
            # pass
        print("Drone location recieved")

        print("Location :", self.absolute_location)
        print("Orientation :",self.absolute_orientation)

        x, y, z = self.absolute_location
        ori_x, ori_y, ori_z, ori_w = self.absolute_orientation

        # pose_msg.header.frame_id = ""  # set the frame ID
        pose_msg.pose.position.x = x  # set the x position
        pose_msg.pose.position.y = y  # set the y position 
        pose_msg.pose.position.z = z  # set the z position
        pose_msg.pose.orientation.x = ori_x  # set the x orientation
        pose_msg.pose.orientation.y = ori_y  # set the y orientation
        pose_msg.pose.orientation.z = ori_z  # set the z orientation
        pose_msg.pose.orientation.w = ori_w  # set the w orientation

        self.control_x = x
        self.control_y = y
        self.control_z = z
        
        while True:
            q = self.get_quaternion_from_euler(0, 0, self.cmd_yaw)

            pose_msg.pose.orientation.x = q[0]  # set the x orientation
            pose_msg.pose.orientation.y = q[1]  # set the y orientation
            pose_msg.pose.orientation.z = q[2]  # set the z orientation
            pose_msg.pose.orientation.w = q[3]  # set the w orientation

            pose_msg.pose.position.x = self.control_x  # set the x position
            pose_msg.pose.position.y = self.control_y  # set the y position
            pose_msg.pose.position.z = self.control_z  # set the z position
            print(pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z)
            self.pose_publisher.publish(pose_msg)

            time.sleep(2)

if __name__ == '__main__':
    rospy.init_node('dummy_controller', anonymous=True)
    dcr = DummyController()

    while not rospy.is_shutdown():
        for i in range(len(dcr.points)):
            dcr.set_set_point(dcr.points[i])
