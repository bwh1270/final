import rospy
from std_msgs.msg import UInt32MultiArray, MultiArrayDimension # Publish score map in 3D Array
from converter.msg import xyz
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
import math
import numpy as np
import time
import threading


class ScoreMap:
    '''
    If map has lots of obstcles,
    change resolution to smaller value
    change maximum distance 1.5 into bigger value

    If SLAM resolution chages, change the value
    '''
    def __init__(self, target_x, target_y, target_z, publisher_name):
        # Node name
        node_name = f'Score_map_node_{publisher_name}'

        rospy.init_node(node_name, anonymous=True)

        # variables
        self.time_start = time.time()
        # uint 32 maximum number : 4294967295
        self.INF = 3987654321 # 3094967290 # !!
        self.NULL = 123456789 # 200000000 # !!
        self.size_x = 10 # x, m  ## 8.6 
        self.size_y = 12 # y, m
        self.size_z = 3 # height, m
        self.is_print = False
        self.is_print_error = True

        # Variable for safety
        self.resolution = 50 # cm
        self.slam_res = self.resolution # cm
        self.DRONE_SIZE = 50 # cm
        self.SAFETY_ALPHA = 15 # cm
        self.time_limit = 20 # s
        self.maximum_time_limit = 200 # s

        self.res_div = 100 // self.resolution
        # print(res_div)
        self.map_size_x = self.size_x * self.res_div + 1
        self.map_size_y = self.size_y * self.res_div + 1
        self.map_size_z = self.size_z * self.res_div + 1
        if self.is_print:
            print("Map size:", self.map_size_x, self.map_size_y, self.map_size_z)

        self.safety_dist = self.slam_res / 2 + self.DRONE_SIZE / 2 + self.SAFETY_ALPHA
        self.safety_block = math.ceil(self.safety_dist / self.resolution)
        if self.is_print:
            print("How many blocks for safety", self.safety_block)
            
        # Subscriber
        rospy.Subscriber('/carrot_team/uav_xyz', Point, self.get_pose)
        rospy.Subscriber('/carrot_team/pcl2xyz', xyz, self.get_wall)

        # Publisher
        self.pub_score_map = rospy.Publisher(f'/carrot_team/{publisher_name}', UInt32MultiArray, queue_size = 10)

        # Target position - Get from POI
        self.target_x = target_x # m 8
        self.target_y = target_y # m 10
        self.target_z = target_z # m 4

        # Target in box
        self.t_box_x = round(self.target_x * self.res_div)
        if self.t_box_x > self.map_size_x:
            self.t_box_x = self.map_size_x
            if self.is_print_error:
                print("Error! Too big; x")
        elif self.t_box_x < 0:
            self.t_box_x = 0
            if self.is_print_error:
                print("Error! Too small; x")

        self.t_box_y = round(self.target_y * self.res_div)
        if self.t_box_y > self.map_size_y:
            self.t_box_y = self.map_size_y
            if self.is_print_error:
                print("Error! Too big; y")
        elif self.t_box_y < 0:
            self.t_box_y = 0
            if self.is_print_error:
                print("Error! Too small; y")
            
        self.t_box_z = round(self.target_z * self.res_div)
        if self.t_box_z > self.map_size_z:
            self.t_box_z = self.map_size_z
            if self.is_print_error:
                print("Error! Too big; z")
        elif self.t_box_z < 0:
            self.t_box_z = 0
            if self.is_print_error:
                print("Error! Too small; z")
        if self.is_print:
            print("Target : ", self.t_box_x, self.t_box_y, self.t_box_z)


        # Drone position - temporary
        self.drone_x = 0 # m
        self.drone_y = 0 # m
        self.drone_z = 1 # m

        self.drone_box_x = round(self.drone_x * self.res_div)
        self.drone_box_y = round(self.drone_y * self.res_div)
        self.drone_box_z = round(self.drone_z * self.res_div)


        # Make score map_init
        self.score_np = np.ones((self.map_size_x, self.map_size_y, self.map_size_z), dtype = np.uint32) * self.NULL
        self.score_np_pub = np.ones((self.map_size_x, self.map_size_y, self.map_size_z), dtype = np.uint32) * self.INF
        self.score_np[self.t_box_x, self.t_box_y, self.t_box_z] = 0
        if self.is_print:
            print("Score of target : ", self.score_np[self.t_box_x, self.t_box_y, self.t_box_z])

        # Data from SLAM - Wall - for test
        self.wall_all = []
        self.wall_x = [] # [10,11,10,10,10,10]
        self.wall_y = [] # [10,11,9,8,7,6]
        self.wall_z = [] # [2,2,2,2,2,2]

        t = threading.Thread(target = self.cal_score_map)
        t_publish = threading.Thread(target = self.pub_score_map_func)
        t.start()
        t_publish.start()

    def cal_score_map(self):
        while not rospy.is_shutdown:
            self.wall_x = self.wall_x_prev
            self.wall_y = self.wall_y_prev
            self.wall_z = self.wall_z_prev

            self.arr_q = [[self.t_box_x, self.t_box_y, self.t_box_z]]
            self.arr_fin = []
            
            # Init score map
            self.score_np = np.ones((self.map_size_x, self.map_size_y, self.map_size_z), dtype = np.uint32) * self.NULL
            self.score_np[self.t_box_x, self.t_box_y, self.t_box_z] = 0
            self.score_np[:,:,0] = self.INF
            self.score_np[:,0,:] = self.INF
            self.score_np[0,:,:] = self.INF
            self.score_np[self.map_size_x - 1,:,:] = self.INF
            self.score_np[:,self.map_size_y - 1,:] = self.INF
            self.score_np[:,:,self.map_size_z - 1] = self.INF
            # Remove corners
            self.score_np[1,1,:] = self.INF
            self.score_np[1, self.map_size_y - 2,:] = self.INF
            self.score_np[self.map_size_x - 2,1,:] = self.INF
            self.score_np[self.map_size_x - 2, self.map_size_y - 2,:] = self.INF

            # Calculate Distance until meeting Drone
            self.time_start = time.time()
            # Set Mapping limit # !!
            self.x_upper_limit = max(self.t_box_x, self.drone_box_x) + self.safety_block + 1
            if self.x_upper_limit > self.map_size_x - 1:
                self.x_upper_limit = self.map_size_x - 1
            # print("x upper limit : ", x_upper_limit)
            self.x_lower_limit = min(self.t_box_x, self.drone_box_x) - self.safety_block - 2
            if self.x_lower_limit < 0:
                self.x_lower_limit = 0

            self.y_upper_limit = max(self.t_box_y, self.drone_box_y) + self.safety_block + 1
            if self.y_upper_limit > self.map_size_y - 1:
                self.y_upper_limit = self.map_size_y - 1
            self.y_lower_limit = min(self.t_box_y, self.drone_box_y) - self.safety_block - 2
            if self.y_lower_limit < 0:
                self.y_lower_limit = 0

            self.z_upper_limit = max(self.t_box_z, self.drone_box_z) + self.safety_block + 1
            if self.z_upper_limit > self.map_size_z - 1:
                self.z_upper_limit = self.map_size_z - 1
            self.z_lower_limit = min(self.t_box_z, self.drone_box_z) - self.safety_block - 2
            if self.z_lower_limit < 0:
                self.z_lower_limit = 0

            if self.is_print:
                print("Upper limit : ", self.x_upper_limit, self.y_upper_limit, self.z_upper_limit)
                print("Lower limit : ", self.x_lower_limit, self.y_lower_limit, self.z_lower_limit)


            # Make wall
            '''
            Get wall data from SLAM and plot them
            '''

            self.time_start = time.time()
            while len(self.wall_x) > 0:
                tmp_x = self.wall_x.pop()
                tmp_y = self.wall_y.pop()
                tmp_z = self.wall_z.pop()

                for i in range(-1 * self.safety_block, self.safety_block + 1):
                    for j in range(-1 * self.safety_block, self.safety_block + 1):
                        for k in range(-1 * self.safety_block, self.safety_block + 1):
                            if tmp_x + i == self.x_lower_limit or tmp_x + i == self.x_upper_limit or \
                            tmp_y + j == self.y_lower_limit or tmp_y + j == self.y_upper_limit or \
                            tmp_z + k == self.z_lower_limit or tmp_z + k == self.z_upper_limit:
                                continue
                            
                            self.score_np[tmp_x + i, tmp_y + j, tmp_z + k] = self.INF

                if time.time() - self.time_start > self.time_limit:
                    if self.is_print_error:
                        print("Too much obstacle; Time out !")
                    break
            if self.is_print:
                print("Obstacle plot time : ", time.time() - self.time_start)
                print("Obstacle plot end")


            # Make Score map
            while len(self.arr_q) > 0:
                tmp_x, tmp_y, tmp_z = self.arr_q.pop(0)
                self.arr_fin.append([tmp_x, tmp_y, tmp_z])

                mini = self.score_np[tmp_x, tmp_y, tmp_z]
                for i in range(-1, 2):
                    if tmp_x + i == self.x_lower_limit or tmp_x + i == self.x_upper_limit:
                        continue 
                    for j in range(-1, 2):
                        if tmp_y + j == self.y_lower_limit or tmp_y + j == self.y_upper_limit:
                            continue
                        for k in range(-1, 2):
                            if tmp_z + k == self.z_lower_limit or tmp_z + k == self.z_upper_limit:
                                continue

                            dist_tmp = self.score_np[tmp_x + i, tmp_y + j, tmp_z + k]
                            if dist_tmp == self.INF:
                                continue
                            
                            dist_tmp += 1
                            if dist_tmp <= mini:
                                mini = dist_tmp
                            else:
                                if [tmp_x + i, tmp_y + j, tmp_z + k] in self.arr_fin or [tmp_x + i, tmp_y + j, tmp_z + k] in self.arr_q:
                                    pass
                                else:
                                    self.arr_q.append([tmp_x + i, tmp_y + j, tmp_z + k])
                
                self.score_np[tmp_x, tmp_y, tmp_z] = mini
                if tmp_x == self.drone_box_x and tmp_y == self.drone_box_y and tmp_z == self.drone_box_z:
                    if self.is_print:
                        print("Meet drone")
                    break

                # Infinitive loop
                if time.time() - self.time_start > self.time_limit:
                    self.time_limit = int(1.1 * self.time_limit)
                    if self.time_limit > self.maximum_time_limit:
                        self.time_limit = self.maximum_time_limit
                    break

            # print(self.score_np[5:20,7:13,2])
            self.score_np_pub = self.score_np * 1

            if self.is_print:
                print("Time : ", time.time() - self.time_start)


    def drone_position(self): # Do every time drone position is recieved.
        # !! Before this, Drone position origin must be aligned.
        # Did not make this function because real environment is unkown. 
        self.drone_box_x = round(self.drone_x * self.res_div)
        if self.drone_box_x > self.map_size_x:
            self.drone_box_x = self.map_size_x
            if self.is_print_error:
                print("Error! Drone position error; x")
        elif self.drone_box_x < 0:
            self.drone_box_x = 0
            if self.is_print_error:
                print("Error! Drone position error (negative value); x")

        self.drone_box_y = round(self.drone_y * self.res_div)
        if self.drone_box_y > self.map_size_y:
            self.drone_box_y = self.map_size_y
            if self.is_print_error:
                print("Error! Drone position error; y")
        elif self.drone_box_y < 0:
            self.drone_box_y = 0
            if self.is_print_error:
                print("Error! Drone position error (negative value); y")

        self.drone_box_z = round(self.drone_z * self.res_div)
        if self.drone_box_z > self.map_size_z:
            self.drone_box_z = self.map_size_z
            if self.is_print_error:
                print("Error! Drone position error; z")
        elif self.drone_box_z < 0:
            self.drone_box_z = 0
            if self.is_print_error:
                print("Error! Drone position error (negative value); z")
        if self.is_print:
            print("Drone position : ", self.drone_box_x, self.drone_box_y, self.drone_box_z)
        
    def pub_score_map_func(self):
        msg = UInt32MultiArray()
        while not rospy.is_shutdown():
            array_flat = np.reshape(self.score_np_pub, -1).astype(np.uint32)
            msg.data = list(array_flat)
            
            dim_height = MultiArrayDimension()
            dim_height.label = "height"
            dim_height.size = len(self.score_np_pub)
            dim_height.stride = len(self.score_np_pub[0]) * len(self.score_np_pub[0][0])
            msg.layout.dim.append(dim_height)

            dim_width = MultiArrayDimension()
            dim_width.label = "width"
            dim_width.size = len(self.score_np_pub[0])
            dim_width.stride = len(self.score_np_pub[0][0])
            msg.layout.dim.append(dim_width)

            dim_depth = MultiArrayDimension()
            dim_depth.label = "depth"
            dim_depth.size = len(self.score_np_pub[0][0])
            dim_depth.stride = 1
            msg.layout.dim.append(dim_depth)
            
            self.pub_score_map.publish(msg)

    def get_pose(self, msg):
        # Edit drone x,y,z into positive value
        self.drone_x = msg.x
        self.drone_y = msg.y
        self.drone_z = msg.z
        
        if msg.x < 0 or msg.y < 0 or msg.z < 0:
            print("Error! Drone index negative number")

        self.drone_position()
    
    

    def get_wall(self, msg):
        # print(msg.xyz)
        # print(len(msg.xyz))
        # print(np.shape(msg.xyz)) # (836, )
        # print(msg.xyz[:].x)
        wall_all = msg.xyz
        
        wall_x_prev = []
        wall_y_prev = []
        wall_z_prev = []
        for tmp in (wall_all):
            wall_x_prev.append(tmp.x)
            wall_y_prev.append(tmp.y)
            wall_z_prev.append(tmp.z)
        
        self.wall_x = wall_x_prev;
        self.wall_y = wall_y_prev;
        self.wall_z = wall_z_prev;
                
        
# ############### code start ###############
# rospy.init_node('pub_score_map')
# score_map_msg = UInt32MultiArray()

# while not rospy.is_shutdown():
#     score_map_msg.data = np.array([1,2,3,4])

#     array_publisher.publish(score_map_msg)



# # ############### code end ###############
# rospy.signal_shutdown('ROS node is shutting down')


if __name__ == "__main__":
    
    # Get POI and make class
    poi_list = np.array([])
    def get_poi(msg):
        global poi_list
        total_poi = np.array([])
        for i in range(len(msg.poi)):
            tmp = np.array([])
            tmp = np.append(tmp, msg.poi[i].x)
            tmp = np.append(tmp, msg.poi[i].y)
            tmp = np.append(tmp, msg.poi[i].z)
        
            total_poi = np.append(total_poi, tmp)
        total_poi = np.reshape(total_poi, (-1,3))
        poi_list = total_poi
    
    rospy.init_node("Subscribe_POI", anonymous=True)

    rospy.Subscriber('/poi', poi, get_poi)

    time_get_poi = time.time()
    while len(poi_list) == 0:
        time.sleep(0.1)
        if time.time() - time_get_poi > 10:
            print("Error! No POI")
            
    target1 = ScoreMap(8, 10, 4, 'target1') # Target in m
