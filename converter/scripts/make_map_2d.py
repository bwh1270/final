import rospy
from std_msgs.msg import UInt32MultiArray, MultiArrayDimension # Publish score map in 3D Array
from converter.msg import xyz
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from icuas23_competition.msg import poi
import math
import numpy as np
import time
import threading

class Sub_msg:
    def __init__(self):
        self.drone_x = 0
        self.drone_y = 0
        
        self.is_print = False
        self.is_print_error = True
        
        self.drone_box_x = 0
        self.drone_box_y = 0
        
        self.wall_x = []
        self.wall_y = []
        
        self.INF = 3987654321 # 3094967290 # !!
        self.NULL = 123456789 # 200000000  # !!
        
        # Variable for safety
        self.resolution = 20 # cm
        self.slam_res = self.resolution # cm
        self.DRONE_SIZE = 50 # cm
        self.SAFETY_ALPHA = 15 # cm
        self.res_div = 100 // self.resolution
        self.safety_block = 1
        if self.is_print:
            print("Resolution (100 / cm) : ", self.res_div)

        self.time_limit = 20 # s
        self.maximum_time_limit = 200 # s

        self.size_x = 8.4 # m
        self.size_y = 6 # m
        self.map_size_x = int(self.size_x * self.res_div + 1)
        self.map_size_y = int(self.size_y * self.res_div + 1)
        self.score_np = np.ones((self.map_size_x, self.map_size_y), dtype = np.uint32) * self.NULL
        self.wall_map_init = np.ones((self.map_size_x, self.map_size_y), dtype = np.uint32) * self.NULL

        self.safety_dist = self.slam_res / 2 + self.DRONE_SIZE / 2 + self.SAFETY_ALPHA
        self.safety_block = math.ceil(self.safety_dist / self.resolution)
        if self.is_print:
            print("How many blocks for safety", self.safety_block)

    def get_pose(self, msg):
        # Edit drone x,y,z into positive value
        self.drone_x = msg.x # !!
        self.drone_y = msg.y # !!
        
        if msg.x < 0 or msg.y < 0:
            print("Error! Drone index negative number")
        
        self.drone_position() # !!
    
    def drone_position(self): # Do every time drone position is recieved.
        # !! Before this, Drone position origin must be aligned.
        # Did not make this function because real environment is unkown.
        # self.drone_box_x = round(self.drone_x * self.res_div)
        self.drone_box_x = self.drone_x
        if self.drone_box_x > self.map_size_x:
            self.drone_box_x = self.map_size_x
            if self.is_print_error:
                print("Error! Drone position error; x")
                print("Drone position : ", self.drone_box_x)
                print("Map size x : ", self.map_size_x)
        elif self.drone_box_x < 0:
            self.drone_box_x = 0
            if self.is_print_error:
                print("Error! Drone position error (negative value); x")

        # self.drone_box_y = round(self.drone_y * self.res_div)
        self.drone_box_y = self.drone_y
        if self.drone_box_y > self.map_size_y:
            self.drone_box_y = self.map_size_y
            if self.is_print_error:
                print("Error! Drone position error; y")
        elif self.drone_box_y < 0:
            self.drone_box_y = 0
            if self.is_print_error:
                print("Error! Drone position error (negative value); y")        

        if self.is_print:
            print("Drone position : ", self.drone_box_x, self.drone_box_y)
    
    def get_wall(self, msg):
        wall_all = msg.xyz
        
        wall_x_prev = []
        wall_y_prev = []
        for tmp in (wall_all):
            wall_x_prev.append(tmp.x)
            wall_y_prev.append(tmp.y)
        
        self.wall_x = wall_x_prev
        self.wall_y = wall_y_prev
        
        self.make_wall_map_init()
        
    def score_map_init(self):
        self.score_np = np.ones((self.map_size_x, self.map_size_y), dtype = np.uint32) * self.NULL
        
        self.score_np[:,0] = self.INF
        self.score_np[0,:] = self.INF
        self.score_np[self.map_size_x - 1,:] = self.INF
        self.score_np[:,self.map_size_y - 1] = self.INF
        
        # Remove corners
        self.score_np[1,1] = self.INF
        self.score_np[1, self.map_size_y - 2] = self.INF
        self.score_np[self.map_size_x - 2,1] = self.INF
        self.score_np[self.map_size_x - 2, self.map_size_y - 2] = self.INF
    
    def make_wall_map_init(self):
        if Var.is_print:
            print("Make wall map init")
        self.score_map_init()
        
        time_start = time.time()
        while len(self.wall_x) > 0:
            tmp_x = int(self.wall_x.pop())
            tmp_y = int(self.wall_y.pop())
            
            for i in range(-1 * self.safety_block, self.safety_block + 1): # !!
                for j in range(-1 * self.safety_block, self.safety_block + 1):
                    if tmp_x + i < 1 or tmp_x + i >= self.map_size_x or \
                    tmp_y + j < 1 or tmp_y + j >= self.map_size_y:
                        continue
                    
                    self.score_np[tmp_x + i, tmp_y + j] = self.INF

            if time.time() - time_start > self.time_limit:
                if self.is_print_error:
                    print("Too much obstacle; Time out !")
                break
        
        if self.is_print:
            print("Obstacle plot time : ", time.time() - time_start)
            print("Obstacle plot end")

        self.wall_map_init = self.score_np
        # if self.is_print:
        #     print(self.wall_map_init[0:20, 15:20])
        
        
            
class ScoreMap:
    '''
    If map has lots of obstcles,
    change resolution to smaller value
    change maximum distance 1.5 into bigger value

    If SLAM resolution chages, change the value
    '''
    def __init__(self, target_x, target_y):
        # variables
        self.time_start = time.time()
        # uint 32 maximum number : 4294967295
        self.INF = Var.INF
        self.NULL = Var.NULL
        self.size_x = Var.size_x # x, m
        self.size_y = Var.size_y # y, m

        self.is_print = Var.is_print
        self.is_print_error = Var.is_print_error

        # Variable for safety
        self.time_limit = Var.time_limit # s
        self.maximum_time_limit = Var.maximum_time_limit # s

        self.map_size_x = Var.map_size_x
        self.map_size_y = Var.map_size_y
        
        self.res_div = Var.res_div
        self.safety_block = Var.safety_block
        
        if self.is_print:
            print("Map size:", self.map_size_x, self.map_size_y)

        # Target position - Get from POI
        self.target_x = target_x # m 8
        self.target_y = target_y # m 10

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
                
        if self.is_print:
            print("Target : ", self.t_box_x, self.t_box_y)

        # Drone position - temporary
        self.drone_x = 0 # m
        self.drone_y = 0 # m

        Var.drone_box_x = round(Var.drone_x * self.res_div)
        Var.drone_box_y = round(Var.drone_y * self.res_div)

        # Make score map_init        
        self.score_np = np.ones((self.map_size_x, self.map_size_y), dtype = np.uint32) * self.NULL
        self.score_np_pub = np.ones((self.map_size_x, self.map_size_y), dtype = np.uint32) * self.INF
        self.score_np[self.t_box_x, self.t_box_y] = 0
        if self.is_print:
            print("Score of target : ", self.score_np[self.t_box_x, self.t_box_y])

        t = threading.Thread(target = self.cal_score_map)
        t.start()

    def cal_score_map(self):
        while True:
            # for i in range(20):
            #     print("Cal score map")
            # Init score map
            self.score_np = Var.wall_map_init
            self.score_np[self.t_box_x, self.t_box_y] = 0
            
            self.arr_q = [[self.t_box_x, self.t_box_y]]
            self.arr_fin = []
                        
            # Calculate Distance until meeting Drone
            self.time_start = time.time()
            # Set Mapping limit # !!
            self.x_upper_limit = max(self.t_box_x, Var.drone_box_x) + self.safety_block + 1
            if self.x_upper_limit > self.map_size_x - 1:
                self.x_upper_limit = self.map_size_x - 1
            # print("x upper limit : ", x_upper_limit)
            self.x_lower_limit = min(self.t_box_x, Var.drone_box_x) - self.safety_block - 2
            if self.x_lower_limit < 0:
                self.x_lower_limit = 0

            self.y_upper_limit = max(self.t_box_y, Var.drone_box_y) + self.safety_block + 1
            if self.y_upper_limit > self.map_size_y - 1:
                self.y_upper_limit = self.map_size_y - 1
            self.y_lower_limit = min(self.t_box_y, Var.drone_box_y) - self.safety_block - 2
            if self.y_lower_limit < 0:
                self.y_lower_limit = 0

            if self.is_print:
                print("Upper limit : ", self.x_upper_limit, self.y_upper_limit)
                print("Lower limit : ", self.x_lower_limit, self.y_lower_limit)

            # Make wall
            '''
            Get wall data from SLAM and plot them
            '''

            # Make Score map
            while len(self.arr_q) > 0:
                tmp_x, tmp_y = self.arr_q.pop(0)
                self.arr_fin.append([tmp_x, tmp_y])

                mini = self.score_np[tmp_x, tmp_y]
                for i in range(-1, 2):
                    if tmp_x + i == self.x_lower_limit or tmp_x + i == self.x_upper_limit:
                        continue 
                    for j in range(-1, 2):
                        if tmp_y + j == self.y_lower_limit or tmp_y + j == self.y_upper_limit:
                            continue

                        dist_tmp = self.score_np[tmp_x + i, tmp_y + j]
                        if dist_tmp == self.INF:
                            continue
                        
                        dist_tmp += 1
                        if dist_tmp <= mini:
                            mini = dist_tmp
                        else:
                            if [tmp_x + i, tmp_y + j] in self.arr_fin or [tmp_x + i, tmp_y + j] in self.arr_q:
                                pass
                            else:
                                self.arr_q.append([tmp_x + i, tmp_y + j])
                                
                if mini < self.NULL:
                    self.score_np[tmp_x, tmp_y] = mini
                    
                if tmp_x == Var.drone_box_x and tmp_y == Var.drone_box_y:
                    if self.is_print:
                        print("Meet drone")
                    break

                # Infinitive loop
                if time.time() - self.time_start > self.time_limit:
                    self.time_limit = int(1.1 * self.time_limit)
                    if self.time_limit > self.maximum_time_limit:
                        self.time_limit = self.maximum_time_limit
                    break

            self.score_np_pub = self.score_np * 1

            if self.is_print:
                print("Time : ", time.time() - self.time_start)

if __name__ == "__main__":
    Var = Sub_msg()
    
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
        total_poi = np.reshape(total_poi, (-1,3)) # Not necessary
        poi_list = total_poi
    
    rospy.init_node("Subscribe_POI", anonymous=True)

    rospy.Subscriber('/red/poi', poi, get_poi)
    rospy.Subscriber('/carrot_team/uav_xyz', Point, Var.get_pose)
    rospy.Subscriber('/carrot_team/pcl2xyz', xyz, Var.get_wall)

    time_get_poi = time.time()
    while len(poi_list) == 0:
        time.sleep(0.1)
        if time.time() - time_get_poi > 10:
            if Var.is_print_error:
                print("Error! No POI")
    
    # Dynamic class + publisher
    for i in range(len(poi_list)):
        globals()[f'target{i}'] = ScoreMap(poi_list[i][0] + 4.2, poi_list[i][1] + 3)
        globals()[f'pub_score_map_{i}'] = rospy.Publisher(f'/carrot_team/target{i}', UInt32MultiArray, queue_size = 10)
        if Var.is_print:
            print(f"target {i} started")
        
    while not rospy.is_shutdown():
        msg = UInt32MultiArray()
        for i in range(len(poi_list)):
            # test debug
            #if i == 5:
                #print(globals()[f'target{i}'].score_np_pub[0:20, 15:20])

            array_flat = np.reshape(globals()[f'target{i}'].score_np_pub, -1).astype(np.uint32)
            msg.data = list(array_flat)
            
            globals()[f'pub_score_map_{i}'].publish(msg)
            if Var.is_print:
                # print("Publish map ", i)
                pass
        time.sleep(0.1)