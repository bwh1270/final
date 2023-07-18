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