import rclpy
from rclpy.node import Node
import tf_transformations
from std_msgs.msg import String

import numpy as np

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range

import sys

import math

import time

import random

     
class RobomasterNode(Node):
    def __init__(self):
        super().__init__('robomaster_node')

        self.range_f = -1
        self.range_l = -1
        self.range_r = -1
        self.range_b = -1
        self.initial_pose = None
        self.current_pose = None
        self.current_coordinate = [5,5] # center of the grid, origin of the grid is left up corner [x,y]
        self.distance_travelled = 0
        # self.state = "up" # default move up
        self.state = random.choice(["up","left","right","down"])

        self.multiplier = 0.0

        self.ranges = [self.range_f, self.range_l, self.range_r, self.range_b]
        self.proximity_names = ["up", "left", "down", "right", "blocked"]

        # Key: state, Val: [linearX, linearY]
        self.state_dict = {"up":[1.0,0.0], "down": [-1.0,0.0], "right":[0.0,-1.0], "left": [0.0,1.0], "blocked":[0.0,0.0]}

        self.grid = np.full((12,12), fill_value=' ')
        
        # Create attributes to store odometry pose and velocity
        self.odom_pose = None
        self.odom_velocity = None
                
        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, '/RM0001/cmd_vel', 10)

        # Create a subscriber to the topic 'odom', which will call 
        # self.odom_callback every time a message is received
        self.odom_subscriber = self.create_subscription(Odometry, '/RM0001/odom', self.odom_callback, 10)

        # Get sensor data
        self.proximity_f = self.create_subscription(Range, '/RM0001/range_0', self.prox_callback_f, 10)
        self.proximity_l = self.create_subscription(Range, '/RM0001/range_1', self.prox_callback_l, 10)
        self.proximity_b = self.create_subscription(Range, '/RM0001/range_2', self.prox_callback_b, 10)
        self.proximity_r = self.create_subscription(Range, '/RM0001/range_3', self.prox_callback_r, 10)


        self.timer = self.create_timer(1/100, self.timer_callback)

        #self.gridTimer = self.create_timer(10, self.grid_print_callback)

        #self.coordinateTimer = self.create_timer(10, self.coord_print_callback)

        self.timer_counter = 0
        
        # NOTE: we're using relative names to specify the topics (i.e., without a 
        # leading /). ROS resolves relative names by concatenating them with the 
        # namespace in which this node has been started, thus allowing us to 
        # specify which Thymio should be controlled.
        
    def start(self):
        # Create and immediately start a timer that will regularly publish commands
        self.timer = self.create_timer(1/60, self.update_callback)

    
    def stop(self):
        # Set all velocities to zero
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)
    
    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_valocity = msg.twist.twist
        
        pose2d = self.pose3d_to_2d(self.odom_pose)

        self.current_pose = pose2d

        if self.initial_pose is None:
            self.multiplier = 0.0
        else:
            self.multiplier = 1.0


        if self.initial_pose is None:
            self.initial_pose = pose2d
            

            
        
        # self.get_logger().info(
        #     "odometry: received pose (x: {:.2f}, y: {:.2f}, theta: {:.2f})".format(*pose2d),
        #      throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        # ) 
        if self.initial_pose is not None and self.current_pose is not None:
            self.distance_travelled = math.sqrt( (self.current_pose[0] - self.initial_pose[0])**2 + (self.current_pose[1] - self.initial_pose[1])**2 )

        if self.distance_travelled >= .5:
            self.distance_travelled = 0
            self.initial_pose = None
            y, x = self.state_dict[self.state] #dictionary logic inverted
            # make x and y equal to 1 or -1 
            self.grid[self.current_coordinate[1]][self.current_coordinate[0]] = '.'
            self.current_coordinate = [self.current_coordinate[0] - int(x), self.current_coordinate[1] - int(y)]
            self.grid[self.current_coordinate[1]][self.current_coordinate[0]] = 'o'
            self.coord_print_callback()
            self.grid_print_callback()

        # self.get_logger().info(
        #             str(self.initial_pose)
        #         )
        # self.get_logger().info(
        #             str(self.current_pose)
        #         )
        # self.get_logger().info(
        #             str(self.distance_travelled)
        #         )


    def prox_callback_f(self, msg):
        self.range_f = msg.range
    
    def prox_callback_l(self, msg):
        self.range_l = msg.range

    def prox_callback_r(self, msg):
        self.range_r = msg.range

    def prox_callback_b(self, msg):
        self.range_b = msg.range
        
    def pose3d_to_2d(self, pose3):
        quaternion = (
            pose3.orientation.x,
            pose3.orientation.y,
            pose3.orientation.z,
            pose3.orientation.w
        )
        
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)
        
        pose2 = (
            pose3.position.x,  # x position
            pose3.position.y,  # y position
            yaw                # theta orientation
        )
        
        return pose2
    
    def choose_neighbour(self):
        #self.initial_pose = None
        x,y = self.current_coordinate

        possible_routes = []

        if y-1 >= 0 and self.grid[y-1][x] == ' ':
            #self.state = "up"
            possible_routes.append("up")

        if x-1 >= 0 and self.grid[y][x-1] == ' ':
            #self.state = "left"
            possible_routes.append("left")

        if y+1 < self.grid.shape[0] and self.grid[y+1][x] == ' ':
            # self.state = "down"
            possible_routes.append("down")

        if x+1 < self.grid.shape[1] and self.grid[y][x+1] == ' ':
             # self.state = "right"
            possible_routes.append("right")

        if len(possible_routes) == 0:
            self.choose_neighbour_visited()
        else:
            self.state = random.choice(possible_routes)
            self.initial_pose = self.current_pose

    def choose_neighbour_visited(self):
        #self.initial_pose = None
        x,y = self.current_coordinate

        possible_routes = []

        if y-1 >= 0 and self.grid[y-1][x] == '.':
            #self.state = "up"
            possible_routes.append("up")

        if x-1 >= 0 and self.grid[y][x-1] == '.':
            #self.state = "left"
            possible_routes.append("left")

        if y+1 < self.grid.shape[0] and self.grid[y+1][x] == '.':
            # self.state = "down"
            possible_routes.append("down")

        if x+1 < self.grid.shape[1] and self.grid[y][x+1] == '.':
             # self.state = "right"
            possible_routes.append("right")
        
        self.initial_pose = self.current_pose
        self.state = random.choice(possible_routes)



    def timer_callback(self):

        if self.initial_pose is not None:
            self.get_logger().info(
                    str(self.current_pose[2] * 180/np.pi)
                )

        # self.get_logger().info(
        #                 str(self.initial_pose) # Throttle logging frequency to max 2Hz
        #             )
        
        cmd_vel = Twist() 
        

        #cmd_vel.linear.x  = 0.2 # [m/s]
        #cmd_vel.linear.y  = -0.2 # [m/s]
        #cmd_vel.angular.z = 0.2 # [rad/s]

        if self.state == 'blocked':
            #find free neighbour
            self.choose_neighbour()
            

        # ONLY SHOW SENSOR READING THAT ARE BELOW A CERTAIN THRESHOLD
        ranges = [self.range_f, self.range_l, self.range_r, self.range_b]
        names = ["up", "left", "right", "down"]
        id = -1
        for elem in ranges:
            id += 1
            elem = round(float(elem),4)
            
            if elem > 0.0 and elem < 10.0:
                
                self.get_logger().info(
                    str(names[id]) + ' range:' + str(elem),
                    throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
                )
                if elem < 0.15:
                    sensor = str(names[id])
                    x, y = self.current_coordinate
                    if sensor == "up" and self.grid[y-1][x] == ' ':
                        self.grid[y-1][x] = 'x'
                        self.grid_print_callback()
                        self.state = 'blocked'
                        #self.initial_pose = self.current_pose
                    elif sensor == "down" and self.grid[y+1][x] == ' ':
                        self.grid[y+1][x] = 'x'
                        self.grid_print_callback()
                        self.state = 'blocked'
                        #self.initial_pose = self.current_pose
                    elif sensor == "left" and self.grid[y][x-1] == ' ':
                        self.grid[y][x-1] = 'x'
                        self.grid_print_callback()
                        self.state = 'blocked'
                        #self.initial_pose = self.current_pose
                    elif sensor == "right" and self.grid[y][x+1] == ' ':
                        self.grid[y][x+1] = 'x'
                        self.grid_print_callback()
                        self.state = 'blocked'
                        #self.initial_pose = self.current_pose
                 

        stat = self.state
        cmd_vel.linear.x, cmd_vel.linear.y  = (np.array(self.state_dict[stat])/5)* self.multiplier

        self.vel_publisher.publish(cmd_vel)

    def coord_print_callback(self):
        self.get_logger().info(
                        "current coordinate" + str(self.current_coordinate) # Throttle logging frequency to max 2Hz
                    )

    def grid_print_callback(self):

        for elem in self.grid:
            self.get_logger().info(
                        str(elem) # Throttle logging frequency to max 2Hz
                    )
        self.get_logger().info(
                        '------------------------------------------------' # Throttle logging frequency to max 2Hz
                    )


def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)
    
    # Create an instance of your node class
    node = RobomasterNode()

    # Keep processings events until someone manually shuts down the node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


    
    # Ensure the Thymio is stopped before exiting
    node.stop()

if __name__ == '__main__':
    main()
