import rclpy
from rclpy.node import Node
import tf_transformations
from std_msgs.msg import String

import numpy as np

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range

import matplotlib.pyplot as plt

import sys

import math

import time

import random


w_discover = False
room_discvoer = False
object_discover = False

class RobomasterNode(Node):
    def __init__(self):
        super().__init__('robomaster_node')

        # self.range_l = -1.0
        # self.range_r = -1.0
        # self.range_b = -1.0
        self.range_limit = 5.0 + 0.1
        self.scaling = 20 #10

        self.initial_pose = None
        self.current_pose = None
        self.counter = 0
        self.range_f = -1.0
        self.distance_travelled = 0     
        self.previous_angle = 0         # previous yaw in rad
        self.spins = 0                  # how many full spins the robot has done
        self.points = []                # points list saved as [range, yaw]

        self.state = "scanning"

        # Key: state, Val: velocities [lin x, lin y, ang z]
        self.state_dict = {"scanning":[0.0, 0.0, 1.0], 
                           "done": [ 0.0, 0.0, 0.0], 
                           "map":[ 0.0 , 0.0, 0.0],
                           "move: ": [ 0.0, 0.0, 0.0]} 

        
        self.odom_pose = None
        self.odom_velocity = None
                
        self.vel_publisher = self.create_publisher(Twist, '/RM0001/cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/RM0001/odom', self.odom_callback, 10)

        # Get sensor data
        self.proximity_f = self.create_subscription(Range, '/RM0001/range_0', self.prox_callback_f, 10)
        # self.proximity_l = self.create_subscription(Range, '/RM0001/range_1', self.prox_callback_l, 10)
        # self.proximity_b = self.create_subscription(Range, '/RM0001/range_2', self.prox_callback_b, 10)
        # self.proximity_r = self.create_subscription(Range, '/RM0001/range_3', self.prox_callback_r, 10)


        

        self.timer_counter = 0

        
    def start(self):
        # Create and immediately start a timer that will regularly publish commands
        self.timer = self.create_timer(1/100, self.timer_callback)

    
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
            self.initial_pose = pose2d


    def prox_callback_f(self, msg):
        # 10.0 is the coppelia standard reading for out of range
        self.range_f = self.range_limit if msg.range == 10.0 else msg.range
    
    # def prox_callback_l(self, msg):
    #     self.range_l = msg.range

    # def prox_callback_r(self, msg):
    #     self.range_r = msg.range

    # def prox_callback_b(self, msg):
    #     self.range_b = msg.range
        
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


    def rotate_360(self, rot_step):
        cmd_vel = Twist() 
        
        if self.state == 'scanning' and self.initial_pose is not None:
            angle = self.current_pose[2]*180/np.pi if self.current_pose[2] > 0 else 360 + self.current_pose[2]*180/np.pi
            angle = round(angle + (self.spins * 360.00),2)
            cmd_vel.angular.z = rot_step

            if self.counter % 10 == 0:
                self.get_logger().info(
                            'angle:' + str(angle) + ' | range:' + str(round(self.range_f, 2)))
                self.points.append([self.range_f, self.current_pose[2]]) 
            
            if  abs(angle - self.previous_angle) > 180 and self.counter > 500: #5 seconds minimum
                 self.spins +=1
                 self.state = "done"
                 cmd_vel.angular.z = 0.0
            
            self.previous_angle = angle
        
        self.vel_publisher.publish(cmd_vel)

    def timer_callback(self):
        

        self.rotate_360(0.2)

        cmd_vel = Twist() 

        if self.state == 'done':
            cmd_vel.angular.z = 0.0

            if len(self.points) >= 2:
                self.compute_points()
            else:
                print("ERROR, NOT ENOUGH POINTS")

        self.counter += 1
        cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z = (np.array(self.state_dict[self.state])/5)

        self.vel_publisher.publish(cmd_vel)

    # Populate all visited points and all wall points. Discretizes basd on a factor
    def pop_visited_wall_p(self, visited, wall, discrete):
        x0, y0, _ = self.initial_pose

        min_x = 0
        max_x = 0
        min_y = 0
        max_y = 0 

        
        for dist,theta in self.points:
            x1 = x0 + dist * np.cos(theta)
            y1 = y0 + dist * np.sin(theta)


            if dist < self.range_limit:
                wall.append([x1,y1])

            map_len_const = discrete #0.25
            step = map_len_const
            while(step < dist):
                x_mid = x0 + step * np.cos(theta)
                y_mid = y0 + step * np.sin(theta)
                visited.append([x_mid, y_mid])
                step += map_len_const

            if x1 > max_x:
                max_x = round(x1,2)
            elif x1 < min_x:
                min_x = round(x1,2)
            
            if y1 > max_y:
                max_y = y1
            elif y1 < min_y:
                min_y = y1

        self.state = 'map'

        print("X min:" + str(min_x) + " | max:" + str(max_x))
        print("Y min:" + str(min_y) + " | max:" + str(max_y))

        return max_x, min_x, max_y, min_y, visited, wall


    def comp_offset(self, min_x, min_y, points, scaling, scale_x, scale_y):

        # IN FUTURE MIGHT NEED TO DEAL WITH NEGATIVE MAX VALS
        offset_points = []
        
        for point in points:
            x,y = point

            # translate points into the positive quadrant
            x += min_x
            y += min_y 

            # scale points into coordinate system
            x *= scale_x
            y *= scale_y

            x = int(x)
            y = int(y)

            # TOBE FIXED LATER
            if x == scaling:
                x = scaling -1
            if y == scaling:
                y = scaling -1

            offset_points.append((x,y))

        return offset_points


    def compute_points(self):

        max_x, min_x, max_y, min_y, visited_points, wall_points = self.pop_visited_wall_p([], [], 0.2)
        x0, y0, _ = self.initial_pose
        _, ax = plt.subplots()
        ax.scatter(x0,y0, marker='D')
        
        for elem in wall_points:
            x,y = elem
            ax.scatter(x,y, marker='.', color="red")

        for elem in visited_points:
            x,y = elem
            ax.scatter(x,y, marker='+', color="gray")

        scaling = self.scaling

        x_delta = max_x - min_x
        y_delta = max_y - min_y

        # Offset points (put them in a square box)
        min_x = abs(min_x)
        min_y = abs(min_y)

        scale_x = scaling/x_delta
        scale_y = scaling/y_delta

        wall_offset = self.comp_offset(min_x, min_y, wall_points, scaling, scale_x, scale_y)
        visited_offset = self.comp_offset(min_x, min_y, visited_points, scaling, scale_x, scale_y)

        # ### INTERNAL

        # offset_internal_points = []
        # for point in visited_points:
        #     x,y = point

        #     # translate points into the positive quadrant
        #     x += min_x
        #     y += min_y 

        #     # scale points into coordinate system
        #     x *= scale_x
        #     y *= scale_y

        #     x = int(x)
        #     y = int(y)

        #     # Avoids out of bounds indexes
        #     if x == scaling:
        #         x = scaling-1
        #     if y == scaling:
        #         y = scaling-1

        #     offset_internal_points.append((x,y))

        # DISCRETIZE INITIAL COORDS
        x0 = int((x0 + min_x)*scale_x)
        y0 = int((y0 + min_y)*scale_y)
        
        #print("new mapped points")
        #print(offset_points)

        #TODO 
        # Make offset discretized points a set

        grid = np.full((scaling,scaling), fill_value=0)
        for point in wall_offset:
            x,y =  point # inverted points to conforn to array logic
            grid[y][x] += 1

        internal_only = set(visited_offset).difference(set(wall_offset))

        for point in visited_offset:
            if point in internal_only:
                x,y =  point # inverted points to conforn to array logic
                grid[y][x] -= 1
        
        #grid[y0,x0] = 'シ'
        corrected_grid = np.flip(grid, axis=0)
        #[print(elem) for elem in corrected_grid]
        print(corrected_grid)
        #print(corrected_grid)

        blocked_grid = np.where(corrected_grid >= 2, 'x', (np.where(corrected_grid <= -2, '.', '0')))

        blocked_grid[y0,x0] = 'シ' #This is us

        coords = np.argwhere(blocked_grid == 'x')
        print("wall coords amount:", len(coords))

        print(blocked_grid)
        #print(blocked_grid.transpose())

    
        ax.set_title("map1")
        plt.show()



    def plot_points(self, points):
        #for point in points:
        return

        


def main():
    np.set_printoptions(linewidth=100)
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)
    
    # Create an instance of your node class
    node = RobomasterNode()

    node.start()

    # Keep processings events until someone manually shuts down the node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Ensure the Thymio is stopped before exiting
    node.stop()

if __name__ == '__main__':
    main()