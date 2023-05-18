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


        self.timer = self.create_timer(1/100, self.timer_callback)

        self.timer_counter = 0

        
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


        ### ADDED ###

        self.current_pose = pose2d


        if self.initial_pose is None:
            self.initial_pose = pose2d
            

        # if self.initial_pose is not None and self.current_pose is not None:
        #     self.distance_travelled = math.sqrt( (self.current_pose[0] - self.initial_pose[0])**2 + (self.current_pose[1] - self.initial_pose[1])**2 )




    def prox_callback_f(self, msg):

        self.range_f = 1.0 if msg.range == 10.0 else msg.range

        #self.range_f = msg.range

    
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



    def timer_callback(self):
        
        cmd_vel = Twist() 
        
        if self.state == 'scanning' and self.initial_pose is not None:
            angle = self.current_pose[2]*180/np.pi if self.current_pose[2] > 0 else 360 + self.current_pose[2]*180/np.pi
            angle = round(angle + (self.spins * 360.00),2)
            cmd_vel.angular.z = 0.2

            if self.counter % 10 == 0:
                self.get_logger().info(
                            'angle:' + str(angle) + ' | range:' + str(round(self.range_f, 2)))
                self.points.append([self.range_f, self.current_pose[2]])
                
            
            if  abs(angle - self.previous_angle) > 180 and self.counter > 500: #5 seconds minimum
                 self.spins +=1
                 self.state = "done"
                 cmd_vel.angular.z = 0.0
            
            self.previous_angle = angle
        elif self.state == 'done':
            cmd_vel.angular.z = 0.0

            if len(self.points) >= 2:
                self.compute_points()
            else:
                print("ERROR, NOT ENOUGH POINTS")

        self.counter += 1

        stat = self.state
        cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z = (np.array(self.state_dict[stat])/5)

        self.vel_publisher.publish(cmd_vel)

    
    def compute_points(self):
        current_points = []
        visited_points = []

        x0, y0, t = self.initial_pose

        fig, ax = plt.subplots()
        ax.scatter(x0,y0, marker='D')
        min_x = 0
        max_x = 0
        min_y = 0
        max_y = 0 

        
        for dist,theta in self.points:
            x1 = x0 + dist * np.cos(theta)
            y1 = y0 + dist * np.sin(theta)

            current_points.append([x1,y1])


            # MIDDLE POINTS / VISITED COMPUTATION # 
            map_len_const = 0.25 #0.25
            step = map_len_const
            while(step < dist):
                x_mid = x0 + step * np.cos(theta)
                y_mid = y0 + step * np.sin(theta)
                visited_points.append([x_mid, y_mid])
                step += map_len_const
            #######################################

            if x1 > max_x:
                max_x = round(x1,2)
            elif x1 < min_x:
                min_x = round(x1,2)
            
            if y1 > max_y:
                max_y = y1
            elif y1 < min_y:
                min_y = y1


            print('x ' + str(round(x1,2)), 'y ' + str(round(y1,2)))
            ax.scatter(x1,y1, marker='.', color="red")


        for elem in visited_points:
            x,y = elem
            ax.scatter(x,y, marker='+', color="gray")

        self.state = 'map'

        print("X min:" + str(min_x) + " | max:" + str(max_x))
        print("Y min:" + str(min_y) + " | max:" + str(max_y))

        x_delta = max_x - min_x
        y_delta = max_y - min_y
        max_disc = max(x_delta, y_delta) # ensures to have a squared bounding box

        scaling = 10

        # Offset points (put them in a square box)
        
        if min_x < 0:
            min_x = -(min_x)
        
        if min_y < 0:
            min_y = -(min_y)

        scale_x = scaling/x_delta
        scale_y = scaling/y_delta

        # IN FUTURE MIGHT NEED TO DEAL WITH NEGATIVE MAX VALS
        offset_points = []
        
        for point in current_points:
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
            if x == 10:
                x = 9
            if y == 10:
                y = 9

            offset_points.append((x,y))

        ### INTERNAL

        offset_internal_points = []
        for point in visited_points:
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
            if x == 10:
                x = 9
            if y == 10:
                y = 9

            offset_internal_points.append((x,y))

        # DISCRETIZE INITIAL COORDS
        x0 = int((x0 + min_x)*scale_x)
        y0 = int((y0 + min_y)*scale_y)
        
        #print("new mapped points")
        #print(offset_points)

        #TODO 
        # Make offset discretized points a set

        grid = np.full((10,10), fill_value=0)
        for point in offset_points:
            x,y =  point # inverted points to conforn to array logic
            grid[y][x] += 1

        internal_only = set(offset_internal_points).difference(set(offset_points))

        for point in offset_internal_points:
            if point in internal_only:
                x,y =  point # inverted points to conforn to array logic
                grid[y][x] -= 1
        
        #grid[y0,x0] = 'シ'
        corrected_grid = np.flip(grid, axis=0)
        print(corrected_grid)

        blocked_grid = np.where(corrected_grid >= 2, 'x', 0)

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