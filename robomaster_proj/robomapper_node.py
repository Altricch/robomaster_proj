# import rclpy
# from rclpy.node import Node
# import tf_transformations
# from std_msgs.msg import String

# import numpy as np
# import math

# from geometry_msgs.msg import Twist, Pose
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import Range

# import sys


# class RobomasterNode(Node):
#     def __init__(self):
#         super().__init__('robomaster_node')

#         self.range_f = -1
#         self.range_l = -1
#         self.range_r = -1
#         self.range_b = -1
        
#         self.w_discover = False
#         self.w_turning = False
#         self.w_finished = False
#         self.room_discvoer = False
#         self.object_discover = False
#         self.right_history = []
#         self.min_rdist = 100
#         self.last_fron = 100
#         self.last_righ = 100

#         self.chassi_x = 0.10078
#         self.chassi_y = 0.21501


#         # Create attributes to store odometry pose and velocity
#         self.odom_pose = None
#         self.odom_velocity = None
                
#         # Create a publisher for the topic 'cmd_vel'
#         self.vel_publisher = self.create_publisher(Twist, '/RM0001/cmd_vel', 10)

#         # Create a subscriber to the topic 'odom', which will call 
#         # self.odom_callback every time a message is received
#         self.odom_subscriber = self.create_subscription(Odometry, '/RM0001/odom', self.odom_callback, 10)

#         # Get sensor data
#         self.proximity_f = self.create_subscription(Range, '/RM0001/range_0', self.prox_callback_f, 10)
#         self.proximity_l = self.create_subscription(Range, '/RM0001/range_1', self.prox_callback_l, 10)
#         self.proximity_b = self.create_subscription(Range, '/RM0001/range_2', self.prox_callback_b, 10)
#         self.proximity_r = self.create_subscription(Range, '/RM0001/range_3', self.prox_callback_r, 10)


#         self.timer = self.create_timer(1, self.discover_wall)
#         self.timer_counter = 0
        
#         # NOTE: we're using relative names to specify the topics (i.e., without a 
#         # leading /). ROS resolves relative names by concatenating them with the 
#         # namespace in which this node has been started, thus allowing us to 
#         # specify which Thymio should be controlled.
        
#     def start(self):
#         # Create and immediately start a timer that will regularly publish commands
#         self.timer = self.create_timer(1/60, self.update_callback)

    
#     def stop(self):
#         # Set all velocities to zero
#         cmd_vel = Twist()
#         self.vel_publisher.publish(cmd_vel)
    
#     def odom_callback(self, msg):
#         self.odom_pose = msg.pose.pose
#         self.odom_valocity = msg.twist.twist
        
#         pose2d = self.pose3d_to_2d(self.odom_pose)
        
#         # self.get_logger().info(
#         #     "odometry: received pose (x: {:.2f}, y: {:.2f}, theta: {:.2f})".format(*pose2d),
#         #      throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
#         # )


#     def prox_callback_f(self, msg):
#         self.range_f = msg.range
    
#     def prox_callback_l(self, msg):
#         self.range_l = msg.range

#     def prox_callback_r(self, msg):
#         self.range_r = msg.range

#         if len(self.right_history) > 10:
#             self.right_history = self.right_history[8:]
        
#         self.right_history.append(self.range_r)

#     def prox_callback_b(self, msg):
#         self.range_b = msg.range
    
#     def pose3d_to_2d(self, pose3):
#         quaternion = (
#             pose3.orientation.x,
#             pose3.orientation.y,
#             pose3.orientation.z,
#             pose3.orientation.w
#         )
        
#         roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)
        
#         pose2 = (
#             pose3.position.x,  # x position
#             pose3.position.y,  # y position
#             yaw                # theta orientation
#         )
        
#         return pose2

#     # Turns right until perpendicular to wall
#     def turn_right(self):
#         perpendicular = False
#         min_r_range = 1000

#         self.get_logger().info(
#                     "Now in Right Turn ",
#                     throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
#                 )

#         while not perpendicular:

#             self.get_logger().info(
#                     "Range back: {:.2f}".format(self.range_b),
#                     throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
#                 )

#             cmd_vel = Twist()
#             cmd_vel.linear.x  = 0.1

            

#             if min_r_range > self.range_r:
#                 min_r_range = self.range_r 
#         return 

#     def discover_wall(self):


#         self.get_logger().info(
#                     "DISCOVER WALL",
#                     throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
#                 )
        
#         cmd_vel = Twist() 

#         if self.last_fron > self.range_f:
#             self.last_fron = self.range_f

#         if self.last_righ > self.range_r:
#             self.last_righ = self.range_r
        

#         self.get_logger().info(
#             "Range front: {:.2f}".format(self.range_f),
#             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
#         )

#         self.get_logger().info(
#             "Range left: {:.2f}".format(self.range_l),
#             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
#         )

#         self.get_logger().info(
#             "Range right: {:.2f}".format(self.range_r),
#             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
#         )

#         self.get_logger().info(
#             "Range back: {:.2f}".format(self.range_b),
#             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
#         ) 


#         if not self.w_discover:
#             if self.range_f > 0.2 and not self.w_turning:
                
#                 cmd_vel.linear.x  = 0.2

                
#             else:
#                 self.w_turning = True

#                 if not math.isclose(self.last_fron, self.last_righ, abs_tol=0.065): 
#                     cmd_vel.angular.z = 0.1
#                     self.get_logger().info(
#                         "Range front: {:.4f}, Range back {:.4f}".format(self.last_fron - self.chassi_x, self.last_righ - self.chassi_y),
#                         throttle_duration_sec=0.2 # Throttle logging frequency to max 2Hz
#                     ) 

#                 else:
#                     cmd_vel.angular.z  = 0.0
                    
#             self.vel_publisher.publish(cmd_vel)

#     def timer_callback(self):
        
#         t_circle = 96

#         cmd_vel = Twist() 
        
#         if self.timer_counter < t_circle // 2:
#             cmd_vel.linear.x  = 0.2 # [m/s]
#             cmd_vel.angular.z = 0.0 # [rad/s]
#         else:
#             cmd_vel.linear.x  = 0.2 # [m/s]
#             cmd_vel.angular.z = -0.0 # [rad/s]bool

#         self.timer_counter += 1
#         if self.timer_counter > t_circle:
#             self.timer_counter = self.timer_counter % t_circle

#         # self.get_logger().info(
#         #     "in timed function, counter is: {:.2f})".format(self.timer_counter),
#         #      throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
#         # )
#         self.get_logger().info(
#             "Range front: {:.2f}".format(self.range_f),
#             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
#         )

#         self.get_logger().info(
#             "Range left: {:.2f}".format(self.range_l),
#             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
#         )

#         self.get_logger().info(
#             "Range right: {:.2f}".format(self.range_r),
#             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
#         )

#         self.get_logger().info(
#             "Range back: {:.2f}".format(self.range_b),
#             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
#         )

        
#         self.vel_publisher.publish(cmd_vel)


# def main():
#     # Initialize the ROS client library
#     rclpy.init(args=sys.argv)
    
#     # Create an instance of your node class
#     node = RobomasterNode()

#     # Keep processings events until someone manually shuts down the node
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass


    
#     # Ensure the Thymio is stopped before exiting
#     node.stop()

# if __name__ == '__main__':
#     main()

##########################################NEW#########################################33

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

        self.counter = 0

        self.range_f = -1.0
        self.range_l = -1.0
        self.range_r = -1.0
        self.range_b = -1.0
        self.initial_pose = None
        self.current_pose = None
        self.current_coordinate = [5,5] # center of the grid, origin of the grid is left up corner [x,y]
        self.distance_travelled = 0
        self.previous_angle = 0
        self.spins = 0
        self.points = []

        self.state_new = "scanning"
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
        # self.proximity_l = self.create_subscription(Range, '/RM0001/range_1', self.prox_callback_l, 10)
        # self.proximity_b = self.create_subscription(Range, '/RM0001/range_2', self.prox_callback_b, 10)
        # self.proximity_r = self.create_subscription(Range, '/RM0001/range_3', self.prox_callback_r, 10)


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

    def plot_points(self, points):
        #for point in points:
        return
            


    def timer_callback(self):
        
        cmd_vel = Twist() 
        
        if self.state_new == 'scanning' and self.initial_pose is not None:
            angle = self.current_pose[2]*180/np.pi if self.current_pose[2] > 0 else 360 + self.current_pose[2]*180/np.pi
            angle = round(angle + (self.spins * 360.00),2)
            cmd_vel.angular.z = 0.2

            if self.counter % 10 == 0:
                self.get_logger().info(
                            'angle:' + str(angle) + ' | range:' + str(round(self.range_f, 2)))
                self.points.append([self.range_f, self.current_pose[2]])
                
            
            if  abs(angle - self.previous_angle) > 180:
                 self.spins +=1
                 self.state_new = "done"
                 cmd_vel.angular.z = 0.0
            
            self.previous_angle = angle
        elif self.state_new == 'done':
            cmd_vel.angular.z = 0.0
            self.compute_points()

        if self.state == 'blocked':
            #find free neighbour
            self.choose_neighbour()

        self.counter += 1

        stat = self.state
        cmd_vel.linear.x, cmd_vel.linear.y  = (np.array(self.state_dict[stat])/5)* 0.0#self.multiplier

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
    
    def compute_points(self):
        current_points = []

        x, y, t = self.initial_pose

        fig, ax = plt.subplots()
        ax.scatter(x,y, marker='D')
        min_x = 0
        max_x = 0
        min_y = 0
        max_y = 0 
        for dist,theta in self.points:
            x1 = x + dist * np.cos(theta)
            y1 = y + dist * np.sin(theta)

            current_points.append([x1,y1])

            if x1 > max_x:
                max_x = round(x1,2)
            elif x1 < min_x:
                min_x = round(x1,2)
            
            if y1 > max_y:
                max_y = y1
            elif y1 < min_y:
                min_y = y1


            print('x ' + str(round(x1,2)), 'y ' + str(round(y1,2)))
            ax.scatter(x1,y1, marker='.')
        self.state_new = 'map'

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

            offset_points.append([x,y])
        
        #print("new mapped points")
        #print(offset_points)

        grid = np.full((10,10), fill_value=0)
        for point in offset_points:
            x,y =  point # inverted points to conforn to array logic
            grid[y][x] += 1

        corrected_grid = np.flip(grid, axis=0)
        print(corrected_grid)

        blocked_grid = np.where(corrected_grid >= 2, 'x', ' ')

        print(blocked_grid)

    
        ax.set_title("map1")
        plt.show()

        


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