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
        self.speed_damper = 5.0

        self.discrete = 0.25
        
        self.initial_pose = None
        self.current_pose = None
        self.target_pose = None

        self.counter = 0
        self.range_f = -1.0
        self.distance_travelled = 0     
        self.previous_angle = 0         # previous yaw in rad
        self.spins = 0                  # how many full spins the robot has done
        self.points = []                # points list saved as [range, yaw]

        self.state = "scanning"

        # Key: state, Val: velocities [lin x, lin y, ang z]
        self.state_dict = {"scanning":[0.0, 0.0, 1.0], 
                               "done":[ 0.0, 0.0, 0.0], 
                                "map":[ 0.0 , 0.0, 0.0],
                               "move":[ 0.0, 1.0, 0.0]} 

        
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

    # Rotates robot by 360 degrees
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

            if len(self.points) >= 2:
                self.compute_all()
            else:
                print("ERROR, NOT ENOUGH POINTS")

        self.counter += 1
        cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z = np.array(self.state_dict[self.state])/self.speed_damper

        self.vel_publisher.publish(cmd_vel)

    # Populate all visited points and all wall points. Discretizes basd on a factor
    def pop_visited_wall_p(self):
        x0, y0, _ = self.initial_pose

        min_x = 0
        max_x = 0
        min_y = 0
        max_y = 0 

        visited = []
        wall = []
        discrete = self.discrete
        
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

    # given offset given point coordinates by a given amount
    def comp_offset(self, min_x, min_y, points, scale_x, scale_y):

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
            if x == self.scaling:
                x = self.scaling -1
            if y == self.scaling:
                y = self.scaling -1

            offset_points.append((x,y))

        return offset_points


    # Populates a grid with cumulative statistics according to discretized votes
    # for both wall points (postiive values) and visited points (negative values) 
    def pop_grid(self, wall_offset, visited_offset):
        grid = np.zeros((self.scaling,self.scaling))
        for point in wall_offset:
            x,y =  point            # inverted points to conforn to array logic
            grid[y][x] += 1

        internal_only = set(visited_offset).difference(set(wall_offset))




        for point in visited_offset:
            if point in internal_only:
                x,y =  point # inverted points to conforn to array logic
                grid[y][x] -= 1

        return grid.astype(int)
    
    # takes the cummulative grid and transforms it into an binary representation. 
    # "x" if the point is a wall, "." if the point has been visited and is walkable. 0 otherwise. 
    # Lastly, we cap the cumulative probability for both states according to observations
    def pop_binary_grid(self, acc_grid, x0, y0, cap_wall = 2, cap_visited = -2):
        # binary_grid = np.where(acc_grid >= cap_wall, '□', (np.where(acc_grid <= cap_visited, '·', '?')))
        # binary_grid[binary_grid.shape[0] - y0, x0] = '웃' # ﾂｯツシｼッシ'This is us

        binary_grid = np.where(acc_grid >= cap_wall, 'x', (np.where(acc_grid <= cap_visited, '.', '0')))
        binary_grid[binary_grid.shape[0] - y0, x0] = 'ﾂ' # ﾂｯツシｼッシ'This is us

        return binary_grid
    


    def compute_all(self):
    
        max_x, min_x, max_y, min_y, visited_points, wall_points = self.pop_visited_wall_p()
        x0, y0, _ = self.initial_pose
        _, ax = plt.subplots()
        ax.scatter(x0,y0, marker='D')

        visited_points = np.array(visited_points)
        visited_x = visited_points[:,0]
        visited_y = visited_points[:,1]

        # for elem in visited_points:
        #     x,y = elem
        ax.scatter(x = visited_x,y = visited_y, marker='+', color="gray")

        wall_points = np.array(wall_points)
        wall_x = wall_points[:,0]
        wall_y = wall_points[:,1]
        ax.scatter(x = wall_x,y = wall_y, marker='.', color="red")

        # Offset points (put them in a square box)
        # scaling = self.scaling
        x_delta = max_x - min_x
        y_delta = max_y - min_y
        min_x = abs(min_x)
        min_y = abs(min_y)
        scale_x = self.scaling/x_delta
        scale_y = self.scaling/y_delta


        # Compute offset for wall points and visited points
        wall_offset = self.comp_offset(min_x, min_y, wall_points, scale_x, scale_y)
        visited_offset = self.comp_offset(min_x, min_y, visited_points, scale_x, scale_y)

        # DISCRETIZE INITIAL COORDS
        x0 = int((x0 + min_x)*scale_x)
        y0 = int((y0 + min_y)*scale_y)
        
        # Get cumulative grid with votes
        grid = self.pop_grid(wall_offset, visited_offset)
        
        # Flip the grid and print cummulative grid
        corrected_grid = np.flip(grid, axis=0)
        print(corrected_grid)
        
        # Get binary grid and print
        binary_grid = self.pop_binary_grid(corrected_grid, x0, y0)

        # print(binary_grid)
        print(np.array2string(binary_grid, separator=' ', formatter={'str_kind': lambda x: x}))

        # check how many points are identified as wall
        coords = np.argwhere(binary_grid == 'x')
        print("wall coords amount:", len(coords))
        

        ### GETTING CLOSEST ####
        print(binary_grid.shape)
        nearest, walkable = select_route(binary_grid)
        dest_x, dest_y = nearest
        print("nearest and walkable options")
        print(nearest, walkable)
        #########################

        #self.state = 'move'
        ax.set_title("map1")
        plt.show()


# Retrieves all plausible candidates, e.g. that have a 0 neighbor and a reachable neighbor.
def unseen_neighbors(binary):
    unseen = np.where(binary == "0")
    x, y = unseen 
    plausible_pos = []
    for elem in zip(x,y):
        if candidate(binary, elem):
            plausible_pos.append(elem)
    #print("Plausible candidates are", plausible_pos)
    return plausible_pos

# Retrieves the element with min distance amongst all plausible candidates 
def min_dist(plausible_cand, current_pos):
    sx, sy = current_pos
    nearest = None
    dist = 100
    for cand in plausible_cand:
        fx, fy = cand
        temp_dist = math.sqrt((fx-sx)**2 + (fy-sy)**2)
        if temp_dist < dist:
            dist = temp_dist
            nearest = cand
        
    return nearest

# Checks whether a position is a candidate
def candidate(binary, elem, max_x = 20, min_x = 0, max_y = 20, min_y = 0):
    in_0_neihborhood = False
    in_reach_neighborhood = False
    
    x, y = elem
    
    elems = []
    # Border element 
    if min_x < x < max_x - 1 and min_y < y < max_y - 1:
        elems.append((x+1,y))
        elems.append((x-1,y))
        elems.append((x,y+1))
        elems.append((x,y-1))
        elems.append((x+1,y+1))
        elems.append((x-1,y-1))
        elems.append((x-1,y+1))
        elems.append((x-1,y+1))
        
    if len(elems) == 0:
        return False
    
    for el in elems:
        x_t, y_t = el
        if binary[x_t,y_t] == "0":
            in_0_neihborhood = True
        if binary[x_t,y_t] == ".":  
            in_reach_neighborhood = True
        
        if in_0_neihborhood and in_reach_neighborhood:
            return True
            
    return in_0_neihborhood and in_reach_neighborhood

# either LT, UT, Diag, Hor
def check_path(current_pos, target_pos, binary):
    sx, sy = current_pos
    fx, fy = target_pos
    
    csx = sx
    csy = sy
    
    dx = np.sign(fx - sx)
    dy = np.sign(fy - sy)    
    
    LT = True
    UT = True
    Diagonal = True
    
    # UTriangular
    sx = csx
    sy = csy
    while sx != fx:
        sx += dx
        # CMD.Velangular.x
        if binary[sx,sy] == "x":
            UT = False
            
    if UT:
        while sy != fy:
            sy += dy
            elem = binary[sx,sy]
            if elem == "x":
                UT = False
            
            
    # LTriangular
    sx = csx
    sy = csy
    while sy != fy:
        sy += dy
        elem = binary[sx,sy]
        if elem == "x":
            LT = False
    if LT:
        while sx != fx:
            sx += dx
            # CMD.Velangular.x
            if binary[sx,sy] == "x":
                LT = False
                
    # Diagonal 
    sx = csx
    sy = csy
    while sy != fy and sx != fx:
        if sy != fy:
            sy += dy
        if sx != fx:
            sx += dx
        elem = binary[sx,sy]
        if elem == "x":
            Diagonal = False
            
    return UT, LT, Diagonal
        
# Retrieves from the array the position we currenly have
def get_current_pos(binary):
    
    # binary =   np.array([['0', '0', '0', '0', '0', '0', '0', '0', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x'],
    #                     ['0', '0', '0', '0', '0', '0', '0', '0', 'x', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', 'x'],
    #                     ['0', '0', '0', '0', '0', '0', '0', '0', 'x', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', 'x'],
    #                     ['0', '0', '0', '0', '0', '0', '0', '0', 'x', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', 'x'],
    #                     ['0', '0', '0', '0', '0', '0', '0', '0', 'x', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', 'x'],
    #                     ['x', '.', '.', '0', '0', '0', '0', '0', 'x', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', 'x'],
    #                     ['x', '.', '.', '.', '.', '0', '0', '0', 'x', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', 'x'],
    #                     ['x', '.', '.', '.', '.', '.', '.', '.', 'x', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', 'x'],
    #                     ['x', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', 'x'],
    #                     ['x', '.', '0', '.', 'x', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', 'x'],
    #                     ['0', '.', '.', '.', '0', '.', '.', '.', '.', '.', 'シ', '.', '.', '.', '.', '.', '.', '.', '.', 'x'],
    #                     ['x', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', 'x'],
    #                     ['x', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', 'x'],
    #                     ['x', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', 'x'],
    #                     ['x', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', 'x'],
    #                     ['x', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', 'x'],
    #                     ['x', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', 'x'],
    #                     ['x', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', 'x'],
    #                     ['x', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', 'x'],
    #                     ['x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x']])
    
    position = np.where(binary == 'ﾂ')
    return position

def select_route(binary):
    position = get_current_pos(binary)
    plausible_pos = unseen_neighbors(binary)
    nearest = min_dist(plausible_pos, position)
    print(nearest)
    walkable = check_path(position, nearest, binary)
    print("WALKABLE", walkable)
    return nearest, walkable        


def main():
    np.set_printoptions(linewidth=100, legacy="1.13")
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