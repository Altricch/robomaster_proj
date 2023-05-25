import rclpy
from rclpy.node import Node
import tf_transformations
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import sys
import math
import time
import random


# TODO: 
# - ensure that the angle after mapping is 0
# - pid to target position
# - stress test with various rooms
# - embellish map maybe with 3d stuff



class RobomasterNode(Node):
    def __init__(self):
        super().__init__('robomaster_node')

        # self.range_l = -1.0
        # self.range_r = -1.0
        # self.range_b = -1.0
        self.range_limit = 5.0 + 0.15   # range limit plus distance from ICR
        self.scaling = 20               # grid size
        self.speed_damper = 5.0         # the higher the slower

        self.discrete = 0.2             # computation distance of walkable points

        self.test = 1

        self.initial_pose = None
        self.current_pose = None
        self.target_pose = None

        self.tranlsations = []          # list of triples (last deltax, last deltay, movementtype)
        
        self.target_approach = None
        self.current_map = 0

        self.resetting = False          #If we have to backtrack to earlier mapping position

        ### CHRIS
        self._delt_target_pose = None

        self.xtrav = True
        self.ytrav = True
        ###

        self.counter = 0
        self.range_f = -1
        self.distance_travelled = 0
        self.previous_angle = 0         # previous yaw in rad
        self.spins = 0                  # how many full spins the robot has done
        self.points = []                # points list saved as [range, yaw]
        
        self.global_visited_points = []
        self.global_wall_points = []

        self.state = "scanning"

        # Key: state, Val: velocities [lin x, lin y, ang z]
        self.state_dict = {"scanning":[0.0, 0.0, 1.0],
                               "done":[ 0.0, 0.0, 0.0],
                                "map":[ 0.0 , 0.0, 0.0],
                               "move":[ 0.0, 1.0, 0.0],
                               "target": [0.0,0.0,0.0],
                               "target_def": [0.0,0.0,0.0],
                               "stop": [0.0,0.0,0.0]}

        self.odom_pose = None
        self.odom_velocity = None

        self.vel_publisher = self.create_publisher(
            Twist, '/RM0001/cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(
            Odometry, '/RM0001/odom', self.odom_callback, 10)

        # Get sensor data
        self.proximity_f = self.create_subscription(
            Range, '/RM0001/range_0', self.prox_callback_f, 10)
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
        self.range_f = self.range_limit if msg.range == 10.0 else msg.range + 0.15

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
            self.xtrav = True
            self.ytrav = True

            angle = self.current_pose[2]*180 / \
                np.pi if self.current_pose[2] > 0 else 360 + \
                self.current_pose[2]*180/np.pi
            angle = round(angle + (self.spins * 360.00), 2)
            cmd_vel.angular.z = rot_step

            if self.counter % 10 == 0 and self.range_f > 0:
                # self.get_logger().info(
                #     'angle:' + str(angle) + ' | range:' + str(round(self.range_f, 2)))
                self.points.append([self.range_f, self.current_pose[2]])

            if abs(angle - self.previous_angle) > 180 and self.counter > 500:  # 5 seconds minimum
                self.spins += 1
                self.state = "done"
                cmd_vel.angular.z = 0.0

            self.previous_angle = angle

        self.vel_publisher.publish(cmd_vel)

    def timer_callback(self):
        self.counter += 1

        self.rotate_360(0.2)

        cmd_vel = Twist()

        if self.state == 'done':

            if len(self.points) >= 2:
                self.compute_all()
            else:
                print("ERROR, NOT ENOUGH POINTS")

        ### CHRIS
        if self.state == "target_def":
            
            sx, sy, t = self.current_pose

            print("theta", t)

            d_x = self.delt_target_pose[0]
            d_y = self.delt_target_pose[1]

            fx = sx + d_x
            fy = sy + d_y

            self.target_pos = fx, fy, 0

            self.state = "target"

            #self.tranlsations.append((d_x, d_y, self.target_approach))

        if self.state == "target":
            sx, sy, _ = self.current_pose
            dir_x = np.sign(self.delt_target_pose[0])
            dir_y = np.sign(self.delt_target_pose[1])

            if self.counter % 1000 == 0:
                self.get_logger().info(
                    'Sx:' + str(sx) + ' | Sy:' + str(sy) + 
                    ' Fx ' + str(self.target_pos[0]) + " Fy" + str(self.target_pos[1]))
                print(" DIR X ", dir_x)
                print(" DIR Y", dir_y)

            if self.target_approach == "UT":
            ### THIS IS FOR UPPER TRIANGULAR
                if abs(self.target_pos[1] - sy) > 0.02 and self.ytrav:
                    self.state_dict["target"] = [0.0, dir_y, 0.0]
                elif abs(self.target_pos[0] - sx) > 0.02 and self.xtrav:
                    self.ytrav = False
                    self.state_dict["target"] = [dir_x, 0.0, 0.0]
                else:
                    self.xtrav = False
                    self.state = "scanning" #if self.resetting == False else "done"
                    self.resetting = False

                    self.points = []
                    self.initial_pose = None
                    self.counter = 0
            elif self.target_approach == "LT":
                ### THIS IS FOR LOWER TRIANGULAR
                if abs(self.target_pos[0] - sx) > 0.02 and self.xtrav:
                    self.state_dict["target"] = [dir_x, 0.0, 0.0]
                elif abs(self.target_pos[1] - sy) > 0.02 and self.ytrav:
                    self.xtrav = False
                    self.state_dict["target"] = [0.0, dir_y, 0.0]
                else:
                    self.ytrav = False
                    self.state = "scanning" #if self.resetting == False else "done"
                    self.resetting = False

                    self.points = []
                    self.initial_pose = None
                    self.counter = 0

        cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z = np.array(
            self.state_dict[self.state])/self.speed_damper

        self.vel_publisher.publish(cmd_vel)

    # Populate all visited points and all wall points. Discretizes basd on a factor
    def pop_visited_wall_p(self):
        
        x0, y0, _ = self.current_pose

        min_x = 0
        max_x = 0
        min_y = 0
        max_y = 0

        visited = []
        wall = []

        for dist, theta in self.points:
    
            x1 = x0 + dist * np.cos(theta)
            y1 = y0 + dist * np.sin(theta)

            if dist < self.range_limit:
                wall.append([x1, y1])
                self.global_wall_points.append([x1, y1])

            map_len_const = self.discrete  # 0.25
            step = map_len_const
            while (step < dist):
                x_mid = x0 + step * np.cos(theta)
                y_mid = y0 + step * np.sin(theta)
                visited.append([x_mid, y_mid])
                self.global_visited_points.append([x_mid, y_mid])
                step += map_len_const

            if x1 > max_x:
                max_x = round(x1, 2)
            elif x1 < min_x:
                min_x = round(x1, 2)

            if y1 > max_y:
                max_y = y1
            elif y1 < min_y:
                min_y = y1

        self.state = 'map'

        print("X min:" + str(min_x) + " | max:" + str(max_x))
        print("Y min:" + str(min_y) + " | max:" + str(max_y))

        return max_x, min_x, max_y, min_y, visited, wall

    # given offset given point coordinates by a given amount
    def comp_offset(self, min_x, min_y, points, scale):
        max_x = 0
        max_y = 0
        # IN FUTURE MIGHT NEED TO DEAL WITH NEGATIVE MAX VALS
        offset_points = []

        for point in points:
            x, y = point

            # translate points into the positive quadrant
            x += min_x
            y += min_y

            # scale points into coordinate system
            x /= scale
            y /= scale

            x = int(round(x,0))
            y = int(round(y,0))

            # TOBE FIXED LATER
            # if x >= self.scaling:
            #     print(x, "x out of range")
            #     x = self.scaling - 1
            # if y >= self.scaling:
            #     print(y, "y out of range")
            #     y = self.scaling - 1
            if x > max_x:
                max_x = x
            if y > max_y:
                max_y = y

            offset_points.append((x, y))

        return offset_points, max_x, max_y

    # Populates a grid with cumulative statistics according to discretized votes
    # for both wall points (postiive values) and visited points (negative values)

    def pop_grid(self, wall_offset, visited_offset, max_x, max_y, square_grid = False):

        if square_grid:
            square_dimension = max(max_x, max_y) +1
            grid = np.zeros((square_dimension, square_dimension))
        else:
            grid = np.zeros((max_y+1, max_x+1))


        for point in wall_offset:
            x, y = point            # inverted points to conform to array logic
            grid[y][x] += 1

        internal_only = set(visited_offset).difference(set(wall_offset))

        for point in visited_offset:
            if point in internal_only:
                x, y = point  # inverted points to conforn to array logic
                grid[y][x] -= 1

        return grid.astype(int)

    # takes the cummulative grid and transforms it into an binary representation.
    # "x" if the point is a wall, "." if the point has been visited and is walkable. 0 otherwise.
    # Lastly, we cap the cumulative probability for both states according to observations
    def pop_binary_grid(self, acc_grid, x0, y0, cap_wall=2, cap_visited=-2):
        # binary_grid = np.where(acc_grid >= cap_wall, '□', (np.where(acc_grid <= cap_visited, '·', '?')))
        # binary_grid[binary_grid.shape[0] - y0, x0] = '웃' # This is us

        binary_grid = np.where(acc_grid >= cap_wall, 'x',
                               (np.where(acc_grid <= cap_visited, '.', '0')))
        binary_grid[binary_grid.shape[0] - y0, x0] = 'ﾂ'  # This is us

        return binary_grid
    
    def map_plot(self, points, ax, marker, color):
        points = np.array(points)
        x = points[:, 0]
        y = points[:, 1]
        ax.scatter(x, y, marker=marker, color=color)

    def compute_all(self):
        reverting_pos = False

        max_x, min_x, max_y, min_y, visited_points, wall_points = self.pop_visited_wall_p()
        x0, y0, _ = self.initial_pose
        
        _, (ax1, ax2) = plt.subplots(2, 1, figsize=(5, 10))
        ax1.scatter(x0, y0, marker='D')
        ax2.scatter(x0, y0, marker='D')
        
        #plot of the map      
        self.map_plot(visited_points, ax1, marker='.', color="silver")
        self.map_plot(wall_points, ax1, marker='+', color="lightcoral")
        
        self.map_plot(self.global_visited_points, ax2, marker='.', color="gray")
        self.map_plot(self.global_wall_points, ax2, marker='+', color="red")

        # Offset points (put them in a square box)
        x_delta = max_x - min_x
        y_delta = max_y - min_y
        
        min_x = abs(min_x)
        min_y = abs(min_y)
        
        abs_delta = x_delta if x_delta > y_delta else y_delta
        
        scale = abs_delta/self.scaling

        # Compute offset for wall points and visited points
        wall_offset, max_x_w, max_y_w = self.comp_offset(
            min_x, min_y, self.global_wall_points, scale)
        
        visited_offset, max_x_v, max_y_v = self.comp_offset(
            min_x, min_y, self.global_visited_points, scale)

        # DISCRETIZE INITIAL COORDS
        x0 = int(round((x0 + min_x)/scale, 0))
        y0 = int(round((y0 + min_y)/scale, 0))

        # Max index observed for grid creation
        max_x = max(max_x_v,max_x_w)
        max_y = max(max_y_v,max_y_w)

        # Get cumulative grid with votes
        grid = self.pop_grid(wall_offset, visited_offset, max_x, max_y, square_grid=True)

        # Flip the grid and print cummulative grid
        corrected_grid = np.flip(grid, axis=0)
        print(corrected_grid)

        # Get binary grid and print
        binary_grid = self.pop_binary_grid(corrected_grid, x0, y0)

        # print(binary_grid)
        print(np.array2string(binary_grid, separator=' ',
              formatter={'str_kind': lambda x: x}))

        # check how many points are identified as wall
        coords = np.argwhere(binary_grid == 'x')
        print("wall coords amount:", len(coords))

        ### GETTING CLOSEST ####
        result = select_route(binary_grid)
        if type(result) is not type("hello"):
            nearest, position, walkable, vertical_delta, horizontal_delta = result
        else:

            ### New solution, go back to previous pos
            ### once there do not map but simply search for new candidates
            ### if no more previous poses available or reachable, stop node
            if self.tranlsations is not []:
                self.resetting = True
                print("NO REACHABLE CANDIDATE, REVERTING TO PREVIOUS MAPPING POSE")
                position = get_current_pos(binary_grid)
                tx, ty, approach = self.tranlsations.pop()
                #INVERSION of direction
                dfx = -tx
                dfy = -ty
                self.target_approach = "LT" if approach == "UT" else "UT"
                self.delt_target_pose = (dfx , dfy, 0)
                
                print("in world scale")
                print("DFX", dfx)
                print("DFY", dfy)
                print("approach", self.target_approach)

                ax1.set_title("Current map " + str(self.current_map))
                ax2.set_title("Combined map " + str(self.current_map))
                self.current_map += 1
                plt.ion()
                plt.show(block = False)
                plt.pause(interval = 2)

                print("now after block")

                self.state = "target_def"
                return 

            else:
                print("NO REACHABLE POINTS, NODE IS STOPPED")
                self.state = "stop"
                ax1.set_title("Current map " + str(self.current_map))
                ax2.set_title("Combined map " + str(self.current_map))
                self.current_map += 1
                plt.ion()

                #animation = FuncAnimation(fig, update, frames=len(self.points), interval=200, blit=True)

                plt.show(block = False)
                plt.pause(interval = 2)
                return


        # inverted x & y
        fy, fx = nearest
        sy, sx = position
        #########################

        case1 = walkable[0] # Move vertically first then horizontally
        case2 = walkable[1] # Move horizontally first then vertically
        case3 = walkable[2] # Move diagonally first then either vertically or horizontally

        if case1:
            self.target_approach = "UT"
        elif case2:
            self.target_approach = "LT"
        else:
            self.target_approach = "Diag"
            print("DIAGONAL APPROACH NOT IMPLEMENTED YET")
            self.target_approach = "UT"

        # self.state = 'move'
        ax1.set_title("Current map " + str(self.current_map))
        ax2.set_title("Combined map " + str(self.current_map))
        self.current_map += 1
        plt.ion()
        plt.show(block = False)
        plt.pause(interval = 2)

        print("now after block")
        #self.state = 'move'

        print("inversion of x and y")
        print("Start X ", sx , " Start Y ", sy)
        print("Target X ", fx , " Target Y ", fy)
        print("SCALE FACTOR", scale)

        # Inverted
        dfx = horizontal_delta * scale 
        dfy = -vertical_delta * scale

        self.tranlsations.append([dfx,dfy,self.target_approach])

        print("in world scale")
        print("DFX", dfx)
        print("DFY", dfy)

        print("target pose in world coordinates")
        print("with respect to our current pose")
        print("x", self.current_pose[0] + dfx)
        print("y", self.current_pose[1] + dfy)

        #this might indicate how much to travel but not the end position
        self.delt_target_pose = (dfx , dfy, 0)

        self.state = "target_def"


# Retrieves all plausible candidates, e.g. that have a 0 neighbor and a reachable neighbor.
def unseen_neighbors(binary):
    unseen = np.where(binary == "0")
    x, y = unseen
    plausible_pos = []
    for elem in zip(x, y):
        if candidate(binary, elem):
            plausible_pos.append(elem)
    # print("Plausible candidates are", plausible_pos)
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
def candidate(binary, elem):

    max_x = len(binary)
    min_x = 0
    max_y = len(binary)
    min_y = 0
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
        elems.append((x+1,y-1))
        elems.append((x-1,y+1))

    if len(elems) == 0:
        return False

    for el in elems:
        x_t, y_t = el
        if binary[x_t, y_t] == "0":
            in_0_neihborhood = True
        if binary[x_t, y_t] == ".":
            in_reach_neighborhood = True
        if binary[x_t, y_t] == "x":
            return False

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
        if binary[sx, sy] == "x":
            UT = False

    if UT:
        while sy != fy:
            sy += dy
            elem = binary[sx, sy]
            if elem == "x":
                UT = False

    # LTriangular
    sx = csx
    sy = csy
    while sy != fy:
        sy += dy
        elem = binary[sx, sy]
        if elem == "x":
            LT = False
    if LT:
        while sx != fx:
            sx += dx
            # CMD.Velangular.x
            if binary[sx, sy] == "x":
                LT = False

    # Diagonal
    sx = csx
    sy = csy
    while sy != fy and sx != fx:
        if sy != fy:
            sy += dy
        if sx != fx:
            sx += dx
        elem = binary[sx, sy]
        if elem == "x":
            Diagonal = False

    return UT, LT, False # Diagonal not implemented

# Retrieves from the array the position we currenly have
def get_current_pos(binary):
    px,py = np.where(binary == 'ﾂ')
    position = (px[0],py[0]) # array[int] -> int
    return position

def get_known(point, binary):
    x, y = point
    neighbours =  [(x+1,y), (x-1,y), (x,y+1), (x,y-1), (x+1,y+1), (x-1,y-1), (x+1,y-1), (x-1,y+1)]
    for elem in neighbours:
        if binary[elem[0], elem[1]] == '.':
            return elem


def select_route(binary):
    position = get_current_pos(binary)

    keep_looping = True
    plausible_pos = unseen_neighbors(binary)
    if len(plausible_pos) == 0:
        print("WE HAVE MAPPED EVERYTHING")
    else:
        while keep_looping:
            
            nearest = min_dist(plausible_pos, position)

            nearest_known = get_known(nearest, binary)

            walkable = check_path(position, nearest_known, binary)

            if np.all(np.asarray(walkable) == False):
                print("KEEP SEARCHING CANDIDATES")
                plausible_pos.remove(nearest)
                if len(plausible_pos) == 0:
                    print("WE HAVE MAPPED EVERYTHING")
                    return "Mapped_All"
            else: 
                print("STOP LOOPING: CANDIDATE FOUND")
                keep_looping = False


        binary[nearest_known] = '◎'
        print(np.array2string(binary, separator=' ',
                formatter={'str_kind': lambda x: x}))
        
        dx = nearest_known[0]-position[0]
        dy = nearest_known[1]-position[1]

        # THIS FIXES DISCRETIZATION
        # BASED ON THE SSUMPTION THAT NUMBERS 
        # HAVE BEEN ROUNDED DOWN EARLIER
        dx += np.sign(dx)
        dy += np.sign(dy)

        print()
        print("array coordinates")
        print("OUR POSITION ﾂ:", position)
        print("NEAREST POSITION ◎:", nearest_known)
        print("WALKABLE | UT:", walkable[0], " | LT:", walkable[1], " | Diag:", walkable[-1])
        print("VERTICAL TRASLATION (rounded up):", dx)
        print("HORIZONTAL TRASLATION (rounded up):", dy)
        return nearest_known, position, walkable, dx, dy
    
    return "Mapped_All"


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
