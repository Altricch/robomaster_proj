import rclpy
from rclpy.node import Node
import tf_transformations
from std_msgs.msg import String

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range

import sys

class RobomasterNode(Node):
    def __init__(self):
        super().__init__('robomaster_node')

        self.range_f = -1
        self.range_l = -1
        self.range_r = -1
        self.range_b = -1
        
        # Create attributes to store odometry pose and velocity
        self.odom_pose = None
        self.odom_velocity = None
                
        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, '/RM0001/cmd_vel', 10)

        # Create a subscriber to the topic 'odom', which will call 
        # self.odom_callback every time a message is received
        self.odom_subscriber = self.create_subscription(Odometry, '/RM0001/odom', self.odom_callback, 10)

        # Get sensor data
        self.proximity_f = self.create_subscription(Odometry, '/RM0001/range_0', self.prox_callback_f, 10)
        self.proximity_l = self.create_subscription(Odometry, '/RM0001/range_1', self.prox_callback_l, 10)
        self.proximity_r = self.create_subscription(Odometry, '/RM0001/range_2', self.prox_callback_r, 10)
        self.proximity_b = self.create_subscription(Odometry, '/RM0001/range_3', self.prox_callback_b, 10)


        self.timer = self.create_timer(1, self.timer_callback)
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
        
        # self.get_logger().info(
        #     "odometry: received pose (x: {:.2f}, y: {:.2f}, theta: {:.2f})".format(*pose2d),
        #      throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        # )


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

    def timer_callback(self):
        
        t_circle = 96

        cmd_vel = Twist() 
        
        if self.timer_counter < t_circle // 2:
            cmd_vel.linear.x  = 0.2 # [m/s]
            cmd_vel.angular.z = 0.2 # [rad/s]
        else:
            cmd_vel.linear.x  = 0.2 # [m/s]
            cmd_vel.angular.z = -0.2 # [rad/s]

        self.timer_counter += 1
        if self.timer_counter > t_circle:
            self.timer_counter = self.timer_counter % t_circle

        # self.get_logger().info(
        #     "in timed function, counter is: {:.2f})".format(self.timer_counter),
        #      throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        # )

        self.get_logger().info(
            "Range front: {:.2f}".format(self.range_f),
             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        )

        self.get_logger().info(
            "Range left: {:.2f}".format(self.range_l),
             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        )

        self.get_logger().info(
            "Range right: {:.2f}".format(self.range_r),
             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        )

        self.get_logger().info(
            "Range back: {:.2f}".format(self.range_b),
             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        )

        self.vel_publisher.publish(cmd_vel)


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
