# Importing required libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry as Odometry
from time import sleep
from tf_transformations import euler_from_quaternion
import math

# Setting maximum difference
max_difference = 0.1

# Defining class TurtleController
class TurtleController(Node):
    
    def __init__(self, control_period=0.05):
        super().__init__('turtlecontroller')
        
        # Initializing variables
        self.array_position = 0
        self.odometry = Odometry()
        self.positions = [[2.0, 2.0],[0, 0]]
        self.set_point_x = self.positions[self.array_position][0]
        self.set_point_y = self.positions[self.array_position][1]
        
        # Creating publisher and subscriber objects
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.pose_callback, 10)
        self.control_timer = self.create_timer(timer_period_sec = control_period, callback = self.control_callback)

    # Callback function for position and orientation data
    def pose_callback(self, msg):
        position_x = msg.pose.pose.position.x
        position_y = msg.pose.pose.position.y

        ang = msg.pose.pose.orientation
        _, _, self.theta = euler_from_quaternion([ang.x, ang.y, ang.z, ang.w])

        # Updating actual pose
        self.actual_pose_x = position_x
        self.actual_pose_y = position_y
        self.get_logger().info(f"Position X ={round(position_x, 2)}, Position Y={round(position_y, 2)}, Theta={round(math.degrees(self.theta), 2)}")

    # Callback function for controlling movement
    def control_callback(self):
        msg = Twist()
        self.set_point_x = self.positions[self.array_position][0]
        self.set_point_y = self.positions[self.array_position][1]
        x_difference = self.set_point_x - self.actual_pose_x
        y_difference = self.set_point_y - self.actual_pose_y

        # Calculating angle needed to reach the set point
        angle_needed = math.atan2(y_difference, x_difference)
        theta_difference = angle_needed - self.theta

        # Printing the difference in x, y and theta
        print(f"X Difference={round(abs(x_difference), 2)}, Y Difference={round(abs(y_difference), 2)}, theta_difference={round(abs(theta_difference), 2)}")

        # Checking if the set point is reached
        if abs(x_difference) <= max_difference and abs(y_difference) <= max_difference:
            if self.array_position == len(self.positions) - 1:
                # Stopping the turtlebot and printing success message
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.publisher.publish(msg)
                self.get_logger().info("Cheguei no meu destino final.")
                exit()
            else:
                # Updating the set point
                self.array_position += 1

        # Turning the turtlebot
        if abs(theta_difference) >= (max_difference -0.05):
            msg.linear.x = 0.0
            msg.angular.z = 0.2 if theta_difference > 0 else -0.2

        # Moving the turtlebot forward
        elif abs(x_difference) >= max_difference:
            msg.linear.x = 0.2

        # Publishing the movement message
        self.publisher.publish(msg)

# Define a main function that takes an optional argument 'args'
def main(args=None):
    # Initialize the ROS 2 client library with the provided arguments
    rclpy.init(args=args)
    
    # Create an instance of the TurtleController class
    tc = TurtleController()
    
    # Start spinning the node to process incoming messages
    rclpy.spin(tc)
    
    # Destroy the node and free up resources
    tc.destroy_node()
    
    # Shutdown the ROS 2 client library
    rclpy.shutdown()

# Call the main function if this script is executed directly
if __name__ == "__main__":
    main()