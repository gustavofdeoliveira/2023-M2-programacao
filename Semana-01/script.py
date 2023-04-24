# import all the required libraries
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# define the class TurtleController which inherits from the Node class
class TurtleController(Node):
    # define the constructor
    def __init__(self):
        # call the constructor of the parent class
        super().__init__('turtle_controller')
        # create a publisher object
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        # create a timer object
        self.timer_ = self.create_timer(0.1, self.move_turtle)
        # create a message object
        self.twist_msg_ = Twist()
    
    #define the function move_turtle
    def move_turtle(self):
        # define the coordinates of X, Y and Z 
        coordinatesX = [2.0, 0.0]
        coordinatesY = [2.0, 0.0]
        coordinatesZ = [ 0.0, 2.84]
        # loop through the coordinates
        for i in range(22):
            # check if the value of i is even or odd
            if i %2==0:
                # assign the values of the coordinates to the message object
                self.twist_msg_.angular.z = coordinatesZ[1]
                self.twist_msg_.linear.x = coordinatesX[1]
                self.twist_msg_.linear.y = coordinatesY[1] 
            # if the value of i is odd  
            else:
                # assign the values of the coordinates to the message object
                self.twist_msg_.angular.z = coordinatesZ[0]
                self.twist_msg_.linear.x = coordinatesX[0]
                self.twist_msg_.linear.y = coordinatesY[0] 
            # publish the message object
            self.publisher_.publish(self.twist_msg_)
            # sleep for 1 second
            time.sleep(1.0)
        # cancel the timer
        self.timer_.cancel()


# define the main function
def main(args=None):
    # initialize the rclpy library
    rclpy.init()
    # create an object of the class TurtleController
    turtle_controller = TurtleController()
    # spin the node
    rclpy.spin(turtle_controller)
    # destroy the node
    turtle_controller.destroy_node()
    # shutdown the rclpy library
    rclpy.shutdown()


if __name__ == '__main__':
    main()