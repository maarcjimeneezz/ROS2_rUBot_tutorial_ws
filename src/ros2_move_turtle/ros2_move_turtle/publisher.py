import rclpy
# import the Node module from ROS2 Python library
from rclpy.node import Node

from geometry_msgs.msg import Twist # Using twist function

class MoveTurtle(Node):

    def __init__(self):
        super().__init__('move_turtle') # Initialize move_turtle node
        
        # Publishing /turtle1/cmd_vel to subscriber node
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 2.0      # Turtle moves forward
        msg.angular.z = 1.0     # Turtle starts rotating
        
        self.publisher_.publish(msg) 
        self.get_logger().info('Publishing velocity')


def main(args=None):
    rclpy.init(args=args)
    node = MoveTurtle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()