import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('turtlesim_subscriber')
        
        # Subscriber to the turtle's position (Pose messages)
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.listener_callback,
            10)
        
        # Publisher that sends Twist messages to control the turtle's velocity
        # Used here to stop the turtle when needed
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def listener_callback(self, msg):
        # This function is executed every time a new Pose message is recieved
        twist = Twist()
        
        # If the turtle's position exceeds the limit (x or y bigger/equal than 7)
        # publish a message to stop the movement
        if msg.x >= 7.0 or msg.y >= 7.0:
            twist.linear.x = 0.0
            self.publisher_.publish(twist)
            # Log the stop event with the current coordinates
            self.get_logger().info(f'Stopping turtle at x={msg.x:.2f}, y={msg.y:.2f}')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# EXPLANATION: This script it subscribes to the topic /turtle1/pose to constantly read
# the turtle's position. Every time a new position message is recieved, the node checks
# whether the x or y coordinate is greater or equal than 7.0.
# If that happens, it publishes a Twist message which stops the turtle.
# When this node runs alongside the Turtlesim simulator, it continuously monitors the
# turtle's movement and automatically stops it when it reaches the limit.
