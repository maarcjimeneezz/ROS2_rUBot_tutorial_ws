import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class MoveTurtle(Node):
    def __init__(self):
        super().__init__('move_turtle')

        # Publisher to move the turtle
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Subscriber to recieve the position (Pose messages)
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Timer that triggers the callback function every 0.1s
        # to continuously check and update the turtle's movement
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Variable to store the most recent pose of the turtle
        self.pose = None

    def pose_callback(self, msg):
        # This callback is executed each time a new Pose message is recieved
        self.pose = msg  # Save the actual pose 

    def timer_callback(self):
        # Wait until the first pose message is recieved
        if self.pose is None:
            return

        # Create a new Twist message to define linear and angular velocities
        msg = Twist() 

        # If the turtle's x or y position exceeds 7s, stop the movement
        if self.pose.x >= 7.0 or self.pose.y >= 7.0:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.get_logger().info(f'Stopping turtle at x={self.pose.x:.2f}, y={self.pose.y:.2f}')
        else:
            #Otherwise, move the turtle straight forward
            msg.linear.x = 1.0  
            msg.angular.z = 0.0

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MoveTurtle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# EXPLANATION: This node (move_turtle) subscribes to the topic /turtle1/pose to read 
# the turtle's position and publishes velocity commands to /turtle1/cmd_vel
# Every 0.1s, it checks the turtle's position: if x or y is greater than 7.0,
# it stops the movement; otherwise, it keeps moving forward.
# When launched together with the Turtlesim node, the turtle moves in a straight
# line until it reaches the limit of the screen and then stops.
