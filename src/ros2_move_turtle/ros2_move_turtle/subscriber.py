import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('turtlesim_subscriber')
        
        # Suscripción a la pose de la tortuga
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.listener_callback,
            10)
        
        # Publisher para detener la tortuga
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def listener_callback(self, msg):
        twist = Twist()
        
        # Condición para detener la tortuga
        if msg.x >= 7.0 or msg.y >= 7.0:
            twist.linear.x = 0.0
            self.publisher_.publish(twist)
            self.get_logger().info(f'Stopping turtle at x={msg.x:.2f}, y={msg.y:.2f}')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
