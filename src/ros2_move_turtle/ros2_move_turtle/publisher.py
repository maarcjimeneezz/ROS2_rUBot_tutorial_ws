import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class MoveTurtle(Node):
    def __init__(self):
        super().__init__('move_turtle')

        # Publisher para mover la tortuga
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Subscriber para recibir la posición
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Timer que publica cada 0.1s
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Inicializamos la pose
        self.pose = None

    def pose_callback(self, msg):
        self.pose = msg  # Guardamos la pose actual

    def timer_callback(self):
        # Esperamos a recibir la primera pose
        if self.pose is None:
            return

        msg = Twist()

        # Lógica para detener la tortuga al límite
        if self.pose.x >= 7.0 or self.pose.y >= 7.0:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.get_logger().info(f'Stopping turtle at x={self.pose.x:.2f}, y={self.pose.y:.2f}')
        else:
            msg.linear.x = 1.0  # Avanza recto
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

