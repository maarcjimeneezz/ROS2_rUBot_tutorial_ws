import rclpy # biblioteca Python para ROS2
from rclpy.node import Node # Clase para crear NODEs
from geometry_msgs.msg import Pose, Twist

class MinimalSubscriber(Node): # Definició de la classe MinimalSubscriber
    def __init__(self):
        super().__init__('turtlesim') # creació NODE turtlesim
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.listener_callback,
            10)
        # publisher to stop the turtle
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def listener_callback(self, msg):
        twist = Twist()
        if msg.x > 7.0 or msg.y > 7.0:
            twist.linear.x = 0.0
            self.get_logger().info(f'Stopping turtle: x={msg.x:.2f}, y={msg.y:.2f}')
            self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args) # inicio ROS2
    minimal_subscriber = MinimalSubscriber() # creo NODE MinimalSubscriber
    rclpy.spin(minimal_subscriber) # mantiene el nodo activo y escuchando mensajes
    minimal_subscriber.destroy_node() # destruye NODE al acabar
    rclpy.shutdown() # apaga ROS2 al finalizar

if __name__ == '__main__':
    main()