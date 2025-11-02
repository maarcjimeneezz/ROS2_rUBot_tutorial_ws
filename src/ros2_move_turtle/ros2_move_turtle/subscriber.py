import rclpy # biblioteca Python para ROS2
from rclpy.node import Node # Clase para crear NODEs
from geometry_msgs.msg import Twist

class MinimalSubscriber(Node): # Definició de la classe MinimalSubscriber
    def __init__(self):
        super().__init__('turtlesim') # creació NODE turtlesim
        self.subscription = self.create_subscription(
            Twist,
            '/turtle1/cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: linear.x={msg.linear.x}, angular.z={msg.angular.z}')

def main(args=None):
    rclpy.init(args=args) # inicio ROS2
    minimal_subscriber = MinimalSubscriber() # creo NODE MinimalSubscriber
    rclpy.spin(minimal_subscriber) # mantiene el nodo activo y escuchando mensajes
    minimal_subscriber.destroy_node() # destruye NODE al acabar
    rclpy.shutdown() # apaga ROS2 al finalizar

if __name__ == '__main__':
    main()