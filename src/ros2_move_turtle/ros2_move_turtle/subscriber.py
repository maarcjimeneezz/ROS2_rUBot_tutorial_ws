import rclpy # biblioteca Python para ROS2
from rclpy.node import Node # Clase para crear NODEs
from std_msgs.msg import String # Tipus de missatge per a cadenes de text a ROS2

class MinimalSubscriber(Node): # Definició de la classe MinimalSubscriber
    def __init__(self):
        super().__init__('turtlesim') # creació NODE turtlesim
        self.subscription = self.create_subscription(
            String,
            '/turtle1/cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg): # Cada vez que llega un mensaje al topic, se llama a esta función
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args) # inicio ROS2
    minimal_subscriber = MinimalSubscriber() # creo NODE MinimalSubscriber
    rclpy.spin(minimal_subscriber) # mantiene el nodo activo y escuchando mensajes
    minimal_subscriber.destroy_node() # destruye NODE al acabar
    rclpy.shutdown() # apaga ROS2 al finalizar

if __name__ == '__main__':
    main()