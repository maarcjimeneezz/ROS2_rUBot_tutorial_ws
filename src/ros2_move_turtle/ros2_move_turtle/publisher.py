import rclpy
from rclpy.node import Node # import the Node module from ROS2 Python library
from geometry_msgs.msg import Twist # Using twist function

class MoveTurtle(Node): # definició de la classe MoveTurtle
    def __init__(self):
        super().__init__('move_turtle') # Initialize move_turtle NODE
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10) # Publishing /turtle1/cmd_vel to subscriber node
        self.timer = self.create_timer(0.5, self.timer_callback) # Wait 0.5 s between

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 10.0      # Turtle moves forward 
        msg.angular.z = 1.0     # Turtle starts rotating
        self.publisher_.publish(msg) # Publicar missatge
        self.get_logger().info('Publishing velocity') # imprimir missatge al terminal

def main(args=None): # funció principal que executa tot
    rclpy.init(args=args) # inicialitzar ROS2
    node = MoveTurtle() # Crea instancia del NODE
    rclpy.spin(node) # Manté NODE actiu
    node.destroy_node() # Destrueix NODE al finalitzar
    rclpy.shutdown() # Tanca ROS2

if __name__ == '__main__':
    main()