import rclpy
from rclpy.node import Node # import the Node module from ROS2 Python library
from geometry_msgs.msg import Twist # Using twist function

class MoveTurtle(Node): # definició de la classe MoveTurtle
    def __init__(self):
        super().__init__('move_turtle') # Initialize move_turtle NODE
        
        # Publishing /turtle1/cmd_vel to subscriber node
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.5 # Wait 0.5 s between
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 2.0      # Turtle moves forward 2 u/s
        msg.angular.z = 1.0     # Turtle starts rotating 1 rad/s
        
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