import rclpy
from rclpy.node import Node

class HelloRover(Node):
    def __init__(self):
        super().__init__('hello_rover')
        self.create_timer(1.0, self.tick)

    def tick(self):
        self.get_logger().info("Hello from SlugBot!")

def main():
    rclpy.init()
    rclpy.spin(HelloRover())
    rclpy.shutdown()
