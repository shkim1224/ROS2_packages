#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
#from example_interfaces.msg import Int64
import random
class TemperatureSensorNode(Node):
    def __init__(self):
        super().__init__("temperature_sensor")
        self.temperature_publisher_ = self.create_publisher(
            String, "topic", 10)
        self.temperature_timer_ = self.create_timer(
            1.0, self.publish_temperature)
    def publish_temperature(self):
        temperature = str(random.randint(20, 50))
        msg = String()
        msg.data = temperature
        print(msg.data)
        self.temperature_publisher_.publish(msg)
def main(args=None):
    rclpy.init(args=args)
    node = TemperatureSensorNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()
