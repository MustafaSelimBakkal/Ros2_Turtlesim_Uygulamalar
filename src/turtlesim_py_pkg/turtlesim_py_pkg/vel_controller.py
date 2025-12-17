#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class VelControllerNode(Node): #1
    def __init__(self):
        super().__init__("vel_controller_node") #2
        self.publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.timer_ = self.create_timer(1, self.publisher_vel)

        self.get_logger().info("Velocity Controller Node has been started")

    def publisher_vel(self):
        msg = Twist()

        msg.linear.x = 1.0
        msg.linear.y = 0.0
        msg.angular.z = 0.5 # radyan: -3.14 ile +3.14 arası
        self.publisher_.publish(msg)




def main(args=None):
    rclpy.init(args=args)
    node = VelControllerNode() #3
    rclpy.spin(node)
    rclpy.shutdown()

    

if __name__ == "__main__":
    main()