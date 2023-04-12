#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

class MultiCarCmd(Node):
    def __init__(self) -> None:
        super().__init__('multi_car_cmd')

        self.pub_fusion_cmd = self.create_publisher(Twist, '/fusion/cmd_vel', 1)
        self.pub_fusion_enable = self.create_publisher(Empty, '/fusion/enable', 1)
        self.pub_pacifica_cmd = self.create_publisher(Twist, '/pacifica/cmd_vel', 1)
        self.pub_pacifica_enable = self.create_publisher(Empty, '/pacifica/enable', 1)
        self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        cmd = Twist()
        cmd.linear.x = 2.0
        cmd.angular.z = 0.25
        self.pub_fusion_cmd.publish(cmd)
        self.pub_fusion_enable.publish(Empty())
        cmd.angular.z = -0.25
        self.pub_pacifica_cmd.publish(cmd)
        self.pub_pacifica_enable.publish(Empty())


def main(args=None):
    rclpy.init(args=args)
    node_instance = MultiCarCmd()
    rclpy.spin(node_instance)

if __name__ == '__main__':
    main()
