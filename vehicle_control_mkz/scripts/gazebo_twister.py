#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

class ROS2Twist(Node):
    def __init__(self) -> None:
        super().__init__('multi_car_cmd')

        #self.pub_mkz_cmd = self.create_publisher(Twist, '/vehicle/cmd_vel', 1)
        self.pub_mkz_enable = self.create_publisher(Empty, '/vehicle/enable', 1)
        print("VEHICLE_ENABLE")
        self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        #cmd = Twist()
        #cmd.linear.x = 4.0
        #cmd.angular.z = 0.1
        #self.pub_mkz_cmd.publish(cmd)
        self.pub_mkz_enable.publish(Empty())


def main(args=None):
    rclpy.init(args=args)
    node_instance = ROS2Twist()
    rclpy.spin(node_instance)

if __name__ == '__main__':
    main()
