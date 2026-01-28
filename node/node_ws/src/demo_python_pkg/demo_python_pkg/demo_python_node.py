#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

def main():
    rclpy.init()
    node = Node("ROS2_Py_Node")
    node.get_logger().info("Hello, ROS2 with Python!")
    node.get_logger().debug("Hello, ROS2 with Python!")
    node.get_logger().warn("Hello, ROS2 with Python!")
    rclpy.spin(node)
    rclpy.shutdown()

