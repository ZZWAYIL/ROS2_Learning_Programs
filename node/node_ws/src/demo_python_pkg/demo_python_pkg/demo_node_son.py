import rclpy
from rclpy.node import Node

class Son(Node):
    def __init__(self, node_name: str ,name: str,age : int):
        super().__init__(node_name)
        self.node_name = node_name
        self.name = name
        self.age = age
    
    def eat(self, food: str):
        # return f"{self.name} is eating {food}."
        self.get_logger().info(f"{self.name} is eating {food}.")


        
    
def main():
    rclpy.init()
    son = Son("eat_node", "ZZW", 20)
    son.eat("fish")
    rclpy.spin(son)
    rclpy.shutdown()