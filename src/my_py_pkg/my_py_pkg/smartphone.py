import rclpy
from rclpy.node import Node

from example_interfaces.msg import String


class Smartphone(Node):
    
    def __init__(self):

        super().__init__('smartphone')

        self.subscriber_ = self.create_subscription(String,"robot_news",self.callback_robot_news,10)

    def callback_robot_news(self,msg):
        self.get_logger().info(msg.data)





def main(args = None):

    rclpy.init(args=args)

    node = Smartphone()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()