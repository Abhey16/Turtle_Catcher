import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64

class NumberPublisher(Node):
    
    def __init__(self):
        super().__init__('number_publisher')

        self.publisher_ = self.create_publisher(Int64,'number',10)

        self.create_timer(0.5,self.number_callback)

        self.get_logger().info("publisher_created")


    def number_callback(self):

        msg = Int64()
        msg.data = 10

        self.publisher_.publish(msg)


def main(args =None):
    rclpy.init(args=args)

    node = NumberPublisher()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ =="__main__":
    main()
