import rclpy
from rclpy.node import Node

from my_robot_interfaces.msg import HardwareStatus

class HardwareStatusPublisher(Node):

    def __init__(self):

        super().__init__('hw_status_publisher')

        self.publisher_ = self.create_publisher(HardwareStatus,'hw_status',10)

        self.create_timer(0.5,self.status_publish)

        self.get_logger().info("publisher created")

    def status_publish(self):
        msg = HardwareStatus()

        msg.temperature = 50
        msg.are_motor_ready = True
        msg.debug_message = "Hardware Status"

        self.publisher_.publish(msg)

def main(args =None):

    rclpy.init(args=args)
    node = HardwareStatusPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()




