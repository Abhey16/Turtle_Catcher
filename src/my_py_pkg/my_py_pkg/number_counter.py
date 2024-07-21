import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64

from example_interfaces.srv import SetBool

class NumberCounter(Node):

    def __init__(self):

        super().__init__('number_counter')

        self.counter = 0

        self.subcriber_=self.create_subscription(Int64,'number',self.counter_callback,10)

        self.publisher_=self.create_publisher(Int64,'number_count',10)

        self.get_logger().info("pubisher and subscriber created")

        self.server_ = self.create_service(SetBool,'reset_counter',self.callback_reset_counter)

        self.get_logger().info("server created")

    def counter_callback(self,msg):

        counter_msg = Int64()

        if msg.data == 10:

            self.counter +=1

            counter_msg.data = self.counter
            self.get_logger().info(f"{self.counter}")

            self.publisher_.publish(counter_msg)
    
    def callback_reset_counter(self,request,response):

        if request.data == True:
            self.counter = 0

            self.get_logger().info("reset counter")
            response.success = True

        return response
        



def main(args=None):

    rclpy.init(args=args)

    node = NumberCounter()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
   
    main()