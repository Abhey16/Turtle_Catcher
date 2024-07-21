import rclpy
from rclpy.node import Node

from example_interfaces.srv import SetBool

from functools import partial

import rclpy.node

class Reset_number_count(Node):

    def __init__(self):

        super().__init__('reset_number_count')

        self.callback_reset_request(True)

    def callback_reset_request(self,data):

        client = self.create_client(SetBool,'reset_counter')

        while not client.wait_for_service(1):
            self.get_logger().warn("serve not available")

        request = SetBool.Request()
        request.data = data

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_response,data = data))

    def callback_response(self,future,data):
        
        try:
            response = future.result()
            self.get_logger().info(f"{response.success}")
        except Exception as e:
            self.get_logger().error(f"{e}")

def main():
    rclpy.init()
    node = Reset_number_count()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()