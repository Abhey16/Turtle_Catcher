#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from functools import partial
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClient(Node):

    def __init__(self):

        super().__init__('add_to_inits_client')

        self.call_add_two_ints_server(3,7)


    def call_add_two_ints_server(self,a,b):
    
        client = self.create_client(AddTwoInts,'add_two_ints')

        while not client.wait_for_service(1):
            self.get_logger().warn("Waiting for the server")

        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_response,a=a,b=b))

    def callback_response(self,future,a,b):

        try:
            response = future.result()
            self.get_logger().info(f"{a} + {b} = {response.sum}")

        except Exception as e:
            self.get_logger().error(f"Service failed -- {e}")
        

def main(args = None):

    rclpy.init(args=args)
    node = AddTwoIntsClient()

    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()