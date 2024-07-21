import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts

def main(args = None):

    rclpy.init(args= args)

    node = Node('add_two_ints_no_oop')

    client = node.create_client(AddTwoInts,'add_two_ints')

    #wait for the server node to be available within 1 second
    while not client.wait_for_service(1):
        node.get_logger().warn("waiting for server")

    request = AddTwoInts.Request()
    request.a = 3
    request.b = 8

    future = client.call_async(request)

    rclpy.spin_until_future_complete(node,future)

    try:
        response = future.result()
        node.get_logger().info(f"{request.a} + {request.b} = {response.sum}")

    except Exception as e:
        node.get_logger().error("Service call failed %r" % (e,))

    rclpy.shutdown()

if __name__ == "__main":
    main()