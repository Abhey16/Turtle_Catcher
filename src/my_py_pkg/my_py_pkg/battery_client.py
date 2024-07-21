import rclpy
from rclpy.node import Node

from my_robot_interfaces.srv import SetLed

from functools import partial

class BatteryCliet(Node):
    def __init__(self):
        super().__init__('battery_client')

        self.count = 0
        self.state = True

        self.callback_battery_client(3,True)

        self.create_timer(1,self.time_counter)

        self.get_logger().info("client created")

    def time_counter(self):

        self.count +=1

        if self.count == 4 and self.state == True:
            self.callback_battery_client(3,False)
            self.count = 0
            self.state = False

        elif self.count == 6 and self.state == False:
            self.callback_battery_client(3,True)
            self.count = 0
            self.state = True

    def callback_battery_client(self,led_number,state):

        client = self.create_client(SetLed,'set_led')

        while not client.wait_for_service(1):
            self.get_logger().warn("server not available")

        request = SetLed.Request()

        request.led_number = led_number
        request.state = state

        future = client.call_async(request)

        future.add_done_callback(partial(self.callback_battery_response,led_number=led_number,state=state))

    def callback_battery_response(self,future,led_number,state):

        try:
            response = future.result()
            self.get_logger().info(f"Request: led_number:{led_number}, state:{state} Response: {response.success}")

        except Exception as e:
            self.get_logger().error({e})
    

def main(args = None):

    rclpy.init(args=args)
    node = BatteryCliet()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()


        