import rclpy
from rclpy.node import Node

from my_robot_interfaces.srv import SetLed

from my_robot_interfaces.msg import LedStates

class LedPanelServer(Node):
    def __init__(self):
        super().__init__("led_panel_server")

        self.server_ = self.create_service(SetLed,'set_led',self.callback_set_led)

        self.get_logger().info("server_created")

        self.publisher_ = self.create_publisher(LedStates,'led_states',10)

        self.get_logger().info("publisher_created")

    def callback_set_led(self,request,response):

        msg = LedStates()

        # self.get_logger().info(f"request : {request.state}")

        if request.state == True:

            msg.led_state[0:2] = 0
            msg.led_state[2] = 1

        elif request.state == False:

            msg.led_state[0:2] = 0

        self.publisher_.publish(msg)
        # self.get_logger().info(f"{msg.led_state}")
        response.success = True
        
        return response


    
def main(args = None):

    rclpy.init(args=args)

    node = LedPanelServer()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()






