#!usr/bin/env python3

import rclpy
from rclpy.node import Node
import math

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import AliveTurtle
from my_robot_interfaces.srv import CatchTurtle

class TurtleController(Node):

    def __init__(self):

        super().__init__('turtle_controller')

        #parameter
        self.declare_parameter("catch_closest_turtle_first",False)
        self.catch_closest = self.get_parameter("catch_closest_turtle_first").value
        self.get_logger().warn(f"{self.catch_closest}")

        #Initializing
        self.goal_x = None
        self.goal_y = None
        self.goal_theta = None

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        #creating subscriber
        self.subscriber_pose_ = self.create_subscription(Pose,'/turtle1/pose',self.callback_turtle_pose,10)
        self.subscriber_alive_ = self.create_subscription(AliveTurtle,'/alive_turtles',self.callback_turtle_alive,10)

        #creating publisher
        self.publisher_ = self.create_publisher(Twist,'/turtle1/cmd_vel',10)
        self.create_timer(0.01,self.publish_velocity)


    def callback_turtle_pose(self,msg):

        self.current_x = msg.x
        self.current_y = msg.y
        self.current_theta = msg.theta
    
    def callback_turtle_alive(self,msg):

        if msg.turtle:
            self.get_logger().warn(f"catch_closest : {self.catch_closest}")

            if self.catch_closest == True:
                closest_distance = None

                #checking the distance from every turtle
                for turtle in msg.turtle:
                    distance = math.sqrt((turtle.coordinates[0]-self.current_x)**2 + ((turtle.coordinates[1]-self.current_y)**2))

                    if closest_distance == None or distance < closest_distance:
                        #updating the closest distance
                        closest_distance = distance
                        self.goal_turtle = turtle
                        
            elif self.catch_closest == False:
                self.goal_turtle = msg.turtle[0]

            #target turtle coordinates
            self.goal_x = self.goal_turtle.coordinates[0]
            self.goal_y = self.goal_turtle.coordinates[1]
            self.goal_theta = self.goal_turtle.coordinates[2]

        else:
            self.get_logger().warn("No turtles in the alive turtles list.")

    def publish_velocity(self):

        if self.goal_x is None or self.goal_y is None or self.goal_theta is None:
            self.get_logger().info("Goal pose not set yet.")
            return
        
        msg =Twist()

        #get the error values
        heading_error,distance_error,orientation_error = self.turle_control()
        
        #gain values
        k1=2 #linear vel
        k2=6 #angular vel

        #Initializing
        pos_flag = 0
        head_flag = 0
        orient_flag =0

        #position control
        if distance_error<0.1 and distance_error>-0.1:
            msg.linear.x = 0.0
            pos_flag = 1
        else:
            msg.linear.x = k1*(distance_error)

        #heading control
        if heading_error<0.1 and heading_error>-0.1:
            msg.angular.z = 0.0
        else:
            msg.angular.z = k2*heading_error

        #orientation control
        if pos_flag == 1:
            if orientation_error<0.1 and orientation_error>-0.1:
                msg.angular.z = 0.0
                orient_flag =1
            else:
                msg.angular.z = k2*orientation_error

        #Publishing cmd vel
        self.publisher_.publish(msg)

        #if goal reached
        if pos_flag == 1 and orient_flag ==1:
            self.callback_catch_status()


    def turle_control(self):

        #position error
        x_error = self.goal_x - self.current_x
        y_error = self.goal_y - self.current_y
        distance_error = math.sqrt((x_error)**2 + (y_error)**2)

        #heading error
        heading_theta = math.atan2((self.goal_y - self.current_y),(self.goal_x - self.current_x))

        #noramlize heading_theta from (-2*pi - 2*pi) to (0 - 2*pi)
        if heading_theta <0:
            heading_theta += 2*math.pi 

        heading_error = heading_theta - self.current_theta

        #normalizing the heading error from (pi to -pi ), this will ensure shortest turning angle
        if heading_error >=math.pi:
            heading_error -= 2*math.pi

        elif heading_error < -math.pi:
            heading_error += 2*math.pi

        #orientation error
        orientation_error = self.goal_theta-self.current_theta

        return heading_error,distance_error,orientation_error
    
    def callback_catch_status(self):

        client =self.create_client(CatchTurtle,'catch_turtle')

        #waiting for server to be available
        while not client.wait_for_service(1):
            self.get_logger().warn("catch turtle server not avialable")
        
        #creating request
        request = CatchTurtle.Request()
        request.catch_status = True
        request.name = self.goal_turtle.name

        #calling request
        future = client.call_async(request)
        future.add_done_callback(self.callback_catch_response)

    def callback_catch_response(self,future):
        try:
            response = future.result()
            self.get_logger().info(f"catch status : {response.success}")
        except Exception as e:
            self.get_logger().error(f"{e}")



#initialization ros communication
def main(args = None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()