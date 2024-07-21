#!usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import random

from turtlesim.srv import Spawn
from turtlesim.srv import Kill

from my_robot_interfaces.msg import TurtleBot
from my_robot_interfaces.msg import AliveTurtle
from my_robot_interfaces.srv import CatchTurtle

from functools import partial

class TurtleSpawner(Node):

    def __init__(self):

        super().__init__('turtle_spawner')
        
        #parameters
        self.declare_parameter("spawn_frequency")
        self.spawn_frequency = self.get_parameter("spawn_frequency").value

        #client call
        self.create_timer(1/self.spawn_frequency,self.callback_turtle_spawner)
        self.get_logger().info("spawning service created")
        
        #create publisher
        self.publisher_ = self.create_publisher(AliveTurtle,'alive_turtles',10)

        #create server
        self.catch_server_ = self.create_service(CatchTurtle,'catch_turtle',self.callback_catch_status)

        self.msg = AliveTurtle()

    def callback_turtle_spawner(self):

        client_ = self.create_client(Spawn,'/spawn')

        #wait for server to be available
        while not client_.wait_for_service(1):
            self.get_logger().info('server not available')

        #get random poses
        x,y,theta = self.random_pose()

        #create service request
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta

        #calling request
        future = client_.call_async(request)
        future.add_done_callback(partial(self.callback_spawn_response,x=x,y=y,theta=theta))
    
    def callback_spawn_response(self,future,x,y,theta):

        try:
            #get response from server
            response = future.result()
            self.get_logger().info(f"{response.name} created")

            #storing the turtlebot name and coordinates
            turtle_bot = TurtleBot()
            turtle_bot.name = response.name
            turtle_bot.coordinates = [x,y,theta] 

            self.msg.turtle.append(turtle_bot)
            
            #publishing alive turtles
            self.publisher_.publish(self.msg)

        except Exception as e:
            self.get_logger().error(f"{e}")
    
    def random_pose(self):

        x = random.uniform(0,11)
        y = random.uniform(0,11)
        theta = random.uniform(0,math.pi)

        return x,y,theta
    
    def callback_catch_status(self,request,response):

        if request.catch_status == True:
            response.success = "goal reached"
            self.get_logger().info(f"{response.success}")

            #pop the killed turtle
            for i,turtle in enumerate(self.msg.turtle):
                
                if turtle.name == request.name:
                    del self.msg.turtle[i]
                    break
            
            #calling service to kill the turtle
            self.callback_turtle_killer(turtle.name)

            #publishing the updated alive turtle topic
            self.publisher_.publish(self.msg)

        else:
            response.success = "goal not reached"
        
        return response

    def callback_turtle_killer(self,name):
        client = self.create_client(Kill,'kill')

        #wait for server to be available
        while not client.wait_for_service(1):
            self.get_logger().warn("kill server not available")
        
        #create request
        request = Kill.Request()
        request.name = name

        # request call
        future = client.call_async(request)
        future.add_done_callback(self.callback_response_killer)

    def callback_response_killer(self,future):
        
        try:
            self.get_logger().info("turtle killed")
        except Exception as e :
            self.get_logger().error(f"{e}")



#initialization ros communication
def main(args = None):
    rclpy.init(args=args)
    node = TurtleSpawner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
