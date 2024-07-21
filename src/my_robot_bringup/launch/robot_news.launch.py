from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    robot_news_station_node1 = Node(
        package= "my_py_pkg",
        executable="robot_news_station",
        name= "robot_news_station_giskard",
        parameters=[
           {"robot_name":"giskard"} 
        ]
    )

    robot_news_station_node2 = Node(
        package= "my_py_pkg",
        executable="robot_news_station",
        name= "robot_news_station_bb8",
        parameters=[
           {"robot_name":"bb8"} 
        ]
    )

    robot_news_station_node3 = Node(
        package= "my_py_pkg",
        executable="robot_news_station",
        name= "robot_news_station_daneel",
        parameters=[
           {"robot_name":"daneel"} 
        ]
    )

    robot_news_station_node4 = Node(
        package= "my_py_pkg",
        executable="robot_news_station",
        name= "robot_news_station_lander",
        parameters=[
           {"robot_name":"lander"} 
        ]
    )

    robot_news_station_node5 = Node(
        package= "my_py_pkg",
        executable="robot_news_station",
        name= "robot_news_station_c3po",
        parameters=[
           {"robot_name":"c3p0"} 
        ]
    )
    
    
    smartphone_node = Node(
        package= "my_py_pkg",
        executable= "smartphone",
    )

    ld.add_action(robot_news_station_node1)
    ld.add_action(robot_news_station_node2)
    ld.add_action(robot_news_station_node3)
    ld.add_action(robot_news_station_node4)
    ld.add_action(robot_news_station_node5)
    ld.add_action(smartphone_node)

    return ld