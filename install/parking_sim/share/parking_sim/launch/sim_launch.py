from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='parking_sim',executable='robot',name='front_robot',
             parameters=[{'name':'front','start_x':0.70,'start_y':0.60,'start_theta':1.5708}]),
        Node(package='parking_sim',executable='robot',name='rear_robot',
             parameters=[{'name':'rear','start_x':0.70,'start_y':0.85,'start_theta':1.5708}]),
        Node(package='parking_sim',executable='cctv',name='sim_cctv'),
        Node(package='parking_sim',executable='controller',name='nav_controller'),
        Node(package='parking_sim',executable='visualizer',name='visualizer'),
    ])
