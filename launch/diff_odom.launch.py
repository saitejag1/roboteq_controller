from launch import LaunchDescription 
from launch_ros.actions import Node 
 
def generate_launch_description(): 
    return LaunchDescription([ 
        Node( 
            package='roboteq_controller', 
            executable='diff_odom_node', 
            output='screen'), 
    ])
