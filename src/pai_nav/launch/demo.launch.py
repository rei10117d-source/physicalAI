import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess,DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share=get_package_share_directory('pai_nav')
    world=os.path.join(share,'worlds','obstacle_course.sdf')
    return LaunchDescription([
        DeclareLaunchArgument('mode',default_value='ai',description='ai or baseline'),
        ExecuteProcess(cmd=['ign','gazebo','-r',world],output='screen'),
        Node(package='ros_ign_bridge',executable='parameter_bridge',name='bridge',output='screen',
             arguments=['/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                        '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                        '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                        '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']),
        Node(package='pai_nav',executable='nav_node',name='pai_autonomous_nav',output='screen',
             parameters=[{'mode':LaunchConfiguration('mode'),'use_sim_time':True}])
    ])
