import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
# Import SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the path to the waypoint_follower package
    waypoint_follower_pkg = get_package_share_directory('waypoint_follower')

    config_file_path = os.path.join(waypoint_follower_pkg, 'config', 'waypoints.yaml')

    # Get the path to the turtlebot3_gazebo package
    turtlebot3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')

    # Path to the RViz configuration file
    rviz_config = os.path.join(waypoint_follower_pkg, 'rviz', 'path_tracking.rviz')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # --- ACTION TO SET THE TURTLEBOT3 MODEL ---
    set_turtlebot_model_env = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value='waffle'  # You can change this to 'waffle' or 'waffle_pi'
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(waypoint_follower_pkg, 'launch', 'custom_world.launch.py')
        )
    )

    # Node for the trajectory generator
    traj_gen_node = Node(
        package='waypoint_follower',
        executable='traj_gen_vis_node',
        name='traj_gen_vis_node',
        output='screen',
        parameters=[config_file_path]
    )

    # Node for the trajectory tracking controller
    tracking_controller_node = Node(
        package='waypoint_follower',
        executable='tracking_controller_node',
        name='tracking_controller_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )
    
    # Node for RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        # Add the environment variable action to the launch description
        set_turtlebot_model_env,
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation (Gazebo) clock if true'),
        gazebo_launch,
        traj_gen_node,
        tracking_controller_node,
        rviz_node
    ])