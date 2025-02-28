from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    map_path_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Path to the map file for navigation'
    )

    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Enable simulation mode'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Enable RViz visualization'
    )

    home_csv_arg = DeclareLaunchArgument(
        'home_csv',
        default_value='home.csv',
        description='CSV file containing home position'
    )
    
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('linorobot2_navigation'),
                'launch',
                'navigation.launch.py'
            ])
        ]),
        launch_arguments={
            'sim': LaunchConfiguration('sim'),
            'rviz': LaunchConfiguration('rviz'),
            'map': LaunchConfiguration('map')
        }.items()
    )


    estimate_pose = Node(
        package='start_navigation',
        executable='start_navigation',
        name='start_navigation',
        arguments=[
            LaunchConfiguration('home_csv'),
        ],
        output='screen'
    )

    show_map = Node(
        package='start_navigation',
        executable='show_map',
        name='show_map',
        output='screen'
    )

    return LaunchDescription([
        map_path_arg,
        sim_arg,
        rviz_arg,
        navigation_launch,
        estimate_pose,
        home_csv_arg ,
        show_map
    ])