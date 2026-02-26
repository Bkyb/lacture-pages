import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Config 파일 load를 위한 작업
    package_name = 'manipulator_practice_python'    
    pkg_share_path = get_package_share_directory(package_name)
    octomap_file_path = os.path.join(pkg_share_path, 'config', 'workspace_map.bt')

    # Arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='execution',
        description='System mode: "mapping" or "execution"'
    )
    
    mode = LaunchConfiguration('mode')
    is_execution = PythonExpression(["'", mode, "' == 'execution'"])
    
    # Octomap Server Node
    octomap_server = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            'octomap_path': octomap_file_path, 
            'frame_id': 'base_link'
        }],
        condition=IfCondition(is_execution)
    )
    
    # Map Publisher Node
    dynamic_node = Node(
        package='manipulator_practice_python',
        executable='map_publisher',
        name='map_publisher',
        output='screen',
        condition=IfCondition(is_execution)
    )

    # DSR launcher
    doosanrobot_launch_file = os.path.join(
        get_package_share_directory('dsr_bringup2'),
        "launch",
        "dsr_bringup2_moveit.launch.py"
    )

    doosanrobot_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(doosanrobot_launch_file),
        launch_arguments={
            "mode":"virtual",
            # "host":"192.168.137.100",
            # "port":"12345",
            "model":"m1013",
        }.items(),
    )

    return LaunchDescription([
        mode_arg,
        octomap_server,
        dynamic_node,
        doosanrobot_node,
    ])
