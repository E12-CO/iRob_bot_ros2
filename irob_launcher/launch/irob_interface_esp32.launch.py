# iRob interface launcher
import os

import launch
import launch_ros.actions
import launch_ros.descriptions

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_name = 'irob_interface'
    
    irob_interface_param_dir = launch.substitutions.LaunchConfiguration(
        'irob_interface_param_dir',
        default=os.path.join(
            get_package_share_directory(pkg_name),
            'params',
            'irob_interface_mpu90.yaml'))

    irob_interface_instant = launch_ros.actions.Node(
        package=pkg_name,
        executable='iRob_interface',
        output='screen',
        parameters=[irob_interface_param_dir]
    )
    
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'irob_interface_param_dir',
            default_value=irob_interface_param_dir,
            description='Path to iRob interface parameter yaml'),
        irob_interface_instant,
    ])
