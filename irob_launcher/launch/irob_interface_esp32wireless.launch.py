# iRob interface launcher
import os

import launch
import launch_ros.actions
import launch_ros.descriptions

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    irob_if_pkg_name = 'irob_interface'
    irob_k_pkg_name = 'irob_controller'
    
    irob_interface_param_dir = launch.substitutions.LaunchConfiguration(
        'irob_interface_param_dir',
        default=os.path.join(
            get_package_share_directory(irob_if_pkg_name),
            'params',
            'irob_interface_wireless.yaml'))

    irob_interface_instant = launch_ros.actions.Node(
        package=irob_if_pkg_name,
        executable='iRob_interface_wireless',
        output='screen',
        parameters=[irob_interface_param_dir]
    )
    
    irob_controller_param_dir = launch.substitutions.LaunchConfiguration(
        'irob_controller_param_dir',
        default=os.path.join(
            get_package_share_directory(irob_k_pkg_name),
            'params',
            'irob_controller_omni3_oriIrob.yaml'))

    irob_controller_instant = launch_ros.actions.Node(
        package=irob_k_pkg_name,
        executable='iRob_controller',
        output='screen',
        parameters=[irob_controller_param_dir]
    )
    
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'irob_interface_param_dir',
            default_value=irob_interface_param_dir,
            description='Path to iRob interface parameter yaml'),
        launch.actions.DeclareLaunchArgument(
            'irob_controller_param_dir',
            default_value=irob_controller_param_dir,
            description='Path to iRob controller parameter yaml'),
        irob_interface_instant,
        irob_controller_instant,
    ])
