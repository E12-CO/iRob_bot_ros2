# iRob controller launcher 
import os

import launch
import launch_ros.actions
import launch_ros.descriptions

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_name = 'irob_controller'
    
    irob_controller_param_dir = launch.substitutions.LaunchConfiguration(
        'irob_controller_param_dir',
        default=os.path.join(
            get_package_share_directory(pkg_name),
            'params',
            'irob_controller.yaml'))

    irob_controller_instant = launch_ros.actions.Node(
        package=pkg_name,
        executable='irob_controller',
        output='screen',
        parameters=[irob_controller_param_dir]
    )
    
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'irob_controller_param_dir',
            default_value=irob_controller_param_dir,
            description='Path to iRob controller parameter yaml'),
        irob_controller_instant,
    ])
