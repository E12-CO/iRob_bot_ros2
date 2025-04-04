# iRob controller launcher 
import os

import launch
import launch_ros.actions
import launch_ros.descriptions

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_name = 'irob_trajectory_server'
    
    irob_trajec_param_dir = launch.substitutions.LaunchConfiguration(
        'irob_trajectory_param_dir',
        default=os.path.join(
            get_package_share_directory(pkg_name),
            'params',
            'irob_traj_server_param.yaml'))

    irob_traj_server_instant = launch_ros.actions.Node(
        package=pkg_name,
        executable='iRob_trajectory_server',
        output='screen',
        parameters=[irob_trajec_param_dir]
    )
    
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'irob_trajectory_param_dir',
            default_value=irob_trajec_param_dir,
            description='Path to iRob trajectory server parameter yaml'),
        irob_traj_server_instant,
    ])
