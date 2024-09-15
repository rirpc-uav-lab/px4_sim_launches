import os

import launch_ros.actions as actions

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription, GroupAction
from launch_ros.actions import PushRosNamespace
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()
    
    spawn_agressivniy_drone1 = ExecuteProcess(
        cmd=[[
            'PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500 ~/PX4-Autopilot/build/px4_sitl_default/bin/px4 -i 0',
        ]],
        shell=True
    )
    
    ld.add_action(spawn_agressivniy_drone1)

    spawn_agressivniy_drone2 = ExecuteProcess(
        cmd=[[
            'PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500 PX4_GZ_MODEL_POSE="0,2" ~/PX4-Autopilot/build/px4_sitl_default/bin/px4 -i 1',
        ]],
        shell=True
    )
    
    ld.add_action(spawn_agressivniy_drone2)

    args0 = {
        'fcu_url': 'udp://:14540@localhost:14580',
        'tgt_system' : '1',
        }.items()
        

    launch_action = GroupAction([
        PushRosNamespace('uav1'),
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mavros'), 'launch/'),
            '/px4.launch']), launch_arguments=args0
        ),
    ])

    ld.add_action(launch_action)

    args1 = {
        'fcu_url': 'udp://:14541@localhost:14581',
        'tgt_system' : '2',
        }.items()
        

    launch_action = GroupAction([
        PushRosNamespace('uav2'),
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mavros'), 'launch/'),
            '/px4.launch']), launch_arguments=args1
        ),
    ])
    ld.add_action(launch_action)

    return ld