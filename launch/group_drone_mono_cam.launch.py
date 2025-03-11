import os

import launch_ros.actions as actions

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription, GroupAction
from launch_ros.actions import PushRosNamespace
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-r agro_drones.sdf'
        }.items(),
    )

    ld.add_action(gz_sim)

    spawn_agressivniy_drone1 = ExecuteProcess(
        cmd=[[
            'PX4_SYS_AUTOSTART=4013 PX4_GZ_MODEL_NAME=uav1 ~/PX4-Autopilot/build/px4_sitl_default/bin/px4 -i 0',
        ]],
        shell=True
    )

    ld.add_action(spawn_agressivniy_drone1)


    spawn_agressivniy_drone2 = ExecuteProcess(
        cmd=[[
            'PX4_SYS_AUTOSTART=4013 PX4_GZ_MODEL_NAME=uav2 ~/PX4-Autopilot/build/px4_sitl_default/bin/px4 -i 1',
        ]],
        shell=True
    )

    ld.add_action(spawn_agressivniy_drone2)

    spawn_agressivniy_drone3 = ExecuteProcess(
        cmd=[[
            'PX4_SYS_AUTOSTART=4013 PX4_GZ_MODEL_NAME=uav3 ~/PX4-Autopilot/build/px4_sitl_default/bin/px4 -i 2',
        ]],
        shell=True
    )

    ld.add_action(spawn_agressivniy_drone3)

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[ 
                    '/world/default/model/uav1/link/mono_cam/base_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image',
                    '/world/default/model/uav1/link/mono_cam/base_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                    '/world/default/model/uav1/link/mono_cam_down/base_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image',
                    '/world/default/model/uav1/link/mono_cam_down/base_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                    '/world/default/model/uav1/link/laser_rangefinder/base_link/sensor/laser/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                    '/world/default/model/uav2/link/mono_cam/base_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image',
                    '/world/default/model/uav2/link/mono_cam/base_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                    '/world/default/model/uav2/link/mono_cam_down/base_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image',
                    '/world/default/model/uav2/link/mono_cam_down/base_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                    '/world/default/model/uav2/link/laser_rangefinder/base_link/sensor/laser/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                    '/world/default/model/uav3/link/mono_cam/base_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image',
                    '/world/default/model/uav3/link/mono_cam/base_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                    '/world/default/model/uav3/link/mono_cam_down/base_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image',
                    '/world/default/model/uav3/link/mono_cam_down/base_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                    '/world/default/model/uav3/link/laser_rangefinder/base_link/sensor/laser/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                    ],
        parameters=[
            {'qos_overrides./uav1.subscriber.reliability': 'reliable'},
            {'qos_overrides./uav2.subscriber.reliability': 'reliable'},
            {'qos_overrides./uav3.subscriber.reliability': 'reliable'},
        ],
        remappings=[
                    ('/world/default/model/uav1/link/mono_cam/base_link/sensor/imager/image', '/uav1/camera'),
                    ('/world/default/model/uav1/link/mono_cam/base_link/sensor/imager/camera_info', '/uav1/camera/camera_info'),
                    ('/world/default/model/uav1/link/mono_cam_down/base_link/sensor/imager/image', '/uav1/camera_down'),
                    ('/world/default/model/uav1/link/mono_cam_down/base_link/sensor/imager/camera_info', '/uav1/camera_down/camera_info'),
                    ('/world/default/model/uav1/link/laser_rangefinder/base_link/sensor/laser/scan', '/uav1/rangefinder'),
                    ('/world/default/model/uav2/link/mono_cam/base_link/sensor/imager/image', '/uav2/camera'),
                    ('/world/default/model/uav2/link/mono_cam/base_link/sensor/imager/camera_info', '/uav2/camera/camera_info'),
                    ('/world/default/model/uav2/link/mono_cam_down/base_link/sensor/imager/image', '/uav2/camera_down'),
                    ('/world/default/model/uav2/link/mono_cam_down/base_link/sensor/imager/camera_info', '/uav2/camera_down/camera_info'),
                    ('/world/default/model/uav2/link/laser_rangefinder/base_link/sensor/laser/scan', '/uav2/rangefinder'),
                    ('/world/default/model/uav3/link/mono_cam/base_link/sensor/imager/image', '/uav3/camera'),
                    ('/world/default/model/uav3/link/mono_cam/base_link/sensor/imager/camera_info', '/uav3/camera/camera_info'),
                    ('/world/default/model/uav3/link/mono_cam_down/base_link/sensor/imager/image', '/uav3/camera_down'),
                    ('/world/default/model/uav3/link/mono_cam_down/base_link/sensor/imager/camera_info', '/uav3/camera_down/camera_info'),
                    ('/world/default/model/uav3/link/laser_rangefinder/base_link/sensor/laser/scan', '/uav3/rangefinder')],
        output='screen'
        )
    
    ld.add_action(bridge)

    return ld